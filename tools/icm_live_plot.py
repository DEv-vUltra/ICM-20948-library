#!/usr/bin/env python3
"""
ICM-20948 live plotter: accel and gyro on two separate subplots, read from the
UART CSV stream. Lost lines are detected from the sequence number (seq) and
shown as a break in the trace, instead of silently joining two distant points.

Run:
    python3 tools/icm_live_plot.py /dev/ttyUSB0 460800

Firmware line format (8 columns, see examples/main_example.c):
    timestamp_ms,seq,ax,ay,az,gx,gy,gz\\r\\n

Use --cols 7 for older firmware without timestamp_ms (seq,ax,...,gz) and
--cols 6 for firmware without seq either (lost lines cannot be detected).

X axis: the firmware timestamp when available (--cols 8), otherwise the PC
receive time (which is bunched, because lines arrive in bursts).

Serial reading runs in a background thread so a slow or partial line can never
freeze the GUI.
"""

import argparse
import math
import queue
import sys
import threading
import time
from collections import deque

import serial

MAX_ITEMS_PER_FRAME = 2000      # cap work per GUI refresh so the plot never falls behind


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
def parse_line(raw, cols):
    """Return (ts_ms_or_None, seq_or_None, [ax, ay, az, gx, gy, gz]) or None if malformed."""
    parts = raw.split(",")
    if len(parts) != cols:
        return None
    try:
        if cols == 8:
            ts_ms, seq, values = int(parts[0]), int(parts[1]), [float(x) for x in parts[2:8]]
        elif cols == 7:
            ts_ms, seq, values = None, int(parts[0]), [float(x) for x in parts[1:7]]
        else:
            ts_ms, seq, values = None, None, [float(x) for x in parts[:6]]
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in values):
        return None
    return ts_ms, seq, values


# ---------------------------------------------------------------------------
# Rolling buffers + loss accounting (no serial / matplotlib dependency)
# ---------------------------------------------------------------------------
class StreamBuffers:
    def __init__(self, window):
        self.t = deque(maxlen=window)
        self.accel = [deque(maxlen=window) for _ in range(3)]
        self.gyro = [deque(maxlen=window) for _ in range(3)]
        self.received = 0
        self.lost = 0
        self.malformed = 0
        self.resets = 0
        self._last_seq = None
        self._last_ts_ms = None
        self._origin_ms = None
        self._base_ms = 0.0
        self._last_t_s = None

    def _append_break(self, t_s):
        """One NaN point makes matplotlib break the line at that time."""
        self.t.append(t_s)
        for i in range(3):
            self.accel[i].append(float("nan"))
            self.gyro[i].append(float("nan"))

    def ingest(self, ts_ms, seq, values, pc_time_s):
        restart = False

        # X axis: firmware time if present, else PC receive time.
        if ts_ms is not None:
            if self._origin_ms is None:
                self._origin_ms = ts_ms
            elif ts_ms < self._last_ts_ms:            # firmware restarted
                restart = True
                self._base_ms = (self._last_t_s or 0.0) * 1000.0
                self._origin_ms = ts_ms
            t_s = (self._base_ms + (ts_ms - self._origin_ms)) / 1000.0
            self._last_ts_ms = ts_ms
        else:
            t_s = pc_time_s

        # Loss detection from the sequence number.
        gap = 0
        if seq is not None and self._last_seq is not None:
            if seq > self._last_seq:
                gap = seq - self._last_seq - 1
            elif seq < self._last_seq:                # counter went backwards
                restart = True
        if seq is not None:
            self._last_seq = seq

        if restart:
            self.resets += 1
        self.lost += gap

        if (gap > 0 or restart) and self._last_t_s is not None:
            self._append_break(0.5 * (self._last_t_s + t_s))

        self.t.append(t_s)
        for i in range(3):
            self.accel[i].append(values[i])
            self.gyro[i].append(values[3 + i])
        self._last_t_s = t_s
        self.received += 1

    def loss_percent(self):
        total = self.received + self.lost
        return (100.0 * self.lost / total) if total > 0 else 0.0


# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------
def reader_thread(ser, cols, out_queue, stop_event, t0):
    """Put (ts_ms, seq, values, rx_s) tuples, the string 'bad' for a malformed line,
    or None on a serial error into out_queue."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except (serial.SerialException, OSError):
            out_queue.put(None)
            return
        if not raw:
            continue
        rx_s = time.time() - t0
        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        parsed = parse_line(line, cols)
        if parsed is None:
            out_queue.put("bad")
        else:
            out_queue.put(parsed + (rx_s,))


def parse_args():
    p = argparse.ArgumentParser(description="ICM-20948 live plot (accel/gyro) with lost-line detection")
    p.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0")
    p.add_argument("baud", nargs="?", type=int, default=460800,
                   help="Baud rate (must match the firmware, default 460800)")
    p.add_argument("--window", type=int, default=300,
                   help="Number of rows kept on screen (default 300)")
    p.add_argument("--cols", type=int, default=8, choices=(6, 7, 8),
                   help="8 = timestamp_ms,seq,... (default); 7 = seq,... ; 6 = no seq")
    return p.parse_args()


def main():
    args = parse_args()

    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    try:
        ser = serial.serial_for_url(args.port, baudrate=args.baud, timeout=0.5)
    except (serial.SerialException, ValueError, OSError) as e:
        print(f"Cannot open {args.port}: {e}")
        print("Check the USB-UART module, the port name, and that your user is in the "
              "'dialout' group (sudo usermod -aG dialout $USER, then log out and in).")
        sys.exit(1)
    ser.reset_input_buffer()

    t0 = time.time()
    buffers = StreamBuffers(args.window)
    items = queue.SimpleQueue()
    stop_event = threading.Event()
    serial_error = {"flag": False}
    thread = threading.Thread(target=reader_thread, args=(ser, args.cols, items, stop_event, t0),
                              daemon=True)
    thread.start()

    fig, (ax_a, ax_g) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    title = fig.suptitle("")
    labels = ("X", "Y", "Z")
    colors = ("tab:red", "tab:green", "tab:blue")
    lines_a = [ax_a.plot([], [], label=f"accel {l}", color=c)[0] for l, c in zip(labels, colors)]
    lines_g = [ax_g.plot([], [], label=f"gyro {l}", color=c)[0] for l, c in zip(labels, colors)]
    ax_a.set_ylabel("Accel [g]")
    ax_g.set_ylabel("Gyro [dps]")
    ax_g.set_xlabel("Firmware time [s]" if args.cols == 8 else "PC receive time [s]")
    for ax in (ax_a, ax_g):
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    def update(_frame):
        for _ in range(MAX_ITEMS_PER_FRAME):
            try:
                item = items.get_nowait()
            except queue.Empty:
                break
            if item is None:
                serial_error["flag"] = True
                break
            if isinstance(item, str):
                buffers.malformed += 1
                continue
            ts_ms, seq, values, rx_s = item
            buffers.ingest(ts_ms, seq, values, rx_s)

        status = (f"lost {buffers.lost} ({buffers.loss_percent():.1f}%) | "
                  f"malformed {buffers.malformed} | restarts {buffers.resets}")
        if serial_error["flag"]:
            status += " | SERIAL ERROR"
        title.set_text(f"ICM-20948 live @ {args.port} ({args.baud} baud) | {status}")

        if buffers.t:
            t_list = list(buffers.t)
            for i in range(3):
                lines_a[i].set_data(t_list, list(buffers.accel[i]))
                lines_g[i].set_data(t_list, list(buffers.gyro[i]))
            for ax in (ax_a, ax_g):
                ax.relim()
                ax.autoscale_view()
        return lines_a + lines_g + [title]

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    stop_event.set()
    thread.join(timeout=1.0)
    ser.close()
    del ani
    print(f"Summary: {buffers.received} rows received, {buffers.lost} lost "
          f"({buffers.loss_percent():.2f}%), {buffers.malformed} malformed, "
          f"{buffers.resets} firmware restarts.")


if __name__ == "__main__":
    main()