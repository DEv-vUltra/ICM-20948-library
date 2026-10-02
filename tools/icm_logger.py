#!/usr/bin/env python3
"""
ICM-20948 data logger: records the UART stream to a CSV file without
plotting (a GUI would add jitter to the serial read).

Example (120 s, IMU completely still, after a 60 s warm-up):
    python3 tools/icm_logger.py /dev/ttyUSB0 460800 --warmup 60 --duration 120 \
        --out static.csv --odr-hz 51 --dlpf-accel-hz 6 --dlpf-gyro-hz 6

Firmware line format (8 columns):
    timestamp_ms,seq,ax,ay,az,gx,gy,gz\\r\\n
    (timestamp_ms: HAL_GetTick(), seq: counter, accel in g, gyro in dps)

Output files:
    <out>           CSV: rx_time_s,timestamp_ms,seq,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps
    <out>.meta.json Run metadata (the settings you pass + counters). It is read
                    automatically by icm_data_quality.py. The extension of <out>
                    is replaced, e.g. static.csv -> static.meta.json.

Notes:
  * rx_time_s is the PC receive time. It jitters with the OS/USB stack and is for
    debugging only; the analysis uses timestamp_ms and seq from the firmware.
  * "lost" counts gaps in seq. A line corrupted on the wire is also a gap, so it
    is counted both as malformed and as lost.
  * The port argument also accepts pySerial URLs (e.g. socket://host:port).
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import serial

EXPECTED_COLS = 8
CSV_HEADER = ["rx_time_s", "timestamp_ms", "seq", "ax_g", "ay_g", "az_g",
              "gx_dps", "gy_dps", "gz_dps"]
FLUSH_EVERY_ROWS = 200


def parse_args():
    p = argparse.ArgumentParser(description="Log raw ICM-20948 UART data to CSV")
    p.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0")
    p.add_argument("baud", nargs="?", type=int, default=460800,
                   help="Baud rate (must match the firmware, default 460800)")
    p.add_argument("--duration", type=float, default=60.0,
                   help="Recording time in seconds (default 60). The white-noise region of the "
                        "Allan plot needs minutes; the bias-instability minimum of a MEMS gyro "
                        "typically needs tens of minutes.")
    p.add_argument("--warmup", type=float, default=0.0,
                   help="Seconds to discard before recording, so the sensor can reach thermal "
                        "equilibrium (default 0)")
    p.add_argument("--out", type=str, default="icm_log.csv", help="Output CSV file")

    meta = p.add_argument_group("sensor settings stored in the metadata file (optional)")
    meta.add_argument("--odr-hz", type=float, default=None, help="Configured sensor output rate [Hz]")
    meta.add_argument("--dlpf-accel-hz", type=float, default=None, help="Accel DLPF cut-off [Hz]")
    meta.add_argument("--dlpf-gyro-hz", type=float, default=None, help="Gyro DLPF cut-off [Hz]")
    meta.add_argument("--accel-fs-g", type=float, default=None, help="Accel full-scale range [g]")
    meta.add_argument("--gyro-fs-dps", type=float, default=None, help="Gyro full-scale range [dps]")
    meta.add_argument("--note", type=str, default="", help="Free-text note (surface, orientation, ...)")
    return p.parse_args()


def parse_row(raw):
    """Parse 'timestamp_ms,seq,ax,ay,az,gx,gy,gz'. Return (ts_ms, seq, values) or None."""
    parts = raw.split(",")
    if len(parts) != EXPECTED_COLS:
        return None
    try:
        ts_ms = int(parts[0])
        seq = int(parts[1])
        values = [float(x) for x in parts[2:8]]
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in values):
        return None
    return ts_ms, seq, values


def open_port(port, baud):
    try:
        return serial.serial_for_url(port, baudrate=baud, timeout=1)
    except (serial.SerialException, ValueError, OSError) as e:
        print(f"Cannot open {port}: {e}")
        print("Check the USB-UART module, the port name, and that your user is in the "
              "'dialout' group (sudo usermod -aG dialout $USER, then log out and in).")
        sys.exit(1)


def write_metadata(out_path, args, stats):
    meta_path = os.path.splitext(out_path)[0] + ".meta.json"
    meta = {
        "csv": os.path.basename(out_path),
        "columns": CSV_HEADER,
        "port": args.port,
        "baud": args.baud,
        "start_epoch_s": stats["t_record_start"],
        "warmup_s": args.warmup,
        "duration_requested_s": args.duration,
        "duration_actual_s": round(stats["elapsed"], 3),
        "rows": stats["rows"],
        "lost_by_seq": stats["lost"],
        "malformed_lines": stats["malformed"],
        "seq_resets": stats["resets"],
        "odr_hz": args.odr_hz,
        "dlpf_accel_hz": args.dlpf_accel_hz,
        "dlpf_gyro_hz": args.dlpf_gyro_hz,
        "accel_fs_g": args.accel_fs_g,
        "gyro_fs_dps": args.gyro_fs_dps,
        "note": args.note,
    }
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)
    return meta_path


def main():
    args = parse_args()
    ser = open_port(args.port, args.baud)
    ser.reset_input_buffer()          # drop anything buffered before we started

    if args.warmup > 0:
        print(f"Warm-up: discarding data for {args.warmup:.0f} s ...")
        t_end = time.time() + args.warmup
        while time.time() < t_end:
            ser.readline()
        ser.reset_input_buffer()

    print(f"Logging to '{args.out}' for {args.duration:.0f} s ... "
          f"keep the IMU completely still on a rigid surface.")
    print("Press Ctrl+C to stop early.")

    t_record_start = time.time()
    rows = 0
    malformed = 0
    lost = 0
    resets = 0
    last_seq = None

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        try:
            while (time.time() - t_record_start) < args.duration:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue
                parsed = parse_row(raw)
                if parsed is None:
                    malformed += 1
                    continue
                ts_ms, seq, values = parsed

                if last_seq is not None:
                    if seq > last_seq:
                        lost += seq - last_seq - 1
                    elif seq < last_seq:
                        resets += 1           # MCU restarted: not a loss
                last_seq = seq

                writer.writerow([f"{time.time():.6f}", ts_ms, seq] + values)
                rows += 1

                if rows % FLUSH_EVERY_ROWS == 0:
                    f.flush()
                    elapsed = time.time() - t_record_start
                    print(f"\r  {elapsed:6.1f} s | rows: {rows} | lost: {lost} | "
                          f"malformed: {malformed}", end="", flush=True)
        except KeyboardInterrupt:
            print("\nStopped early.")

    ser.close()
    elapsed = time.time() - t_record_start
    total = rows + lost
    lost_pct = (100.0 * lost / total) if total > 0 else 0.0

    meta_path = write_metadata(args.out, args, {
        "t_record_start": t_record_start, "elapsed": elapsed, "rows": rows,
        "lost": lost, "malformed": malformed, "resets": resets,
    })

    print(f"\n\nDone: {rows} rows written to '{args.out}' (metadata: '{meta_path}').")
    print(f"      lost (seq gaps): {lost} ({lost_pct:.2f}%), malformed lines: {malformed}, "
          f"firmware restarts seen: {resets}")
    if rows == 0:
        print("WARNING: no valid rows received. Check the baud rate and the firmware line format "
              "(8 columns: timestamp_ms,seq,ax,ay,az,gx,gy,gz).")
    if lost_pct > 1.0:
        print("WARNING: more than 1% of lines were lost; Allan/delay results may be unreliable. "
              "Check the UART baud rate, cabling and the firmware polling period.")
    if resets:
        print("WARNING: seq went backwards (firmware restart). The analysis splits the log at such points.")


if __name__ == "__main__":
    main()