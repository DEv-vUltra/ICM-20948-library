#!/usr/bin/env python3
"""
ICM-20948 live plotter - accel va gyro VE TACH BIET tren 2 subplot rieng,
doc du lieu CSV tu UART, PHAT HIEN mau bi rot (dropped sample) bang so
thu tu (seq) gui kem tu firmware, hien thi ro khoang trong tren do thi
thay vi am tham noi thang 2 diem cach xa nhau lam nham thanh "nhieu".

Chay:
    python3 icm_live_plot.py /dev/ttyUSB0 460800

Dinh dang firmware dang gui (8 cot, khop main_icm_polling.c hien tai):
    timestamp_ms,seq,ax,ay,az,gx,gy,gz\r\n

Neu firmware ban dang dung la ban CU HON (7 cot, khong co timestamp_ms,
chi co seq,ax,ay,az,gx,gy,gz) dung --cols 7. Neu ban cu hon nua (6 cot,
khong co seq, khong phat hien duoc rot mau) dung --cols 6.
"""

import sys
import time
import argparse
from collections import deque

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def parse_args():
    p = argparse.ArgumentParser(description="ICM-20948 live plot accel/gyro tach biet, co phat hien rot mau")
    p.add_argument("port", help="Cong serial, vi du /dev/ttyUSB0")
    p.add_argument("baud", nargs="?", type=int, default=460800,
                    help="Baudrate, phai khop voi HAL_UART_Init trong CubeMX (mac dinh 460800)")
    p.add_argument("--window", type=int, default=300,
                    help="So mau giu lai tren truc thoi gian (mac dinh 300)")
    p.add_argument("--cols", type=int, default=8, choices=(6, 7, 8),
                    help="8 = timestamp_ms+seq (mac dinh, khop main_icm_polling.c hien tai), "
                         "7 = chi seq (ban cu hon), 6 = khong co seq (khong phat hien rot mau duoc)")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Khong mo duoc cong {args.port}: {e}")
        print("Kiem tra: da cam module USB-UART chua, dung ten cong chua,")
        print("va user da o trong group 'dialout' chua (sudo usermod -aG dialout $USER, roi logout/login).")
        sys.exit(1)

    n = args.window
    t_buf = deque(maxlen=n)
    accel_buf = [deque(maxlen=n) for _ in range(3)]
    gyro_buf = [deque(maxlen=n) for _ in range(3)]

    t0 = time.time()
    last_seq = None
    total_dropped = 0
    total_received = 0

    # ---- 2 subplot TACH BIET: accel rieng, gyro rieng ----
    fig, (ax_a, ax_g) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    title_text = fig.suptitle(f"ICM-20948 accel/gyro live @ {args.port} ({args.baud} baud) | dropped: 0")

    labels = ("X", "Y", "Z")
    colors = ("tab:red", "tab:green", "tab:blue")

    lines_a = [ax_a.plot([], [], label=f"accel {l}", color=c)[0] for l, c in zip(labels, colors)]
    lines_g = [ax_g.plot([], [], label=f"gyro {l}", color=c)[0] for l, c in zip(labels, colors)]

    ax_a.set_ylabel("Accel [g]")
    ax_g.set_ylabel("Gyro [dps]")
    ax_g.set_xlabel("Thời gian [s]")

    for ax in (ax_a, ax_g):
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    def parse_line(parts):
        """Tra ve (seq_or_None, [ax,ay,az,gx,gy,gz]) tuy theo --cols."""
        if args.cols == 8:
            # timestamp_ms, seq, ax, ay, az, gx, gy, gz
            seq = int(parts[1])
            values = [float(x) for x in parts[2:8]]
        elif args.cols == 7:
            # seq, ax, ay, az, gx, gy, gz
            seq = int(parts[0])
            values = [float(x) for x in parts[1:7]]
        else:
            # ax, ay, az, gx, gy, gz (khong co seq)
            seq = None
            values = [float(x) for x in parts[:6]]
        return seq, values

    def read_pending_lines():
        nonlocal last_seq, total_dropped, total_received
        new_samples = 0
        while ser.in_waiting > 0:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue
            parts = raw.split(",")
            if len(parts) != args.cols:
                continue  # dong loi / le thuc su (garble UART) hoac lech --cols, bo qua

            try:
                seq, values = parse_line(parts)
            except (ValueError, IndexError):
                continue

            total_received += 1

            # ---- Phat hien rot mau bang seq, CHEN KHOANG TRONG (NaN) ----
            if seq is not None and last_seq is not None:
                gap = seq - last_seq - 1
                if gap > 0:
                    total_dropped += gap
                    now_gap = time.time() - t0
                    t_buf.append(now_gap)
                    for i in range(3):
                        accel_buf[i].append(float("nan"))
                        gyro_buf[i].append(float("nan"))
            if seq is not None:
                last_seq = seq

            now = time.time() - t0
            t_buf.append(now)
            for i in range(3):
                accel_buf[i].append(values[i])
                gyro_buf[i].append(values[3 + i])
            new_samples += 1
        return new_samples

    def update(_frame):
        read_pending_lines()
        if not t_buf:
            return lines_a + lines_g + [title_text]

        t_list = list(t_buf)
        for i in range(3):
            lines_a[i].set_data(t_list, list(accel_buf[i]))
            lines_g[i].set_data(t_list, list(gyro_buf[i]))

        for ax in (ax_a, ax_g):
            ax.relim()
            ax.autoscale_view()

        total = total_received + total_dropped
        drop_pct = (100.0 * total_dropped / total) if total > 0 else 0.0
        title_text.set_text(
            f"ICM-20948 accel/gyro live @ {args.port} ({args.baud} baud) | "
            f"dropped: {total_dropped} ({drop_pct:.1f}%)"
        )

        return lines_a + lines_g + [title_text]

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()

    ser.close()
    total = total_received + total_dropped
    print(f"Tổng kết: nhận {total_received} mẫu, rớt {total_dropped} mẫu "
          f"({100.0 * total_dropped / max(1, total):.2f}%).")


if __name__ == "__main__":
    main()