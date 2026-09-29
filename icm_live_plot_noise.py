#!/usr/bin/env python3
"""
ICM-20948 live plotter (accel + gyro) - doc du lieu CSV tu UART, ve 2 do
thi theo thoi gian thuc, VA phat hien mau bi rot (dropped sample) bang
so thu tu (sequence number) gui kem tu firmware, thay vi am tham noi
thang 2 diem cach xa nhau lam nham thanh "nhieu dang xung vuong".

Chay:
    python3 icm_live_plot.py /dev/ttyUSB0 460800

Yeu cau firmware gui moi dong theo dinh dang CSV 7 cot:
    seq,ax,ay,az,gx,gy,gz\r\n
(seq: so nguyen tang dan tu g_sample_count; accel [g]; gyro [dps])

Neu firmware van dang gui dinh dang cu (6 cot, khong co seq) dung
--cols 6, luc do khong the phat hien rot mau, chi ve don thuan.
"""

import sys
import time
import argparse
from collections import deque

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def parse_args():
    p = argparse.ArgumentParser(description="ICM-20948 live plot (accel/gyro) qua UART, co phat hien rot mau")
    p.add_argument("port", help="Cong serial, vi du /dev/ttyUSB0")
    p.add_argument("baud", nargs="?", type=int, default=460800,
                    help="Baudrate, phai khop voi HAL_UART_Init trong CubeMX (mac dinh 460800)")
    p.add_argument("--window", type=int, default=300,
                    help="So mau giu lai tren truc thoi gian (mac dinh 300)")
    p.add_argument("--cols", type=int, default=7, choices=(6, 7),
                    help="7 = co seq (mac dinh, phat hien duoc rot mau), 6 = dinh dang cu khong co seq")
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

    fig, (ax_a, ax_g) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    title_text = fig.suptitle(f"ICM-20948 accel/gyro live @ {args.port} ({args.baud} baud) | dropped: 0")

    labels = ("X", "Y", "Z")
    colors = ("tab:red", "tab:green", "tab:blue")

    lines_a = [ax_a.plot([], [], label=f"accel {l}", color=c)[0] for l, c in zip(labels, colors)]
    lines_g = [ax_g.plot([], [], label=f"gyro {l}", color=c)[0] for l, c in zip(labels, colors)]

    ax_a.set_ylabel("Accel [g]")
    ax_g.set_ylabel("Gyro [dps]")
    ax_g.set_xlabel("Thoi gian [s]")

    for ax in (ax_a, ax_g):
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    def read_pending_lines():
        nonlocal last_seq, total_dropped, total_received
        new_samples = 0
        while ser.in_waiting > 0:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue
            parts = raw.split(",")
            if len(parts) != args.cols:
                continue  # dong loi / le thuc su (garble UART), bo qua

            try:
                if args.cols == 7:
                    seq = int(parts[0])
                    values = [float(p) for p in parts[1:7]]
                else:
                    seq = None
                    values = [float(p) for p in parts[:6]]
            except ValueError:
                continue

            total_received += 1

            # ---- Phat hien rot mau bang seq, CHEN KHOANG TRONG (NaN) ----
            # thay vi noi thang - matplotlib se hien thi dut doan ro rang
            # tai dung cho bi mat mau, khong lam nham thanh nhieu.
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

        drop_pct = (100.0 * total_dropped / (total_received + total_dropped)) if (total_received + total_dropped) > 0 else 0.0
        title_text.set_text(
            f"ICM-20948 accel/gyro live @ {args.port} ({args.baud} baud) | "
            f"dropped: {total_dropped} ({drop_pct:.1f}%)"
        )

        return lines_a + lines_g + [title_text]

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()

    ser.close()
    print(f"Tong ket: nhan {total_received} mau, rot {total_dropped} mau "
          f"({100.0 * total_dropped / max(1, total_received + total_dropped):.2f}%).")


if __name__ == "__main__":
    main()
