#!/usr/bin/env python3
"""
ICM-20948 data logger - ghi thang du lieu tho tu UART ra file CSV, KHONG
ve do thi (tranh matplotlib lam cham/jitter qua trinh doc serial, quan
trong cho du lieu se dung phan tich Allan deviation/delay sau nay).

Chay (vi du log 120 giay, dung yen tuyet doi ca IMU trong luc log):
    python3 icm_logger.py /dev/ttyUSB0 460800 --duration 120 --out static_test.csv

Dinh dang firmware dang gui (7 cot):
    timestamp_ms,seq,ax,ay,az,gx,gy,gz\r\n

File CSV dau ra them 1 cot rx_time_s (thoi diem PC nhan dong, epoch giay,
chi de doi chieu/debug UART - KHONG dung cot nay cho phan tich Allan, vi
no co jitter tu he dieu hanh/USB. Phan tich chinh dung timestamp_ms va
seq tu firmware.
"""

import sys
import time
import argparse
import csv

import serial


def parse_args():
    p = argparse.ArgumentParser(description="Log du lieu ICM-20948 tho ra CSV")
    p.add_argument("port", help="Cong serial, vi du /dev/ttyUSB0")
    p.add_argument("baud", nargs="?", type=int, default=460800)
    p.add_argument("--duration", type=float, default=60.0,
                    help="Thoi gian log (giay), mac dinh 60s. Voi Allan deviation, "
                         "cang dai cang tot (>=120s de thay ro vung bias instability).")
    p.add_argument("--out", type=str, default="icm_log.csv", help="File CSV dau ra")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Khong mo duoc cong {args.port}: {e}")
        sys.exit(1)

    print(f"Dang log vao '{args.out}' trong {args.duration:.0f}s ... "
          f"GIU IMU DUNG YEN TUYET DOI tren mat phang cung trong suot qua trinh nay.")
    print("Nhan Ctrl+C de dung som.")

    t_start = time.time()
    n_written = 0
    n_malformed = 0
    last_seq = None
    n_dropped = 0

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["rx_time_s", "timestamp_ms", "seq", "ax_g", "ay_g", "az_g",
                          "gx_dps", "gy_dps", "gz_dps"])

        try:
            while (time.time() - t_start) < args.duration:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue
                parts = raw.split(",")
                if len(parts) != 8:
                    n_malformed += 1
                    continue
                try:
                    ts_ms = int(parts[0])
                    seq = int(parts[1])
                    values = [float(x) for x in parts[2:8]]
                except ValueError:
                    n_malformed += 1
                    continue

                if last_seq is not None:
                    gap = seq - last_seq - 1
                    if gap > 0:
                        n_dropped += gap
                last_seq = seq

                rx_time_s = time.time()
                writer.writerow([f"{rx_time_s:.6f}", ts_ms, seq] + values)
                n_written += 1

                if n_written % 200 == 0:
                    elapsed = time.time() - t_start
                    print(f"\r  {elapsed:5.1f}s | mau: {n_written} | rot: {n_dropped} | "
                          f"loi dong: {n_malformed}", end="", flush=True)
        except KeyboardInterrupt:
            print("\nDung som theo yeu cau.")

    ser.close()
    total = n_written + n_dropped
    drop_pct = (100.0 * n_dropped / total) if total > 0 else 0.0
    print(f"\n\nHoan tat: {n_written} mau ghi vao '{args.out}', "
          f"{n_dropped} mau rot ({drop_pct:.2f}%), {n_malformed} dong loi.")
    if drop_pct > 1.0:
        print("CANH BAO: ty le rot mau > 1% - ket qua phan tich Allan/delay ben duoi "
              "co the khong chinh xac. Xem lai I2C speed / ICM_SAMPLE_PERIOD_MS truoc "
              "khi tin tuong ket qua phan tich.")


if __name__ == "__main__":
    main()
