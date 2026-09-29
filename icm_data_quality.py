#!/usr/bin/env python3
"""
ICM-20948 Data Quality Analyzer - danh gia du lieu accel/gyro co du "sach"
de dua vao thuat toan sensor fusion (complementary/Mahony/EKF/ESKF) hay
khong, dua tren 3 nhom phan tich:

  1. ALLAN DEVIATION (chuan IEEE Std 952 cho gyro, ap dung tuong tu cho
     accel) - tach bach duoc White Noise (ARW/VRW) khoi Bias Instability
     khoi Rate Random Walk (troi dai han). Day la phuong phap chuan cong
     nghiep de dinh luong nhieu IMU, KHONG phai doan mo bang mat.

  2. DO TRE (GROUP DELAY) cua DLPF - uoc luong 2 cach:
     a) Cong thuc xap xi (bo loc 1 cuc): delay ~ 1/(2*pi*fc). CHI la xap
        xi, vi DLPF that cua ICM la bo loc bac cao hon. Muon so chinh xac
        tuyet doi, tra bang Group Delay trong datasheet ICM-20948 (Table
        ung voi tung muc DLPF ban chon).
     b) Do tre TUONG DOI thuc do bang cross-correlation giua goc nghieng
        tinh tu gyro (tich phan) va goc nghieng tinh tu accel (atan2),
        trong 1 doan co chuyen dong that su trong file log. Day moi la so
        QUAN TRONG NHAT cho sensor fusion: neu 2 kenh lech pha nhau nhieu,
        bo loc fusion (Mahony/EKF) se phai "danh nhau" giua 2 nguon tin.

  3. KIEM TRA VAT LY ACCELEROMETER - ||a|| phai ~1g khi dung yen. Diem
     lech qua nguong duoc gan nhan la "gia toc ngoai" (external
     acceleration) - phan biet voi nhieu thuan tuy.

CACH DUNG:
    # Buoc 1: log du lieu TINH (dung yen tuyet doi, cang lau cang tot,
    # toi thieu 60s, ly tuong >=300s de thay ro bias instability):
    python3 icm_logger.py /dev/ttyUSB0 460800 --duration 120 --out static.csv

    # Buoc 2 (TUY CHON, de do delay tuong doi): log them 1 file rieng
    # trong luc CHUYEN DONG CHAM, DEU DAN (vi du nghieng qua lai tay 1
    # truc trong ~1Hz, bien do vua phai):
    python3 icm_logger.py /dev/ttyUSB0 460800 --duration 20 --out motion.csv

    # Buoc 3: phan tich
    python3 icm_data_quality.py --static static.csv --motion motion.csv \
        --dlpf-accel-hz 246 --dlpf-gyro-hz 151 --odr-hz 225
"""

import argparse
import sys

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# 1. ALLAN DEVIATION (non-overlapping cluster method - chuan, don gian, on
#    dinh so hoc). Ap dung truc tiep tren tin hieu (rate cho gyro, gia
#    tri accel cho accel) - day la cach lam pho bien trong datasheet IMU
#    thuong dan (bias instability tinh truc tiep tren tin hieu goc, khong
#    bat buoc phai tich phan sang goc/van toc truoc).
# ---------------------------------------------------------------------------
def allan_deviation(x, dt, n_points=60):
    """
    x: mang 1D du lieu deu nhau theo thoi gian (vi du gyro_x tinh bang dps)
    dt: chu ky lay mau danh nghia (giay) - DUNG GIA TRI DANH NGHIA TU ODR
        cau hinh phan cung (vd 1/225), KHONG dung dt do tu timestamp UART
        (vi ODR tu chip la crystal-accurate, chinh xac hon nhieu so voi
        timestamp_ms 1ms-resolution qua UART).
    Tra ve: (tau_array, adev_array)
    """
    n = len(x)
    if n < 16:
        raise ValueError("Can it nhat 16 mau de tinh Allan deviation.")

    max_m = n // 4  # can it nhat ~4 cluster de uoc luong co y nghia
    m_list = np.unique(np.logspace(0, np.log10(max_m), n_points).astype(int))
    m_list = m_list[m_list >= 1]

    taus = []
    adevs = []
    for m in m_list:
        n_clusters = n // m
        if n_clusters < 3:
            continue
        trimmed = x[: n_clusters * m]
        cluster_means = trimmed.reshape(n_clusters, m).mean(axis=1)
        diffs = np.diff(cluster_means)
        avar = 0.5 * np.mean(diffs ** 2)
        taus.append(m * dt)
        adevs.append(np.sqrt(avar))

    return np.array(taus), np.array(adevs)


def extract_noise_params(taus, adevs):
    """
    Tra ve dict:
      arw        : gia tri ADEV tai vung doc doc -1/2 (quy ve tau=1s) -
                   xap xi bang ADEV tai tau gan 1s nhat co san.
      bias_instab: gia tri nho nhat cua duong ADEV (diem day/flat vung
                   giua) va tau tuong ung.
    """
    idx_min = int(np.argmin(adevs))
    bias_instability = adevs[idx_min]
    tau_bias = taus[idx_min]

    idx_tau1 = int(np.argmin(np.abs(taus - 1.0)))
    arw_like = adevs[idx_tau1]
    tau_used = taus[idx_tau1]

    return {
        "bias_instability": bias_instability,
        "tau_bias_instability_s": tau_bias,
        "arw_like": arw_like,
        "tau_arw_used_s": tau_used,
    }


# ---------------------------------------------------------------------------
# 2b. Do tre TUONG DOI giua accel va gyro qua cross-correlation
# ---------------------------------------------------------------------------
def relative_delay_via_xcorr(t, accel_tilt_deg, gyro_tilt_deg, dt, max_lag_s=0.5):
    """
    Tim do lech thoi gian (giay) giua 2 chuoi tin hieu goc-nghieng bang
    cross-correlation. Gia tri duong nghia la accel_tilt TRE hon so voi
    gyro_tilt (dung ky vong khi DLPF accel thap hon / nhieu hon gyro).
    """
    a = accel_tilt_deg - np.mean(accel_tilt_deg)
    g = gyro_tilt_deg - np.mean(gyro_tilt_deg)

    max_lag_samples = int(max_lag_s / dt)
    corr = np.correlate(a, g, mode="full")
    lags = np.arange(-len(g) + 1, len(a))

    center = len(lags) // 2
    lo = max(0, center - max_lag_samples)
    hi = min(len(lags), center + max_lag_samples)

    corr_window = corr[lo:hi]
    lags_window = lags[lo:hi]

    best_idx = int(np.argmax(corr_window))
    best_lag_samples = lags_window[best_idx]
    return best_lag_samples * dt


# ---------------------------------------------------------------------------
# Ham phu
# ---------------------------------------------------------------------------
def load_log(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data


def analytical_group_delay_s(fc_hz):
    """Xap xi bo loc 1 cuc: delay ~ 1/(2*pi*fc). Chi la CAN DUOI tham khao,
    DLPF thuc cua ICM la bac cao hon nen delay thuc te CO THE LON HON so
    voi con so nay - xem datasheet de co so chinh xac."""
    return 1.0 / (2.0 * np.pi * fc_hz)


def fusion_readiness_verdict(gyro_bias_instab_dps, accel_bias_instab_g, relative_delay_ms):
    """In danh gia dinh tinh, KHONG phai pass/fail cung nhac - chi la
    khung tham chieu de ban tu quyet dinh."""
    lines = []
    gyro_bias_dph = gyro_bias_instab_dps * 3600.0  # deg/s -> deg/hr xap xi

    lines.append(f"  Gyro bias instability  ~ {gyro_bias_instab_dps:.5f} dps "
                 f"(~{gyro_bias_dph:.2f} deg/hr)")
    if gyro_bias_dph < 10:
        lines.append("    -> Muc 'tactical grade' (rat tot cho MEMS gia re)")
    elif gyro_bias_dph < 50:
        lines.append("    -> Muc 'industrial grade' (on cho da so ung dung UAV amatuer)")
    else:
        lines.append("    -> Muc 'consumer grade' - can EKF/ESKF uoc luong bias lien tuc "
                     "de bu, khong the dung truc tiep")

    lines.append(f"  Accel bias instability ~ {accel_bias_instab_g * 1000:.3f} mg")

    lines.append(f"  Do tre tuong doi accel vs gyro ~ {relative_delay_ms:.1f} ms")
    if abs(relative_delay_ms) < 5:
        lines.append("    -> Rat tot, gan nhu dong bo, fusion filter se on dinh")
    elif abs(relative_delay_ms) < 20:
        lines.append("    -> Chap nhan duoc cho complementary/Mahony filter thong thuong, "
                     "nhung EKF/ESKF chat che nen mo hinh hoa do tre nay hoac bu bang "
                     "noi suy/timestamp alignment")
    else:
        lines.append("    -> KHA LON - fusion filter co the dao dong/tre phan hoi ro ret. "
                     "Nen giam DLPF (tang bang thong) hoac bu tre bang buffer/interpolation "
                     "truoc khi dua vao EKF/ESKF")

    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description="Danh gia chat luong du lieu ICM-20948 cho sensor fusion")
    p.add_argument("--static", required=True, help="File CSV log luc IMU dung yen (tu icm_logger.py)")
    p.add_argument("--motion", default=None,
                   help="File CSV log luc IMU chuyen dong cham/deu (tuy chon, de do delay tuong doi)")
    p.add_argument("--odr-hz", type=float, required=True, help="ODR danh nghia da cau hinh (vd 225)")
    p.add_argument("--dlpf-accel-hz", type=float, default=None, help="DLPF cutoff accel (Hz), de tinh delay xap xi")
    p.add_argument("--dlpf-gyro-hz", type=float, default=None, help="DLPF cutoff gyro (Hz), de tinh delay xap xi")
    p.add_argument("--external-accel-threshold-g", type=float, default=0.05,
                   help="Nguong lech |a|-1g de gan nhan gia toc ngoai (mac dinh 0.05g)")
    args = p.parse_args()

    dt_nominal = 1.0 / args.odr_hz

    print("=" * 78)
    print(f"Doc file tinh: {args.static}")
    static_data = load_log(args.static)
    n_static = len(static_data)
    print(f"  {n_static} mau, dt danh nghia = {dt_nominal * 1000:.3f} ms "
          f"(tong thoi gian ~ {n_static * dt_nominal:.1f}s)")

    if n_static < 64:
        print("CANH BAO: qua it mau, ket qua Allan deviation se khong dang tin cay. "
              "Nen log toi thieu vai chuc giay, ly tuong >=120s.")

    axes = ["x", "y", "z"]
    gyro_cols = [static_data[f"gx_dps"], static_data[f"gy_dps"], static_data[f"gz_dps"]]
    accel_cols = [static_data[f"ax_g"], static_data[f"ay_g"], static_data[f"az_g"]]

    # ---- 1. Allan deviation cho tung truc gyro va accel ----
    fig1, (ax_g, ax_a) = plt.subplots(1, 2, figsize=(13, 5))
    gyro_params = []
    accel_params = []

    for i, axis in enumerate(axes):
        tau_g, adev_g = allan_deviation(gyro_cols[i], dt_nominal)
        params_g = extract_noise_params(tau_g, adev_g)
        gyro_params.append(params_g)
        ax_g.loglog(tau_g, adev_g, label=f"gyro {axis}")

        tau_a, adev_a = allan_deviation(accel_cols[i], dt_nominal)
        params_a = extract_noise_params(tau_a, adev_a)
        accel_params.append(params_a)
        ax_a.loglog(tau_a, adev_a, label=f"accel {axis}")

    ax_g.set_xlabel("τ (s)"); ax_g.set_ylabel("Allan Deviation (dps)")
    ax_g.set_title("Gyro Allan Deviation"); ax_g.grid(True, which="both", alpha=0.3); ax_g.legend()
    ax_a.set_xlabel("τ (s)"); ax_a.set_ylabel("Allan Deviation (g)")
    ax_a.set_title("Accel Allan Deviation"); ax_a.grid(True, which="both", alpha=0.3); ax_a.legend()
    fig1.tight_layout()

    print("\n--- Gyro (Allan deviation) ---")
    for axis, params in zip(axes, gyro_params):
        print(f"  [{axis}] bias instability ~ {params['bias_instability']:.5f} dps "
              f"tai τ={params['tau_bias_instability_s']:.2f}s | "
              f"ARW-like(τ=1s) ~ {params['arw_like']:.5f} dps")

    print("\n--- Accel (Allan deviation) ---")
    for axis, params in zip(axes, accel_params):
        print(f"  [{axis}] bias instability ~ {params['bias_instability']*1000:.3f} mg "
              f"tai τ={params['tau_bias_instability_s']:.2f}s | "
              f"VRW-like(τ=1s) ~ {params['arw_like']*1000:.3f} mg")

    # ---- 2. Kiem tra vat ly accelerometer: |a| va gia toc ngoai ----
    accel_matrix = np.vstack(accel_cols).T
    accel_mag = np.linalg.norm(accel_matrix, axis=1)
    mag_mean = np.mean(accel_mag)
    mag_std = np.std(accel_mag)
    external_mask = np.abs(accel_mag - 1.0) > args.external_accel_threshold_g
    n_external = int(np.sum(external_mask))

    print(f"\n--- Kiem tra |a| (phai ~1g khi dung yen) ---")
    print(f"  |a| trung binh = {mag_mean:.5f} g, std = {mag_std * 1000:.3f} mg")
    print(f"  So mau vuot nguong ±{args.external_accel_threshold_g}g quanh 1g: "
          f"{n_external}/{n_static} ({100*n_external/n_static:.2f}%)")
    if n_external / n_static > 0.02:
        print("  -> Ty le kha cao du dang 'dung yen' - kiem tra rung co khi that su "
              "trong luc log (breadboard long, quat may tinh, v.v).")

    t_static = np.arange(n_static) * dt_nominal
    fig2, ax_mag = plt.subplots(figsize=(11, 4))
    ax_mag.plot(t_static, accel_mag, lw=0.8, label="|a| (g)")
    ax_mag.axhline(1.0, color="gray", ls="--", lw=0.8, label="1g (lý tưởng)")
    if n_external > 0:
        ax_mag.scatter(t_static[external_mask], accel_mag[external_mask],
                        color="red", s=8, label="nghi ngờ gia tốc ngoài", zorder=5)
    ax_mag.set_xlabel("Thời gian (s)"); ax_mag.set_ylabel("|a| (g)")
    ax_mag.set_title("Kiểm tra trọng lực tĩnh & gia tốc ngoài")
    ax_mag.legend(); ax_mag.grid(True, alpha=0.3)
    fig2.tight_layout()

    # ---- 2b. Do tre tuong doi (neu co file motion) ----
    relative_delay_ms = 0.0
    if args.motion:
        print(f"\nDoc file chuyen dong: {args.motion}")
        motion_data = load_log(args.motion)
        n_motion = len(motion_data)
        t_motion = np.arange(n_motion) * dt_nominal

        ax_g_motion = motion_data["gx_dps"]
        ay_motion = motion_data["ay_g"]
        az_motion = motion_data["az_g"]

        gyro_tilt = np.cumsum(ax_g_motion) * dt_nominal  # tich phan gyro X -> goc (deg), chi dung ngan han
        accel_tilt = np.degrees(np.arctan2(ay_motion, az_motion))
        accel_tilt = accel_tilt - np.mean(accel_tilt) + np.mean(gyro_tilt)  # can bang offset de so sanh dang song

        delay_s = relative_delay_via_xcorr(t_motion, accel_tilt, gyro_tilt, dt_nominal)
        relative_delay_ms = delay_s * 1000.0
        print(f"  Do tre TUONG DOI do duoc (accel tre hon gyro neu duong): {relative_delay_ms:.2f} ms")

        fig3, ax_delay = plt.subplots(figsize=(11, 4))
        ax_delay.plot(t_motion, gyro_tilt, label="Góc từ gyro (tích phân)")
        ax_delay.plot(t_motion, accel_tilt, label="Góc từ accel (atan2)")
        ax_delay.set_xlabel("Thời gian (s)"); ax_delay.set_ylabel("Góc (deg, đã căn offset)")
        ax_delay.set_title(f"Ước lượng độ trễ tương đối accel↔gyro: {relative_delay_ms:.1f} ms")
        ax_delay.legend(); ax_delay.grid(True, alpha=0.3)
        fig3.tight_layout()
    else:
        print("\n(Không có --motion, bỏ qua bước đo độ trễ tương đối bằng cross-correlation. "
              "Chỉ in delay xấp xỉ theo công thức nếu có --dlpf-*-hz.)")

    # ---- 2a. Delay xap xi tu cong thuc DLPF ----
    if args.dlpf_accel_hz:
        d = analytical_group_delay_s(args.dlpf_accel_hz) * 1000
        print(f"\nDelay xấp xỉ (công thức 1-pole) accel DLPF={args.dlpf_accel_hz}Hz: ~{d:.2f} ms "
              f"(CẬN DƯỚI tham khảo, DLPF thật của ICM là bậc cao hơn nên delay thực có thể lớn hơn)")
    if args.dlpf_gyro_hz:
        d = analytical_group_delay_s(args.dlpf_gyro_hz) * 1000
        print(f"Delay xấp xỉ (công thức 1-pole) gyro DLPF={args.dlpf_gyro_hz}Hz: ~{d:.2f} ms "
              f"(CẬN DƯỚI tham khảo)")

    # ---- 3. Ket luan tong hop ----
    avg_gyro_bias_instab = np.mean([pp["bias_instability"] for pp in gyro_params])
    avg_accel_bias_instab = np.mean([pp["bias_instability"] for pp in accel_params])

    print("\n" + "=" * 78)
    print("ĐÁNH GIÁ TỔNG HỢP (tham khảo, không phải phán quyết tuyệt đối):")
    print(fusion_readiness_verdict(avg_gyro_bias_instab, avg_accel_bias_instab, relative_delay_ms))
    print("=" * 78)

    plt.show()


if __name__ == "__main__":
    main()
