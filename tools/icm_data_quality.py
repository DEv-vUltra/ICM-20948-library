#!/usr/bin/env python3
"""
ICM-20948 data-quality analyzer: is the accel/gyro data clean enough to feed a
sensor-fusion filter (complementary / Mahony / EKF / ESKF)?

It looks at three things:

  1. ALLAN DEVIATION (IEEE Std 952 style) of every gyro and accel axis. It
     separates white noise (ARW / VRW) from bias instability and from rate
     random walk. Default estimator: non-overlapping clusters,
         sigma^2(tau) = 1/2 <(ybar[k+1] - ybar[k])^2>,  tau = m * dt
     (--overlapping selects the overlapping estimator, which is smoother).

  2. RELATIVE DELAY between the accel-derived and the gyro-integrated tilt
     angle, from a motion log, by cross-correlation (with bias removal,
     detrending and parabolic peak interpolation). Positive = accel lags gyro.
     The analytic one-pole DLPF delay 1/(2*pi*fc) is printed as a lower bound.

  3. GRAVITY CHECK: while still, |a| must stay constant and close to 1 g.
     Deviations from the median |a| are flagged as external acceleration.

How the log is prepared (this matters for correct tau and delay values):
  * Consecutive identical rows are collapsed into ONE sensor sample. The
    firmware polls faster than the sensor output rate, so each sample is
    typically read several times; treating those rows as independent samples
    would scale the tau axis and the delay by the repeat factor.
    Use --no-dedupe if your log already has exactly one row per sample.
  * The sample period is MEASURED from the firmware timestamps over the longest
    gap-free segment (--dt-source nominal uses --odr-hz instead). The ICM-20948
    runs from an on-chip oscillator, so its configured ODR is only nominal.
  * The log is split at gaps (lost lines, firmware restarts); Allan deviation
    uses the longest contiguous segment.

Typical use:
    python3 tools/icm_logger.py /dev/ttyUSB0 460800 --warmup 60 --duration 600 \
        --out static.csv --odr-hz 51 --dlpf-accel-hz 6 --dlpf-gyro-hz 6
    python3 tools/icm_logger.py /dev/ttyUSB0 460800 --duration 20 --out motion.csv
    python3 tools/icm_data_quality.py --static static.csv --motion motion.csv
"""

import argparse
import json
import os
import sys

import numpy as np
import matplotlib

if "--no-show" in sys.argv:
    matplotlib.use("Agg")           # must be chosen before pyplot is imported
import matplotlib.pyplot as plt

G_MS2 = 9.80665
IEEE952_FLICKER_FACTOR = 0.664      # ADEV minimum = 0.664 * bias instability (IEEE 952)
VALUE_COLUMNS = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")
REQUIRED_COLUMNS = ("timestamp_ms", "seq") + VALUE_COLUMNS
AXES = ("x", "y", "z")
COLORS = ("tab:red", "tab:green", "tab:blue")


# ---------------------------------------------------------------------------
# Loading and preparing logs
# ---------------------------------------------------------------------------
def load_log(path):
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    names = data.dtype.names or ()
    missing = [c for c in REQUIRED_COLUMNS if c not in names]
    if missing:
        sys.exit(f"{path}: missing columns {missing} (expected the output of icm_logger.py)")
    return data


def load_meta(csv_path):
    meta_path = os.path.splitext(csv_path)[0] + ".meta.json"
    if os.path.exists(meta_path):
        try:
            with open(meta_path) as f:
                return json.load(f)
        except (OSError, ValueError):
            print(f"Warning: could not read {meta_path}, ignoring it.")
    return {}


def collapse_duplicate_rows(data, dedupe=True):
    """Return (t_ms, values[N,6]) with consecutive identical rows collapsed to one sample.
    The timestamp of the first row of each run is kept."""
    values = np.column_stack([data[c] for c in VALUE_COLUMNS])
    t_ms = np.asarray(data["timestamp_ms"], dtype=float)
    if not dedupe or len(values) < 2:
        return t_ms, values
    changed = np.any(values[1:] != values[:-1], axis=1)
    keep = np.concatenate(([True], changed))
    return t_ms[keep], values[keep]


def contiguous_segments(t_ms, period_ms, gap_factor=1.5):
    """Split sample indices where the time step is negative (restart) or larger than
    gap_factor * period (lost samples). Returns a list of (start, stop) index pairs."""
    if len(t_ms) < 2:
        return [(0, len(t_ms))]
    d = np.diff(t_ms)
    breaks = np.where((d < 0) | (d > gap_factor * period_ms))[0] + 1
    edges = np.concatenate(([0], breaks, [len(t_ms)]))
    return [(int(edges[i]), int(edges[i + 1])) for i in range(len(edges) - 1)]


def prepare_log(path, label, args, nominal_hz):
    """Load a CSV and return a dict with the unique samples, the longest contiguous
    segment and the sample period (seconds) to use as time base."""
    data = load_log(path)
    n_rows = len(data)
    seq = np.asarray(data["seq"], dtype=float)
    seq_lost = int(np.sum(np.maximum(np.diff(seq) - 1, 0))) if n_rows > 1 else 0

    t_ms, values = collapse_duplicate_rows(data, dedupe=not args.no_dedupe)
    n_unique = len(t_ms)
    if n_unique < 16:
        sys.exit(f"{path}: only {n_unique} samples after preparation; need at least 16.")

    median_period_ms = float(np.median(np.diff(t_ms)))
    if median_period_ms <= 0:
        sys.exit(f"{path}: timestamps do not advance; cannot derive a sample period.")
    segments = contiguous_segments(t_ms, median_period_ms)
    s0, s1 = max(segments, key=lambda s: s[1] - s[0])
    n_seg = s1 - s0
    measured_ms = (t_ms[s1 - 1] - t_ms[s0]) / (n_seg - 1) if n_seg > 1 else median_period_ms
    nominal_ms = 1000.0 / nominal_hz if nominal_hz else None

    if args.dt_source == "nominal":
        if nominal_ms is None:
            sys.exit("--dt-source nominal needs --odr-hz (or an odr_hz entry in the metadata file).")
        dt_ms = nominal_ms
    else:
        dt_ms = measured_ms

    print("-" * 78)
    print(f"{label}: {path}")
    print(f"  rows: {n_rows} | unique samples: {n_unique} "
          f"(each sample read ~{n_rows / n_unique:.1f}x) | lost by seq: {seq_lost}")
    print(f"  segments: {len(segments)} | longest: {n_seg} samples "
          f"({100.0 * n_seg / n_unique:.1f}% of the log)")
    print(f"  sample period: measured {measured_ms:.3f} ms ({1000.0 / measured_ms:.2f} Hz)", end="")
    if nominal_ms is not None:
        dev = 100.0 * (measured_ms - nominal_ms) / nominal_ms
        print(f" | nominal {nominal_ms:.3f} ms ({nominal_hz:g} Hz) | deviation {dev:+.1f}%")
        if abs(dev) > 3.0:
            print("  WARNING: measured period differs from the nominal ODR by more than 3%. "
                  "Check --odr-hz, the configured sample-rate divider, and whether the log has "
                  "exactly one row per sensor sample.")
    else:
        print()
    print(f"  time base used for analysis: {dt_ms:.3f} ms ({args.dt_source})")
    if n_seg / n_unique < 0.8:
        print("  WARNING: the longest gap-free segment covers less than 80% of the log; "
              "the log has many gaps or restarts.")

    return {
        "t_ms": t_ms, "values": values, "segment": (s0, s1),
        "seg_values": values[s0:s1], "dt_s": dt_ms / 1000.0,
        "n_rows": n_rows, "n_unique": n_unique,
    }


# ---------------------------------------------------------------------------
# 1. Allan deviation
# ---------------------------------------------------------------------------
def allan_deviation(x, dt, n_points=60, overlapping=False):
    """x: evenly sampled data (e.g. gyro X in dps), dt: sample period [s].
    Returns (tau, adev, rel_err); rel_err is the approximate relative 1-sigma error
    1/sqrt(2*(clusters-1)) (conservative for the overlapping estimator)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 16:
        raise ValueError("At least 16 samples are needed for an Allan deviation.")
    x = x - np.mean(x)                  # ADEV ignores constant offsets; this limits round-off
    max_m = n // 4                      # keep at least ~4 clusters at the largest tau
    m_list = np.unique(np.logspace(0, np.log10(max_m), n_points).astype(int))
    csum = np.concatenate(([0.0], np.cumsum(x))) if overlapping else None

    taus, adevs, rel_err = [], [], []
    for m in m_list:
        clusters = n // m
        if m < 1 or clusters < 3:
            continue
        if overlapping:
            means = (csum[m:] - csum[:-m]) / m          # every window of m samples
            diffs = means[m:] - means[:-m]
        else:
            means = x[: clusters * m].reshape(clusters, m).mean(axis=1)
            diffs = np.diff(means)
        if diffs.size == 0:
            continue
        taus.append(m * dt)
        adevs.append(np.sqrt(0.5 * np.mean(diffs ** 2)))
        rel_err.append(1.0 / np.sqrt(2.0 * (clusters - 1)))
    return np.array(taus), np.array(adevs), np.array(rel_err)


def extract_noise_params(taus, adevs):
    """Read the noise figures off an Allan curve.

    bias_instability : ADEV minimum / 0.664 (IEEE 952). Only valid if the minimum is
                       really observed (bias_observed); otherwise it is just a lower
                       value of an unfinished curve and the log is too short.
    arw_n            : ADEV at tau = 1 s, which equals the white-noise coefficient N
                       (units/sqrt(Hz)) when the curve has slope -1/2 there.
    slope_at_1s      : log-log slope for 0.5 s <= tau <= 2 s (None if too few points);
                       about -0.5 means the white-noise reading is valid.
    """
    n = len(taus)
    idx_min = int(np.argmin(adevs))
    sigma_min = float(adevs[idx_min])
    observed = (idx_min <= n - 4) and (adevs[-1] > 1.1 * sigma_min)

    idx_1s = int(np.argmin(np.abs(np.log(taus))))
    window = (taus >= 0.5) & (taus <= 2.0)
    slope = None
    if np.count_nonzero(window) >= 3:
        slope = float(np.polyfit(np.log10(taus[window]), np.log10(adevs[window]), 1)[0])

    return {
        "sigma_min": sigma_min,
        "tau_min_s": float(taus[idx_min]),
        "bias_instability": sigma_min / IEEE952_FLICKER_FACTOR,
        "bias_observed": bool(observed),
        "arw_n": float(adevs[idx_1s]),
        "tau_arw_used_s": float(taus[idx_1s]),
        "slope_at_1s": slope,
    }


# ---------------------------------------------------------------------------
# 2. Relative delay accel <-> gyro
# ---------------------------------------------------------------------------
def integrate_trapezoid(rate, dt):
    return np.concatenate(([0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1])) * dt))


def detrend_linear(y):
    k = np.arange(len(y))
    return y - np.polyval(np.polyfit(k, y, 1), k)       # removes mean and a linear drift


def relative_delay_via_xcorr(accel_tilt_deg, gyro_tilt_deg, dt, max_lag_s=0.5):
    """Return (delay_s, peak_correlation, at_window_edge). Positive delay = accel lags gyro.
    Both signals are detrended first (a leftover gyro bias would otherwise appear as a
    ramp and pull the peak towards zero lag), and the peak is refined by a parabola
    through its two neighbours, so the result is not limited to whole samples."""
    a = detrend_linear(np.asarray(accel_tilt_deg, dtype=float))
    g = detrend_linear(np.asarray(gyro_tilt_deg, dtype=float))
    na, ng = np.linalg.norm(a), np.linalg.norm(g)
    if na == 0.0 or ng == 0.0:
        raise ValueError("A tilt signal is constant; the motion log contains no motion.")

    corr = np.correlate(a, g, mode="full") / (na * ng)
    lags = np.arange(-len(g) + 1, len(a))
    max_lag = int(max_lag_s / dt)
    candidates = np.where(np.abs(lags) <= max_lag)[0]
    best = int(candidates[np.argmax(corr[candidates])])

    lag = float(lags[best])
    if 0 < best < len(corr) - 1:
        y0, y1, y2 = corr[best - 1], corr[best], corr[best + 1]
        denom = y0 - 2.0 * y1 + y2
        if denom != 0.0:
            lag += 0.5 * (y0 - y2) / denom
    at_edge = abs(lags[best]) >= max_lag
    return lag * dt, float(corr[best]), bool(at_edge)


def analytical_group_delay_s(fc_hz):
    """One-pole approximation 1/(2*pi*fc). Only a lower bound: the real ICM DLPF is
    higher order, see the group-delay table in the datasheet."""
    return 1.0 / (2.0 * np.pi * fc_hz)


# ---------------------------------------------------------------------------
# Verdict
# ---------------------------------------------------------------------------
def fusion_readiness_verdict(gyro_params, accel_params, delay_ms, delay_ok):
    lines = []

    g_obs = [p["bias_instability"] for p in gyro_params if p["bias_observed"]]
    if g_obs:
        b = float(np.mean(g_obs))
        dph = b * 3600.0
        lines.append(f"  Gyro bias instability (IEEE 952) ~ {b:.5f} dps (~{dph:.1f} deg/hr), "
                     f"minimum observed on {len(g_obs)}/3 axes")
        if dph < 10:
            lines.append("    -> very good for a MEMS part")
        elif dph < 50:
            lines.append("    -> typical industrial grade; fine for most hobby/UAV use with bias estimation")
        else:
            lines.append("    -> consumer grade: estimate the bias continuously (EKF/ESKF), "
                         "do not rely on a fixed calibration")
    else:
        lines.append("  Gyro bias instability: the Allan minimum is NOT observed on any axis "
                     "-> record a longer static log to grade it")

    a_obs = [p["bias_instability"] for p in accel_params if p["bias_observed"]]
    if a_obs:
        lines.append(f"  Accel bias instability (IEEE 952) ~ {np.mean(a_obs) * 1000:.3f} mg, "
                     f"minimum observed on {len(a_obs)}/3 axes")
    else:
        lines.append("  Accel bias instability: minimum NOT observed -> record a longer static log")

    arw = float(np.mean([p["arw_n"] for p in gyro_params])) * 60.0       # dps/sqrt(Hz) -> deg/sqrt(hr)
    vrw = float(np.mean([p["arw_n"] for p in accel_params])) * G_MS2 * 60.0   # g/sqrt(Hz) -> (m/s)/sqrt(hr)
    lines.append(f"  Gyro ARW ~ {arw:.3f} deg/sqrt(hr) | Accel VRW ~ {vrw:.3f} (m/s)/sqrt(hr) "
                 f"(read at tau = 1 s)")

    if not delay_ok:
        lines.append("  Relative accel/gyro delay: not measured (give --motion)")
    else:
        lines.append(f"  Relative accel/gyro delay ~ {delay_ms:.1f} ms")
        if abs(delay_ms) < 5:
            lines.append("    -> very good, the channels are almost in phase")
        elif abs(delay_ms) < 20:
            lines.append("    -> acceptable for complementary/Mahony filters; an EKF/ESKF should "
                         "model it or align by timestamp")
        else:
            lines.append("    -> large: raise the DLPF bandwidth or compensate with buffering/"
                         "interpolation before the fusion filter")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
def build_arg_parser():
    p = argparse.ArgumentParser(
        description="Assess ICM-20948 accel/gyro data quality for sensor fusion")
    p.add_argument("--static", required=True, help="CSV recorded with the IMU completely still (icm_logger.py)")
    p.add_argument("--motion", default=None,
                   help="CSV recorded during slow, smooth tilting about X (enables the delay estimate)")
    p.add_argument("--odr-hz", type=float, default=None,
                   help="Nominal sensor output rate [Hz]; cross-check only unless --dt-source nominal. "
                        "Defaults to the odr_hz entry of the static log's metadata file.")
    p.add_argument("--dt-source", choices=("measured", "nominal"), default="measured",
                   help="Time base: measured from firmware timestamps (default) or 1/--odr-hz")
    p.add_argument("--dlpf-accel-hz", type=float, default=None, help="Accel DLPF cut-off [Hz] (default: metadata)")
    p.add_argument("--dlpf-gyro-hz", type=float, default=None, help="Gyro DLPF cut-off [Hz] (default: metadata)")
    p.add_argument("--external-accel-threshold-g", type=float, default=0.05,
                   help="Deviation of |a| from its median flagged as external acceleration (default 0.05 g)")
    p.add_argument("--overlapping", action="store_true",
                   help="Use the overlapping Allan estimator instead of non-overlapping clusters")
    p.add_argument("--no-dedupe", action="store_true",
                   help="Do not collapse repeated rows (use when the log has one row per sensor sample)")
    p.add_argument("--save-prefix", default=None, help="Save the figures as <prefix>_allan.png, _gravity.png, _delay.png")
    p.add_argument("--no-show", action="store_true", help="Do not open plot windows (headless use)")
    return p


def main():
    args = build_arg_parser().parse_args()

    meta = load_meta(args.static)
    nominal_hz = args.odr_hz or meta.get("odr_hz")
    dlpf_accel_hz = args.dlpf_accel_hz or meta.get("dlpf_accel_hz")
    dlpf_gyro_hz = args.dlpf_gyro_hz or meta.get("dlpf_gyro_hz")

    print("=" * 78)
    static = prepare_log(args.static, "Static log", args, nominal_hz)
    seg = static["seg_values"]
    dt = static["dt_s"]
    if len(seg) * dt < 60:
        print("WARNING: the usable static segment is shorter than 60 s; the Allan curve will be "
              "short and noisy. Record minutes for the white-noise region, tens of minutes for "
              "the bias-instability minimum.")

    accel = seg[:, 0:3]
    gyro = seg[:, 3:6]
    estimator = "overlapping" if args.overlapping else "non-overlapping"

    # ---- 1. Allan deviation ----
    fig1, (ax_g, ax_a) = plt.subplots(1, 2, figsize=(13, 5))
    gyro_params, accel_params = [], []
    for i, axis in enumerate(AXES):
        tau, adev, rel = allan_deviation(gyro[:, i], dt, overlapping=args.overlapping)
        gyro_params.append(extract_noise_params(tau, adev))
        ax_g.loglog(tau, adev, color=COLORS[i], label=f"gyro {axis}")
        ax_g.fill_between(tau, adev * (1 - rel), adev * (1 + rel), color=COLORS[i], alpha=0.12)

        tau, adev, rel = allan_deviation(accel[:, i], dt, overlapping=args.overlapping)
        accel_params.append(extract_noise_params(tau, adev))
        ax_a.loglog(tau, adev, color=COLORS[i], label=f"accel {axis}")
        ax_a.fill_between(tau, adev * (1 - rel), adev * (1 + rel), color=COLORS[i], alpha=0.12)

    ax_g.set_xlabel("tau (s)"); ax_g.set_ylabel("Allan deviation (dps)")
    ax_g.set_title(f"Gyro Allan deviation ({estimator}, shaded = ~1 sigma)")
    ax_a.set_xlabel("tau (s)"); ax_a.set_ylabel("Allan deviation (g)")
    ax_a.set_title(f"Accel Allan deviation ({estimator}, shaded = ~1 sigma)")
    for ax in (ax_g, ax_a):
        ax.grid(True, which="both", alpha=0.3)
        ax.legend()
    fig1.tight_layout()

    def describe(axis, p, scale, unit):
        text = f"  [{axis}] "
        if p["bias_observed"]:
            text += f"bias instability ~ {p['bias_instability'] * scale:.4g} {unit} (min at tau={p['tau_min_s']:.1f}s)"
        else:
            text += (f"ADEV minimum {p['sigma_min'] * scale:.4g} {unit} at tau={p['tau_min_s']:.1f}s is NOT a "
                     f"real minimum (curve still falling) -> bias instability not observed")
        text += f" | ADEV(1s) ~ {p['arw_n'] * scale:.4g} {unit}"
        if p["slope_at_1s"] is None:
            text += " (slope near 1 s unavailable: log too short)"
        elif not -0.7 <= p["slope_at_1s"] <= -0.3:
            text += f" (slope {p['slope_at_1s']:+.2f}: not in the white-noise region, treat as an upper bound)"
        return text

    print("\n--- Gyro (Allan deviation) ---")
    for axis, p in zip(AXES, gyro_params):
        print(describe(axis, p, 1.0, "dps"))
    print("\n--- Accel (Allan deviation) ---")
    for axis, p in zip(AXES, accel_params):
        print(describe(axis, p, 1000.0, "mg"))

    # ---- 2. Gravity check on all unique static samples ----
    all_acc = static["values"][:, 0:3]
    t_s = (static["t_ms"] - static["t_ms"][0]) / 1000.0
    mag = np.linalg.norm(all_acc, axis=1)
    mag_median = float(np.median(mag))
    external = np.abs(mag - mag_median) > args.external_accel_threshold_g
    n_ext = int(np.sum(external))

    print("\n--- Gravity check (|a| should be constant and ~1 g when still) ---")
    print(f"  |a| median = {mag_median:.5f} g (offset from 1 g: {(mag_median - 1.0) * 1000:+.1f} mg), "
          f"std = {np.std(mag) * 1000:.3f} mg")
    print(f"  Samples more than {args.external_accel_threshold_g} g from the median: "
          f"{n_ext}/{len(mag)} ({100.0 * n_ext / len(mag):.2f}%)")
    if abs(mag_median - 1.0) > 0.02:
        print("  -> |a| is more than 20 mg away from 1 g: the accelerometer needs scale/bias "
              "calibration before it can be trusted as a gravity reference.")
    if n_ext / len(mag) > 0.02:
        print("  -> many samples deviate: check for vibration while logging "
              "(loose breadboard, fan, table).")

    fig2, ax_mag = plt.subplots(figsize=(11, 4))
    ax_mag.plot(t_s, mag, lw=0.8, label="|a| (g)")
    ax_mag.axhline(1.0, color="gray", ls="--", lw=0.8, label="1 g")
    ax_mag.axhline(mag_median, color="tab:orange", ls=":", lw=0.8, label="median |a|")
    if n_ext > 0:
        ax_mag.scatter(t_s[external], mag[external], color="red", s=8, zorder=5,
                       label="suspected external acceleration")
    ax_mag.set_xlabel("Time (s)"); ax_mag.set_ylabel("|a| (g)")
    ax_mag.set_title("Static gravity check")
    ax_mag.legend(); ax_mag.grid(True, alpha=0.3)
    fig2.tight_layout()

    # ---- 3. Relative delay (needs a motion log) ----
    delay_ms, delay_ok = 0.0, False
    fig3 = None
    if args.motion:
        motion = prepare_log(args.motion, "Motion log", args, nominal_hz)
        mv = motion["seg_values"]
        mdt = motion["dt_s"]
        gx_bias = float(np.mean(seg[:, 3]))          # static gyro-X bias
        gyro_tilt = integrate_trapezoid(mv[:, 3] - gx_bias, mdt)
        accel_tilt = np.degrees(np.arctan2(mv[:, 1], mv[:, 2]))

        if np.ptp(accel_tilt) < 5.0:
            print("\nWARNING: tilt amplitude is below 5 deg; the delay estimate will be unreliable. "
                  "Tilt the board about X by 10-30 deg at about 1 Hz.")
        try:
            delay_s, peak, at_edge = relative_delay_via_xcorr(accel_tilt, gyro_tilt, mdt)
            delay_ms, delay_ok = delay_s * 1000.0, True
            print(f"\n--- Relative delay (positive = accel lags gyro) ---")
            print(f"  delay ~ {delay_ms:.2f} ms | correlation peak {peak:.2f} | "
                  f"sample period {mdt * 1000:.1f} ms")
            if peak < 0.7:
                print("  WARNING: low correlation; use slower, smoother, larger tilting and a still start/end.")
            if at_edge:
                print("  WARNING: the peak sits at the edge of the +/-0.5 s search window.")
        except ValueError as e:
            print(f"\nDelay estimate skipped: {e}")

        if delay_ok:
            tt = np.arange(len(mv)) * mdt
            gyro_plot = detrend_linear(gyro_tilt)
            accel_plot = detrend_linear(accel_tilt)
            fig3, ax_delay = plt.subplots(figsize=(11, 4))
            ax_delay.plot(tt, gyro_plot, label="angle from gyro (integrated, detrended)")
            ax_delay.plot(tt, accel_plot, label="angle from accel (atan2, detrended)")
            ax_delay.set_xlabel("Time (s)"); ax_delay.set_ylabel("Angle (deg)")
            ax_delay.set_title(f"Accel vs gyro tilt, estimated delay {delay_ms:.1f} ms")
            ax_delay.legend(); ax_delay.grid(True, alpha=0.3)
            fig3.tight_layout()
    else:
        print("\n(No --motion file: the cross-correlation delay estimate is skipped.)")

    if dlpf_accel_hz:
        print(f"\nAnalytic delay, accel DLPF {dlpf_accel_hz:g} Hz (one-pole lower bound): "
              f"~{analytical_group_delay_s(dlpf_accel_hz) * 1000:.2f} ms")
    if dlpf_gyro_hz:
        print(f"Analytic delay, gyro  DLPF {dlpf_gyro_hz:g} Hz (one-pole lower bound): "
              f"~{analytical_group_delay_s(dlpf_gyro_hz) * 1000:.2f} ms")

    # ---- Summary ----
    print("\n" + "=" * 78)
    print("SUMMARY (a reference, not a pass/fail verdict):")
    print(fusion_readiness_verdict(gyro_params, accel_params, delay_ms, delay_ok))
    print("=" * 78)

    if args.save_prefix:
        fig1.savefig(f"{args.save_prefix}_allan.png", dpi=150)
        fig2.savefig(f"{args.save_prefix}_gravity.png", dpi=150)
        if fig3 is not None:
            fig3.savefig(f"{args.save_prefix}_delay.png", dpi=150)
        print(f"Figures saved with prefix '{args.save_prefix}'.")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()