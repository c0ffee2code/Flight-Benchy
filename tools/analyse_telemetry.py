"""
Detailed stats for a Flight Benchy telemetry run.

Prints diagnostic metrics to console. Run after kpi.py confirms the run
passed the basic KPI gate (reached horizontal).

Usage:
    python tools/analyse_telemetry.py test_runs/2026-04-07_10-00-00
    python tools/analyse_telemetry.py test_runs/run_a test_runs/run_b   # comparison
"""

import csv
import sys
from pathlib import Path

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def resolve_run(path_str):
    p = Path(path_str)
    csv_path = (p / "log.csv") if p.is_dir() else p
    cfg_path = (p / "config.yaml") if p.is_dir() else (p.parent / "config.yaml")
    label = p.name if p.is_dir() else p.stem
    if not csv_path.exists():
        sys.exit(f"CSV not found: {csv_path}")
    config = _load_config(cfg_path) if cfg_path.exists() else None
    return csv_path, config, label


def _load_config(path):
    if yaml is None:
        return None
    with open(path) as f:
        return yaml.safe_load(f)


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        sys.exit(f"Empty CSV: {path}")
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def quat_to_roll(qr, qi):
    return np.degrees(2.0 * np.arctan2(qi, qr))


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def compute_stats(cols, config):
    n = len(cols["T_MS"])
    if n < 2:
        return None

    t_ms = cols["T_MS"]
    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0
    actual_hz = (n - 1) / duration_s if duration_s > 0 else 0
    dts = np.diff(t_ms)
    dt_mean = np.mean(dts)
    dt_median = np.median(dts)

    errors = imu_roll - enc_roll
    abs_errors = np.abs(errors)
    mae = np.mean(abs_errors)
    rms_error = np.sqrt(np.mean(errors ** 2))
    max_ae = np.max(abs_errors)
    bias = np.mean(errors)

    mae_enc_setpoint = np.mean(np.abs(enc_roll))
    mae_imu_setpoint = np.mean(np.abs(imu_roll))
    bias_enc_setpoint = np.mean(enc_roll)
    max_ae_enc_setpoint = np.max(np.abs(enc_roll))

    if np.std(enc_roll) > 0 and np.std(imu_roll) > 0:
        correlation = np.corrcoef(enc_roll, imu_roll)[0, 1]
    else:
        correlation = 0.0

    enc_vel = np.abs(np.diff(enc_roll)) / (dts / 1000.0)
    vel_order = np.argsort(enc_vel)
    quarter = len(vel_order) // 4
    slow_idx = vel_order[:quarter] + 1
    fast_idx = vel_order[-quarter:] + 1
    mae_slow = np.mean(abs_errors[slow_idx]) if len(slow_idx) > 0 else 0
    mae_fast = np.mean(abs_errors[fast_idx]) if len(fast_idx) > 0 else 0

    motion_dir = np.diff(enc_roll)
    imu_offset = enc_roll[1:] - imu_roll[1:]
    moving = np.abs(motion_dir) > 1.0
    if np.sum(moving) > 0:
        trailing = (motion_dir[moving] * imu_offset[moving]) > 0
        trail_pct = np.sum(trailing) / np.sum(moving) * 100
    else:
        trail_pct = 0.0

    enc_range = np.ptp(enc_roll)
    imu_range = np.ptp(imu_roll)

    sign_changes = np.diff(np.sign(errors))
    zero_crossings = np.sum(sign_changes != 0)
    osc_freq = zero_crossings / (2.0 * duration_s) if duration_s > 0 else 0

    ang_iterm_limit = 100.0
    if config and "angle_pid" in config:
        ang_iterm_limit = config["angle_pid"].get("iterm_limit", 100.0)
    ang_windup_threshold = ang_iterm_limit * 0.5
    ang_windup_events = int(np.sum(np.abs(cols["ANG_I"]) >= ang_windup_threshold))

    rate_iterm_limit = 50.0
    if config and "rate_pid" in config:
        rate_iterm_limit = config["rate_pid"].get("iterm_limit", 50.0)
    rate_windup_threshold = rate_iterm_limit * 0.5
    rate_windup_events = int(np.sum(np.abs(cols["RATE_I"]) >= rate_windup_threshold))

    return {
        "n_samples": n, "duration_s": duration_s, "actual_hz": actual_hz,
        "dt_mean_ms": dt_mean, "dt_median_ms": dt_median,
        "mae": mae, "rms_error": rms_error, "max_ae": max_ae, "bias": bias,
        "mae_enc_setpoint": mae_enc_setpoint, "mae_imu_setpoint": mae_imu_setpoint,
        "bias_enc_setpoint": bias_enc_setpoint, "max_ae_enc_setpoint": max_ae_enc_setpoint,
        "correlation": correlation, "mae_fast": mae_fast, "mae_slow": mae_slow,
        "trail_pct": trail_pct, "enc_range": enc_range, "imu_range": imu_range,
        "osc_freq_hz": osc_freq,
        "ang_windup_events": ang_windup_events, "ang_windup_threshold": ang_windup_threshold,
        "rate_windup_events": rate_windup_events, "rate_windup_threshold": rate_windup_threshold,
    }


# ---------------------------------------------------------------------------
# Printing
# ---------------------------------------------------------------------------

def print_config_summary(config, label):
    if config is None:
        print(f"  (no config.yaml for {label})")
        return
    angle = config.get("angle_pid", {})
    rate  = config.get("rate_pid", {})
    motor = config.get("motor", {})
    imu   = config.get("imu", {})
    print(f"  Angle PID: kp={angle.get('kp')}, ki={angle.get('ki')}, "
          f"kd={angle.get('kd')}, iterm_limit={angle.get('iterm_limit')}, hz={angle.get('hz')}")
    print(f"  Rate PID:  kp={rate.get('kp')}, ki={rate.get('ki')}, "
          f"kd={rate.get('kd')}, iterm_limit={rate.get('iterm_limit')}, hz={rate.get('hz')}")
    if "report_hz" in imu:
        print(f"  IMU: {imu.get('report')} @ {imu.get('report_hz')} Hz")
    else:
        print(f"  IMU: angle={imu.get('angle_report')} @ {imu.get('angle_report_hz')} Hz, "
              f"rate={imu.get('rate_report')} @ {imu.get('rate_report_hz')} Hz")
    print(f"  Motor: base={motor.get('base_throttle')}, "
          f"min={motor.get('throttle_min')}, max={motor.get('throttle_max')}")


def _stats_block(stats):
    def row(name, key, fmt=".2f"):
        print(f"  {name:<35} {stats[key]:>12{fmt}}")

    print("  --- Sample Rate ---")
    row("Samples",        "n_samples",   ".0f")
    row("Duration (s)",   "duration_s",  ".1f")
    row("Achieved Hz",    "actual_hz",   ".1f")
    row("Mean dt (ms)",   "dt_mean_ms",  ".1f")
    row("Median dt (ms)", "dt_median_ms",".1f")

    print("\n  --- Sensor Tracking Error (IMU vs ENC) ---")
    row("MAE (overall)",    "mae")
    row("MAE (fast motion)","mae_fast")
    row("MAE (slow motion)","mae_slow")
    row("Max AE",           "max_ae")
    row("RMS Error",        "rms_error")
    row("Bias (IMU-ENC)",   "bias")

    print("\n  --- Setpoint Error (vs 0° horizontal) ---")
    row("ENC MAE (deg)",     "mae_enc_setpoint")
    row("IMU MAE (deg)",     "mae_imu_setpoint")
    row("ENC Bias (deg)",    "bias_enc_setpoint")
    row("ENC Max AE (deg)",  "max_ae_enc_setpoint")

    print("\n  --- Correlation & Tracking ---")
    row("Pearson r",           "correlation", ".4f")
    row("IMU trails motion (%)", "trail_pct", ".1f")
    row("Encoder range (deg)", "enc_range",   ".1f")
    row("IMU range (deg)",     "imu_range",   ".1f")

    print("\n  --- Oscillation & Windup ---")
    row("Oscillation freq (Hz)",  "osc_freq_hz",      ".2f")
    row("Angle windup events",    "ang_windup_events", ".0f")
    print(f"  {'Angle windup threshold':<35} {stats['ang_windup_threshold']:>12.1f}")
    row("Rate windup events",     "rate_windup_events",".0f")
    print(f"  {'Rate windup threshold':<35} {stats['rate_windup_threshold']:>12.1f}")


def print_single(label, stats, config):
    w = 55
    print("=" * w)
    print(f"  {label}")
    print("-" * w)
    print_config_summary(config, label)
    print()
    _stats_block(stats)
    print("=" * w)


def print_comparison(label_a, stats_a, config_a, label_b, stats_b, config_b):
    w = 65
    print("=" * w)
    print(f"  {'Metric':<35} {label_a:>12}   {label_b:>12}")
    print("-" * w)

    print("\n  --- Config ---")
    print("  Run A:"); print_config_summary(config_a, label_a)
    print("  Run B:"); print_config_summary(config_b, label_b)

    def row(name, key, fmt=".2f"):
        va, vb = stats_a[key], stats_b[key]
        print(f"  {name:<35} {va:>12{fmt}}   {vb:>12{fmt}}")

    print("\n  --- Sample Rate ---")
    row("Samples",        "n_samples",    ".0f")
    row("Duration (s)",   "duration_s",   ".1f")
    row("Achieved Hz",    "actual_hz",    ".1f")

    print("\n  --- Sensor Tracking Error (IMU vs ENC) ---")
    row("MAE (overall)",    "mae")
    row("MAE (fast motion)","mae_fast")
    row("MAE (slow motion)","mae_slow")
    row("Max AE",           "max_ae")
    row("RMS Error",        "rms_error")
    row("Bias (IMU-ENC)",   "bias")

    print("\n  --- Setpoint Error (vs 0° horizontal) ---")
    row("ENC MAE (deg)",    "mae_enc_setpoint")
    row("IMU MAE (deg)",    "mae_imu_setpoint")
    row("ENC Bias (deg)",   "bias_enc_setpoint")
    row("ENC Max AE (deg)", "max_ae_enc_setpoint")

    print("\n  --- Correlation & Tracking ---")
    row("Pearson r",            "correlation", ".4f")
    row("IMU trails motion (%)", "trail_pct",  ".1f")
    row("Encoder range (deg)",  "enc_range",   ".1f")
    row("IMU range (deg)",      "imu_range",   ".1f")

    print("\n  --- Oscillation & Windup ---")
    row("Oscillation freq (Hz)",  "osc_freq_hz",       ".2f")
    row("Angle windup events",    "ang_windup_events",  ".0f")
    row("Rate windup events",     "rate_windup_events", ".0f")
    print("=" * w)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python tools/analyse_telemetry.py <run_folder> [run2]")
        sys.exit(1)

    runs = []
    for arg in sys.argv[1:]:
        csv_path, config, label = resolve_run(arg)
        cols = load_csv(csv_path)
        stats = compute_stats(cols, config)
        runs.append((config, label, stats))
        print(f"Loaded {label}: {stats['n_samples']} samples, {stats['duration_s']:.1f}s")

    print()
    if len(runs) == 1:
        config, label, stats = runs[0]
        print_single(label, stats, config)
    elif len(runs) == 2:
        _, la, sa = runs[0]
        _, lb, sb = runs[1]
        print_comparison(la, sa, runs[0][0], lb, sb, runs[1][0])
    else:
        for config, label, stats in runs:
            print_single(label, stats, config)


if __name__ == "__main__":
    main()