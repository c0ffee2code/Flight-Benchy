"""
Analyse Flight Benchy telemetry runs (cascaded PID schema).

Reads a run folder (containing log.csv and optional config.yaml) or a
flat CSV file. Computes diagnostic statistics and produces matplotlib plots.

Usage:
    python tools/analyse_telemetry.py test_runs/2026-02-17_21-15-21/
    python tools/analyse_telemetry.py test_runs/log.csv
    python tools/analyse_telemetry.py run_a/ run_b/          # side-by-side comparison

CSV schema (22 columns):
    T_MS,ENC_QR,ENC_QI,ENC_QJ,ENC_QK,IMU_QR,IMU_QI,IMU_QJ,IMU_QK,GYRO_X,
    ANG_ERR,ANG_P,ANG_I,ANG_D,RATE_SP,RATE_ERR,RATE_P,RATE_I,RATE_D,PID_OUT,M1,M2

Plots (single figure with 4 subplots):
    1. Angle tracking — encoder roll vs IMU roll
    2. Rate tracking — GYRO_X (actual) vs RATE_SP (setpoint)
    3. PID terms — angle loop (left axis) + rate loop (right axis)
    4. Motor output — M1, M2 throttle

Statistics (printed to console):
    Sample rate, duration, angle error (MAE/RMS/max/bias), Pearson correlation,
    fast/slow motion MAE split, trail percentage, oscillation frequency,
    angle windup events, rate windup events, encoder/IMU range.
"""

import csv
import sys
from pathlib import Path

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def resolve_run(path_str):
    """Resolve a CLI argument to (csv_path, config_dict_or_None, label).

    Accepts:
      - directory containing log.csv (+ optional config.yaml)
      - direct path to a .csv file (looks for config.yaml in same dir)
    """
    p = Path(path_str)
    if p.is_dir():
        csv_path = p / "log.csv"
        cfg_path = p / "config.yaml"
        label = p.name
    else:
        csv_path = p
        cfg_path = p.parent / "config.yaml"
        label = p.stem
    if not csv_path.exists():
        sys.exit(f"CSV not found: {csv_path}")
    config = _load_config(cfg_path) if cfg_path.exists() else None
    return csv_path, config, label


def _load_config(path):
    """Load config.yaml, returning a dict or None."""
    if yaml is None:
        print("Warning: pyyaml not installed, skipping config parsing")
        return None
    with open(path) as f:
        return yaml.safe_load(f)


def load_csv(path):
    """Load telemetry CSV into a dict of numpy arrays."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    if not rows:
        sys.exit(f"Empty CSV: {path}")
    cols = {}
    for key in rows[0]:
        cols[key] = np.array([float(r[key]) for r in rows])
    return cols


# ---------------------------------------------------------------------------
# Quaternion math
# ---------------------------------------------------------------------------

def quat_to_roll(qr, qi):
    """Extract roll angle (degrees) from quaternion, vectorized.

    Uses the same formula as the Pico hot loop: roll = 2 * atan2(qi, qr).
    Exact for pure X-axis rotation.
    """
    return np.degrees(2.0 * np.arctan2(qi, qr))


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def compute_stats(cols, config):
    """Compute all diagnostic metrics from telemetry columns."""
    n = len(cols["T_MS"])
    if n < 2:
        return None

    t_ms = cols["T_MS"]
    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

    # --- Sample rate ---
    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0
    actual_hz = (n - 1) / duration_s if duration_s > 0 else 0
    dts = np.diff(t_ms)
    dt_mean = np.mean(dts)
    dt_median = np.median(dts)

    # --- Angle error ---
    errors = imu_roll - enc_roll
    abs_errors = np.abs(errors)
    mae = np.mean(abs_errors)
    rms_error = np.sqrt(np.mean(errors ** 2))
    max_ae = np.max(abs_errors)
    bias = np.mean(errors)

    # --- Pearson correlation ---
    if np.std(enc_roll) > 0 and np.std(imu_roll) > 0:
        correlation = np.corrcoef(enc_roll, imu_roll)[0, 1]
    else:
        correlation = 0.0

    # --- Fast/slow motion MAE ---
    enc_vel = np.abs(np.diff(enc_roll)) / (dts / 1000.0)
    vel_order = np.argsort(enc_vel)
    quarter = len(vel_order) // 4
    slow_idx = vel_order[:quarter] + 1  # +1 because diff shifts index
    fast_idx = vel_order[-quarter:] + 1
    mae_slow = np.mean(abs_errors[slow_idx]) if len(slow_idx) > 0 else 0
    mae_fast = np.mean(abs_errors[fast_idx]) if len(fast_idx) > 0 else 0

    # --- Trail percentage ---
    motion_dir = np.diff(enc_roll)
    imu_offset = enc_roll[1:] - imu_roll[1:]
    moving = np.abs(motion_dir) > 1.0
    if np.sum(moving) > 0:
        trailing = (motion_dir[moving] * imu_offset[moving]) > 0
        trail_pct = np.sum(trailing) / np.sum(moving) * 100
    else:
        trail_pct = 0.0

    # --- Range ---
    enc_range = np.ptp(enc_roll)
    imu_range = np.ptp(imu_roll)

    # --- Oscillation frequency (zero-crossing rate of error signal) ---
    sign_changes = np.diff(np.sign(errors))
    zero_crossings = np.sum(sign_changes != 0)
    osc_freq = zero_crossings / (2.0 * duration_s) if duration_s > 0 else 0

    # --- Angle integral windup events ---
    ang_iterm_limit = 100.0  # default
    if config and "angle_pid" in config:
        ang_iterm_limit = config["angle_pid"].get("iterm_limit", 100.0)
    ang_windup_threshold = ang_iterm_limit * 0.5
    ang_i_term = cols["ANG_I"]
    ang_windup_events = np.sum(np.abs(ang_i_term) >= ang_windup_threshold)

    # --- Rate integral windup events ---
    rate_iterm_limit = 50.0  # default
    if config and "rate_pid" in config:
        rate_iterm_limit = config["rate_pid"].get("iterm_limit", 50.0)
    rate_windup_threshold = rate_iterm_limit * 0.5
    rate_i_term = cols["RATE_I"]
    rate_windup_events = np.sum(np.abs(rate_i_term) >= rate_windup_threshold)

    return {
        "n_samples": n,
        "duration_s": duration_s,
        "actual_hz": actual_hz,
        "dt_mean_ms": dt_mean,
        "dt_median_ms": dt_median,
        "mae": mae,
        "rms_error": rms_error,
        "max_ae": max_ae,
        "bias": bias,
        "correlation": correlation,
        "mae_fast": mae_fast,
        "mae_slow": mae_slow,
        "trail_pct": trail_pct,
        "enc_range": enc_range,
        "imu_range": imu_range,
        "osc_freq_hz": osc_freq,
        "ang_windup_events": int(ang_windup_events),
        "ang_windup_threshold": ang_windup_threshold,
        "rate_windup_events": int(rate_windup_events),
        "rate_windup_threshold": rate_windup_threshold,
    }


# ---------------------------------------------------------------------------
# Printing
# ---------------------------------------------------------------------------

def print_config_summary(config, label):
    """Print key config values."""
    if config is None:
        print(f"  (no config.yaml for {label})")
        return
    angle = config.get("angle_pid", {})
    rate = config.get("rate_pid", {})
    motor = config.get("motor", {})
    imu_cfg = config.get("imu", {})
    print(f"  Angle PID: kp={angle.get('kp')}, ki={angle.get('ki')}, "
          f"kd={angle.get('kd')}, iterm_limit={angle.get('iterm_limit')}, "
          f"hz={angle.get('hz')}")
    print(f"  Rate PID:  kp={rate.get('kp')}, ki={rate.get('ki')}, "
          f"kd={rate.get('kd')}, iterm_limit={rate.get('iterm_limit')}, "
          f"hz={rate.get('hz')}")
    if "report_hz" in imu_cfg:
        print(f"  IMU: {imu_cfg.get('report')}, {imu_cfg.get('report_hz')} Hz (single report)")
    else:
        print(f"  IMU: angle={imu_cfg.get('angle_report')} @ {imu_cfg.get('angle_report_hz')} Hz, "
              f"rate={imu_cfg.get('rate_report')} @ {imu_cfg.get('rate_report_hz')} Hz")
    print(f"  Motor: base={motor.get('base_throttle')}, "
          f"min={motor.get('throttle_min')}, max={motor.get('throttle_max')}")


def print_single(label, stats, config):
    """Print stats for a single run."""
    w = 55
    print("=" * w)
    print(f"  {label}")
    print("-" * w)
    print_config_summary(config, label)
    print()
    _print_stats_block(stats)
    print("=" * w)


def print_comparison(label_a, stats_a, config_a, label_b, stats_b, config_b):
    """Print side-by-side comparison table."""
    w = 65
    print("=" * w)
    print(f"  {'Metric':<35} {label_a:>12}   {label_b:>12}")
    print("-" * w)

    print(f"\n  --- Config ---")
    print(f"  Run A:")
    print_config_summary(config_a, label_a)
    print(f"  Run B:")
    print_config_summary(config_b, label_b)

    def row(name, key, fmt=".2f"):
        va = stats_a[key]
        vb = stats_b[key]
        print(f"  {name:<35} {va:>12{fmt}}   {vb:>12{fmt}}")

    print(f"\n  --- Sample Rate ---")
    row("Samples", "n_samples", ".0f")
    row("Duration (s)", "duration_s", ".1f")
    row("Achieved Hz", "actual_hz", ".1f")
    row("Mean dt (ms)", "dt_mean_ms", ".1f")
    row("Median dt (ms)", "dt_median_ms", ".1f")

    print(f"\n  --- Angle Error (deg) ---")
    row("MAE (overall)", "mae")
    row("MAE (fast motion)", "mae_fast")
    row("MAE (slow motion)", "mae_slow")
    row("Max AE", "max_ae")
    row("RMS Error", "rms_error")
    row("Bias (IMU-ENC)", "bias")

    print(f"\n  --- Correlation & Tracking ---")
    row("Pearson r", "correlation", ".4f")
    row("IMU trails motion (%)", "trail_pct", ".1f")
    row("Encoder range (deg)", "enc_range", ".1f")
    row("IMU range (deg)", "imu_range", ".1f")

    print(f"\n  --- Oscillation & Windup ---")
    row("Oscillation freq (Hz)", "osc_freq_hz", ".2f")
    row("Angle windup events", "ang_windup_events", ".0f")
    row("Rate windup events", "rate_windup_events", ".0f")

    print("=" * w)


def _print_stats_block(stats):
    """Print stats in single-column format."""
    def row(name, key, fmt=".2f"):
        print(f"  {name:<35} {stats[key]:>12{fmt}}")

    print("  --- Sample Rate ---")
    row("Samples", "n_samples", ".0f")
    row("Duration (s)", "duration_s", ".1f")
    row("Achieved Hz", "actual_hz", ".1f")
    row("Mean dt (ms)", "dt_mean_ms", ".1f")
    row("Median dt (ms)", "dt_median_ms", ".1f")

    print("\n  --- Angle Error (deg) ---")
    row("MAE (overall)", "mae")
    row("MAE (fast motion)", "mae_fast")
    row("MAE (slow motion)", "mae_slow")
    row("Max AE", "max_ae")
    row("RMS Error", "rms_error")
    row("Bias (IMU-ENC)", "bias")

    print("\n  --- Correlation & Tracking ---")
    row("Pearson r", "correlation", ".4f")
    row("IMU trails motion (%)", "trail_pct", ".1f")
    row("Encoder range (deg)", "enc_range", ".1f")
    row("IMU range (deg)", "imu_range", ".1f")

    print("\n  --- Oscillation & Windup ---")
    row("Oscillation freq (Hz)", "osc_freq_hz", ".2f")
    row("Angle windup events", "ang_windup_events", ".0f")
    print(f"  {'Angle windup threshold':<35} {stats['ang_windup_threshold']:>12.1f}")
    row("Rate windup events", "rate_windup_events", ".0f")
    print(f"  {'Rate windup threshold':<35} {stats['rate_windup_threshold']:>12.1f}")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_run(cols, config, label, axes=None):
    """Plot 5 diagnostic subplots for a single run.

    If axes is provided (list of 5 Axes), plot onto them. Otherwise create a
    new figure.
    """
    t_ms = cols["T_MS"]
    t_s = (t_ms - t_ms[0]) / 1000.0

    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

    own_figure = axes is None
    if own_figure:
        fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
        fig.suptitle(f"Flight Benchy — {label}", fontsize=13)

    ax1, ax2, ax3, ax4, ax5 = axes

    # 1. Angle tracking
    ax1.plot(t_s, enc_roll, label="Encoder", linewidth=0.8)
    ax1.plot(t_s, imu_roll, label="IMU", linewidth=0.8, alpha=0.6)
    ax1.set_ylabel("Roll (deg)")
    ax1.set_title("Angle Tracking")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    # 2. Rate tracking — GYRO_X (actual) vs RATE_SP (setpoint)
    ax2.plot(t_s, cols["GYRO_X"], linewidth=0.6, color="tab:blue",
             alpha=0.7, label="Gyro X (actual)")
    ax2.plot(t_s, cols["RATE_SP"], linewidth=0.8, color="tab:orange",
             label="Rate setpoint")
    ax2.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax2.set_ylabel("Rate (deg/s)")
    ax2.set_title("Rate Tracking (setpoint vs actual)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    # 3. Angle PID terms (outer loop, 50 Hz)
    ax3.plot(t_s, cols["ANG_P"], label="P", linewidth=0.8, color="tab:blue")
    ax3.plot(t_s, cols["ANG_I"], label="I", linewidth=0.8, color="tab:orange")
    ax3.plot(t_s, cols["ANG_D"], label="D", linewidth=0.8, color="tab:green")
    ax3.set_ylabel("Output (deg/s)")
    ax3.set_title("Angle PID (outer loop)")
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(True, alpha=0.3)

    # 4. Rate PID terms (inner loop, 200 Hz)
    ax4.plot(t_s, cols["RATE_P"], label="P", linewidth=0.8, color="tab:blue")
    ax4.plot(t_s, cols["RATE_I"], label="I", linewidth=0.8, color="tab:orange")
    ax4.plot(t_s, cols["RATE_D"], label="D", linewidth=0.8, color="tab:green")
    ax4.set_ylabel("Output (throttle)")
    ax4.set_title("Rate PID (inner loop)")
    ax4.legend(loc="upper right", fontsize=8)
    ax4.grid(True, alpha=0.3)

    # 5. Motor output
    ax5.plot(t_s, cols["M1"], label="M1", linewidth=0.8)
    ax5.plot(t_s, cols["M2"], label="M2", linewidth=0.8)
    ax5.set_ylabel("Throttle")
    ax5.set_xlabel("Time (s)")
    ax5.set_title("Motor Output")
    ax5.legend(loc="upper right", fontsize=8)
    ax5.grid(True, alpha=0.3)

    if own_figure:
        fig.tight_layout()
    return axes


def plot_comparison(runs):
    """Plot two runs side by side (2 columns of 5 subplots)."""
    fig, axes = plt.subplots(5, 2, figsize=(16, 12), sharex="col")
    for col_idx, (cols, config, label) in enumerate(runs):
        col_axes = [axes[row][col_idx] for row in range(5)]
        plot_run(cols, config, label, axes=col_axes)
        axes[0][col_idx].set_title(f"{label}\nAngle Tracking", fontsize=10)
    fig.suptitle("Flight Benchy — Run Comparison", fontsize=13)
    fig.tight_layout()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    save_mode = "--save" in sys.argv

    if not args:
        print("Usage: python tools/analyse_telemetry.py [--save] <run_folder_or_csv> [run2 ...]")
        print("  --save   Save plot as PNG next to log.csv instead of showing interactively")
        sys.exit(1)

    runs = []
    for arg in args:
        csv_path, config, label = resolve_run(arg)
        cols = load_csv(csv_path)
        stats = compute_stats(cols, config)
        runs.append((cols, config, label, stats, csv_path))
        print(f"Loaded {label}: {stats['n_samples']} samples, "
              f"{stats['duration_s']:.1f}s")

    print()

    if len(runs) == 1:
        cols, config, label, stats, csv_path = runs[0]
        print_single(label, stats, config)
        plot_run(cols, config, label)
    elif len(runs) == 2:
        _, config_a, label_a, stats_a, _ = runs[0]
        _, config_b, label_b, stats_b, _ = runs[1]
        print_comparison(label_a, stats_a, config_a, label_b, stats_b, config_b)
        plot_comparison([(r[0], r[1], r[2]) for r in runs])
    else:
        for cols, config, label, stats, _ in runs:
            print_single(label, stats, config)
        for cols, config, label, stats, _ in runs:
            plot_run(cols, config, label)

    if save_mode:
        for i, fig in enumerate(plt.get_fignums()):
            out_path = runs[min(i, len(runs) - 1)][4].parent / "plot.png"
            plt.figure(fig).savefig(out_path, dpi=150, bbox_inches="tight")
            print(f"Saved: {out_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
