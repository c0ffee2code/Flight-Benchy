"""
Step 1 of the analyse-flight pipeline. Always run first.

Generates a 5-subplot diagnostic figure and saves it as plot.png next to log.csv.
One flight folder in, one plot out.

Five subplots (top → bottom):
  1. Angle tracking   — Encoder (ground truth) vs IMU roll. Check for phase lag
                        and bias between the two signals.
  2. Rate tracking    — Rate setpoint (outer-loop output) vs calibrated gyro.
                        Confirms the inner loop is following the outer loop.
  3. Outer PID terms  — Angle P/I/D contributions (deg/s). Sustained I-term
                        indicates steady-state disturbance (friction, imbalance,
                        tare error).
  4. Inner PID terms  — Rate P/I/D contributions (throttle). Spiky D indicates
                        high gyro noise or rate_kd too large.
  5. Motor outputs    — M1 / M2 throttle. Symmetric hold = balanced frame;
                        asymmetry = mechanical bias or I-term compensation.

Run from project root:
  python .claude/skills/analyse-flight/scripts/plot.py test_runs/<flight_id>
"""

import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker


def load_run(path_str):
    p = Path(path_str)
    csv_path = (p / "log.csv") if p.is_dir() else p
    label = p.name if p.is_dir() else p.stem
    if not csv_path.exists():
        sys.exit(f"CSV not found: {csv_path}")
    rows = []
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        sys.exit(f"Empty CSV: {csv_path}")
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    return csv_path, cols, label


def quat_to_roll(qr, qi):
    return np.degrees(2.0 * np.arctan2(qi, qr))


def _time_formatter(duration_s):
    if duration_s >= 60:
        def fmt(x, _):
            m, s = divmod(int(x), 60)
            return f"{m}:{s:02d}"
        return mticker.FuncFormatter(fmt), "Time (m:ss)"
    return None, "Time (s)"


def plot_run(cols, label):
    t_s = (cols["T_MS"] - cols["T_MS"][0]) / 1000.0
    duration_s = t_s[-1]
    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    fig.suptitle(f"Flight Benchy — {label}  ({duration_s:.1f}s)", fontsize=13)

    ax1.axhline(0, color="gray", linewidth=0.8, linestyle="--", label="Setpoint (0°)")
    ax1.plot(t_s, enc_roll, label="Encoder", linewidth=0.8)
    ax1.plot(t_s, imu_roll, label="IMU", linewidth=0.8, alpha=0.6)
    ax1.set_ylabel("Roll (deg)")
    ax1.set_title("Angle Tracking")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2.plot(t_s, cols["GYRO_X"], linewidth=0.6, color="tab:blue", alpha=0.7, label="Gyro X (actual)")
    ax2.plot(t_s, cols["RATE_SP"], linewidth=0.8, color="tab:orange", label="Rate setpoint")
    ax2.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax2.set_ylabel("Rate (deg/s)")
    ax2.set_title("Rate Tracking (setpoint vs actual)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    ax3.plot(t_s, cols["ANG_P"], label="P", linewidth=0.8, color="tab:blue")
    ax3.plot(t_s, cols["ANG_I"], label="I", linewidth=0.8, color="tab:orange")
    ax3.plot(t_s, cols["ANG_D"], label="D", linewidth=0.8, color="tab:green")
    ax3.set_ylabel("Output (deg/s)")
    ax3.set_title("Angle PID (outer loop)")
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(True, alpha=0.3)

    ax4.plot(t_s, cols["RATE_P"], label="P", linewidth=0.8, color="tab:blue")
    ax4.plot(t_s, cols["RATE_I"], label="I", linewidth=0.8, color="tab:orange")
    ax4.plot(t_s, cols["RATE_D"], label="D", linewidth=0.8, color="tab:green")
    ax4.set_ylabel("Output (throttle)")
    ax4.set_title("Rate PID (inner loop)")
    ax4.legend(loc="upper right", fontsize=8)
    ax4.grid(True, alpha=0.3)

    ax5.plot(t_s, cols["M1"], label="M1", linewidth=0.8)
    ax5.plot(t_s, cols["M2"], label="M2", linewidth=0.8)
    ax5.set_ylabel("Throttle")
    ax5.set_title("Motor Output")
    ax5.legend(loc="upper right", fontsize=8)
    ax5.grid(True, alpha=0.3)

    fmt, xlabel = _time_formatter(duration_s)
    ax5.set_xlabel(xlabel)
    if fmt:
        ax5.xaxis.set_major_formatter(fmt)

    fig.tight_layout()
    return fig


def main():
    if len(sys.argv) != 2:
        print("Usage: python .claude/skills/analyse-flight/scripts/plot.py <flight_folder>")
        sys.exit(1)

    csv_path, cols, label = load_run(sys.argv[1])
    print(f"Loaded {label}: {len(cols['T_MS'])} samples")

    plot_run(cols, label)
    out = csv_path.parent / "plot.png"
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved: {out}")


if __name__ == "__main__":
    main()