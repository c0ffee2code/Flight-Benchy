"""
Plot Flight Benchy telemetry run(s).

Generates a 5-subplot diagnostic figure and saves it as plot.png next to
each log.csv. Pass one run folder for a single plot, two for side-by-side.

Usage:
    python tools/plot.py test_runs/2026-04-07_10-00-00
    python tools/plot.py test_runs/run_a test_runs/run_b
"""

import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_run(path_str):
    """Resolve path string to (csv_path, cols_dict, label)."""
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


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_run(cols, label, axes=None):
    t_s = (cols["T_MS"] - cols["T_MS"][0]) / 1000.0
    enc_roll = quat_to_roll(cols["ENC_QR"], cols["ENC_QI"])
    imu_roll = quat_to_roll(cols["IMU_QR"], cols["IMU_QI"])

    own_figure = axes is None
    if own_figure:
        fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
        fig.suptitle(f"Flight Benchy — {label}", fontsize=13)

    ax1, ax2, ax3, ax4, ax5 = axes

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
    ax5.set_xlabel("Time (s)")
    ax5.set_title("Motor Output")
    ax5.legend(loc="upper right", fontsize=8)
    ax5.grid(True, alpha=0.3)

    if own_figure:
        fig.tight_layout()
    return axes


def plot_comparison(runs):
    fig, axes = plt.subplots(5, 2, figsize=(16, 12), sharex="col")
    for col_idx, (cols, label) in enumerate(runs):
        col_axes = [axes[row][col_idx] for row in range(5)]
        plot_run(cols, label, axes=col_axes)
        axes[0][col_idx].set_title(f"{label}\nAngle Tracking", fontsize=10)
    fig.suptitle("Flight Benchy — Run Comparison", fontsize=13)
    fig.tight_layout()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python tools/plot.py <run_folder> [run2]")
        sys.exit(1)

    runs = []
    for arg in sys.argv[1:]:
        csv_path, cols, label = load_run(arg)
        runs.append((csv_path, cols, label))
        print(f"Loaded {label}: {len(cols['T_MS'])} samples")

    if len(runs) == 1:
        csv_path, cols, label = runs[0]
        plot_run(cols, label)
        out = csv_path.parent / "plot.png"
        plt.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Saved: {out}")
    elif len(runs) == 2:
        plot_comparison([(c, l) for _, c, l in runs])
        out = runs[0][0].parent / f"compare_{runs[1][2]}.png"
        plt.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Saved: {out}")
    else:
        for csv_path, cols, label in runs:
            plot_run(cols, label)
            out = csv_path.parent / "plot.png"
            plt.savefig(out, dpi=150, bbox_inches="tight")
            print(f"Saved: {out}")
            plt.close()


if __name__ == "__main__":
    main()