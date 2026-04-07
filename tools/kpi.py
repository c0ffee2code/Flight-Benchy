"""
Quick KPI check for Flight Benchy runs.

Scans all runs in test_runs/ (or specific folders passed as arguments) and
prints a compact pass/fail table against the standard starting-position KPIs.

Standard starting position: M1 down, encoder approx -59 deg.

KPIs:
    1. Correct start  - encoder within 10 deg of -59 deg
    2. Reached 0 deg  - encoder ever entered +-10 deg of horizontal
    3. Time to reach  - seconds from run start to first entering that band
    4. Hold MAE       - encoder MAE from first reach to end of run

Usage:
    python tools/kpi.py                        # scan all runs in test_runs/
    python tools/kpi.py test_runs/2026-04-07*  # specific runs
"""

import csv
import math
import sys
from pathlib import Path

HORIZONTAL_THRESHOLD_DEG = 10.0
STANDARD_START_DEG       = -59.0
START_TOLERANCE_DEG      = 10.0


def enc_angle(qr, qi):
    """Quaternion component pair to roll angle in degrees."""
    return math.degrees(2.0 * math.atan2(qi, qr))


def load_angles(csv_path):
    """Return list of (t_ms, enc_deg) from a log.csv."""
    result = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            t = float(row["T_MS"])
            angle = enc_angle(float(row["ENC_QR"]), float(row["ENC_QI"]))
            result.append((t, angle))
    return result


def compute_kpis(samples):
    """Return KPI dict for a list of (t_ms, enc_deg) samples."""
    if not samples:
        return None

    t0, a0 = samples[0]
    start_angle = a0
    start_ok = abs(start_angle - STANDARD_START_DEG) <= START_TOLERANCE_DEG

    reached_idx = None
    for i, (t, a) in enumerate(samples):
        if abs(a) <= HORIZONTAL_THRESHOLD_DEG:
            reached_idx = i
            break

    reached = reached_idx is not None
    if reached:
        t_reach, _ = samples[reached_idx]
        time_to_s = (t_reach - t0) / 1000.0
        post = [a for _, a in samples[reached_idx:]]
        hold_mae = sum(abs(a) for a in post) / len(post)
        at_horiz = sum(1 for _, a in samples if abs(a) <= HORIZONTAL_THRESHOLD_DEG)
        dt_mean_ms = (samples[-1][0] - t0) / max(len(samples) - 1, 1)
        time_at_s = at_horiz * dt_mean_ms / 1000.0
    else:
        time_to_s = None
        hold_mae = None
        time_at_s = 0.0

    return {
        "start_angle": start_angle,
        "start_ok": start_ok,
        "reached": reached,
        "time_to_s": time_to_s,
        "hold_mae": hold_mae,
        "time_at_s": time_at_s,
        "duration_s": (samples[-1][0] - t0) / 1000.0,
        "n": len(samples),
    }


def find_runs(paths):
    """Resolve CLI args to a sorted list of run directories."""
    if paths:
        dirs = []
        for p in paths:
            p = Path(p)
            if p.is_dir() and (p / "log.csv").exists():
                dirs.append(p)
        return sorted(dirs)
    base = Path("test_runs")
    if not base.exists():
        sys.exit("test_runs/ not found - run from the project root")
    return sorted(d for d in base.iterdir()
                  if d.is_dir() and (d / "log.csv").exists())


def main():
    args = sys.argv[1:]
    runs = find_runs(args)
    if not runs:
        sys.exit("No runs found.")

    header = (f"{'Run':<26} {'Start':>7} {'OK':>3} {'Reached':>8} "
              f"{'T->0 (s)':>10} {'HoldMAE':>9} {'T@0 (s)':>10} {'Dur (s)':>8}")
    sep = "-" * len(header)
    print(header)
    print(sep)

    passed = []
    for run_dir in runs:
        csv_path = run_dir / "log.csv"
        try:
            samples = load_angles(csv_path)
        except Exception as e:
            print(f"{run_dir.name:<26}  error: {e}")
            continue
        if len(samples) < 5:
            continue

        k = compute_kpis(samples)
        start_flag = "ok" if k["start_ok"] else "!!"
        reached_str = "YES" if k["reached"] else "NO"
        t_to  = f"{k['time_to_s']:.1f}s" if k["reached"] else "-"
        h_mae = f"{k['hold_mae']:.2f}" if k["reached"] else "-"
        t_at  = f"{k['time_at_s']:.1f}s"

        print(f"{run_dir.name:<26} {k['start_angle']:>6.1f}  {start_flag:>3} "
              f"{reached_str:>8} {t_to:>10} {h_mae:>9} {t_at:>10} "
              f"{k['duration_s']:>7.1f}s")

        if k["reached"]:
            passed.append(run_dir.name)

    print(sep)
    print(f"\n{len(passed)}/{len(runs)} runs reached horizontal "
          f"(threshold +-{HORIZONTAL_THRESHOLD_DEG} deg).")
    if passed:
        print("Passed - use analyse_telemetry.py for deep dive:")
        for r in passed:
            print(f"  python tools/analyse_telemetry.py test_runs/{r}")


if __name__ == "__main__":
    main()