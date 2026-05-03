"""
Step 3 of the analyse-flight pipeline. Scores a single flight against the standard test convention.

Standard test convention:
  Start position : M1-end on the restrictor, encoder ≈ +58°
  Goal           : reach within ±10° of the configured setpoint and hold there
  Start OK flag  : start encoder within ±10° of +58°

KPIs (all measured from encoder — ground truth):
  T→0      Seconds from run start to first entry into the ±10° band around setpoint
  HoldMAE  Encoder MAE from setpoint from first reach to end of run — primary hold accuracy metric
  T@0      Total seconds spent inside the ±10° band around setpoint

Run from project root:
  python .claude/skills/analyse-flight/scripts/score_flight.py test_runs/flights/<flight_id>
"""

import csv
import json
import math
import sys
from pathlib import Path

HORIZONTAL_THRESHOLD_DEG = 10.0
STANDARD_START_DEG       = 58.0
START_TOLERANCE_DEG      = 10.0


def load_setpoint(run_dir):
    """Return roll setpoint from config.json in run_dir. Exits on any error."""
    cfg_path = Path(run_dir) / "config.json"
    if not cfg_path.exists():
        sys.exit(f"config.json not found in {run_dir}")
    with open(cfg_path) as f:
        cfg = json.load(f)
    try:
        return float(cfg["bench"]["session"]["setpoint"]["roll_deg"])
    except (KeyError, TypeError) as e:
        sys.exit(f"config.json missing bench.session.setpoint.roll_deg: {e}")


def enc_angle(qr, qi):
    return math.degrees(2.0 * math.atan2(qi, qr))



def load_angles(csv_path):
    result = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            t = float(row["T_MS"])
            angle = enc_angle(float(row["ENC_QR"]), float(row["ENC_QI"]))
            result.append((t, angle))
    return result


def compute_kpis(samples, setpoint):
    if not samples:
        return None

    t0, a0 = samples[0]
    start_angle = a0
    start_ok = abs(start_angle - STANDARD_START_DEG) <= START_TOLERANCE_DEG

    reached_idx = None
    for i, (t, a) in enumerate(samples):
        if abs(a - setpoint) <= HORIZONTAL_THRESHOLD_DEG:
            reached_idx = i
            break

    reached = reached_idx is not None
    if reached:
        t_reach, _ = samples[reached_idx]
        time_to_s = (t_reach - t0) / 1000.0
        post = [a for _, a in samples[reached_idx:]]
        hold_mae = sum(abs(a - setpoint) for a in post) / len(post)
        time_at_s = sum(
            (samples[i + 1][0] - samples[i][0]) / 1000.0
            for i in range(len(samples) - 1)
            if abs(samples[i][1] - setpoint) <= HORIZONTAL_THRESHOLD_DEG
            and abs(samples[i + 1][1] - setpoint) <= HORIZONTAL_THRESHOLD_DEG
        )
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


def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: score_flight.py test_runs/flights/<flight_id>")

    run_dir = Path(sys.argv[1])
    csv_path = run_dir / "log.csv"

    if not csv_path.exists():
        sys.exit(f"log.csv not found in {run_dir}")

    samples = load_angles(csv_path)
    if len(samples) < 5:
        sys.exit(f"Too few samples ({len(samples)}) — run may be corrupt")

    setpoint = load_setpoint(run_dir)
    k = compute_kpis(samples, setpoint)
    start_flag = "ok" if k["start_ok"] else "!!"
    reached_str = "YES" if k["reached"] else "NO"
    t_to  = f"{k['time_to_s']:.1f}s" if k["reached"] else "-"
    h_mae = f"{k['hold_mae']:.2f}" if k["reached"] else "-"
    t_at  = f"{k['time_at_s']:.1f}s"

    header = (f"{'Run':<26} {'Start':>7} {'OK':>3} {'Reached':>8} "
              f"{'T->SP (s)':>10} {'HoldMAE':>9} {'T@SP (s)':>10} {'Dur (s)':>8}")
    sep = "-" * len(header)
    print(header)
    print(sep)
    print(f"{run_dir.name:<26} {k['start_angle']:>6.1f}  {start_flag:>3} "
          f"{reached_str:>8} {t_to:>10} {h_mae:>9} {t_at:>10} "
          f"{k['duration_s']:>7.1f}s")
    print(sep)

    if k["reached"]:
        print(f"\nPassed — use profile_flight.py for deep dive:")
        print(f"  python .claude/skills/analyse-flight/scripts/profile_flight.py {run_dir}")

    if not k["start_ok"]:
        print(f"\nNon-standard start — lever was not at the restrictor "
              f"(expected {STANDARD_START_DEG}° ± {START_TOLERANCE_DEG}°, "
              f"got {k['start_angle']:+.1f}°). KPIs are not comparable to standard runs.")


if __name__ == "__main__":
    main()