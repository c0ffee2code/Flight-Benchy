"""
Step 2 of the analyse-flight pipeline. Scores a single flight against the standard test convention.

Standard test convention:
  Start position : M1-end on the restrictor, encoder ≈ +58°
  Goal           : reach within ±10° of horizontal (0°) and hold there
  Start OK flag  : start encoder within ±10° of +58°

KPIs (all measured from encoder — ground truth):
  T→0      Seconds from run start to first entry into the ±10° band
  HoldMAE  Encoder MAE from first reach to end of run — primary hold accuracy metric
  T@0      Total seconds spent inside the ±10° band

Run from project root:
  python .claude/skills/analyse-flight/scripts/score_flight.py test_runs/flights/<flight_id>
"""

import csv
import math
import sys
from pathlib import Path

HORIZONTAL_THRESHOLD_DEG = 10.0
STANDARD_START_DEG       = 58.0
START_TOLERANCE_DEG      = 10.0

# Power-cut detection thresholds
FLAT_ENCODER_STD_DEG  = 3.0   # encoder std dev below this → lever stationary
ACTIVE_DIFF_THROTTLE  = 20.0  # mean |M2-M1| above this → loop was actively commanding


def enc_angle(qr, qi):
    return math.degrees(2.0 * math.atan2(qi, qr))


def detect_power_cut(csv_path):
    """Return a description string if the power-cut pattern is detected, else None.

    Signature: flat encoder (std < FLAT_ENCODER_STD_DEG) combined with an active
    motor differential (mean |M2-M1| > ACTIVE_DIFF_THROTTLE). Indicates the Pico
    was alive and commanding but ESC power was lost (over-current protection,
    battery cutoff, wiring failure).
    """
    angles, diffs = [], []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            angles.append(enc_angle(float(row["ENC_QR"]), float(row["ENC_QI"])))
            diffs.append(abs(float(row["M2"]) - float(row["M1"])))

    if len(angles) < 5:
        return None

    mean_a   = sum(angles) / len(angles)
    enc_std  = math.sqrt(sum((a - mean_a) ** 2 for a in angles) / len(angles))
    mean_diff = sum(diffs) / len(diffs)

    if enc_std < FLAT_ENCODER_STD_DEG and mean_diff > ACTIVE_DIFF_THROTTLE:
        return (f"POWER CUT — flat encoder (std={enc_std:.1f}° < {FLAT_ENCODER_STD_DEG}°) "
                f"with active motor differential (mean |M2-M1|={mean_diff:.0f}). "
                f"Pico was alive and commanding; ESC power was lost.")
    return None


def load_angles(csv_path):
    result = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            t = float(row["T_MS"])
            angle = enc_angle(float(row["ENC_QR"]), float(row["ENC_QI"]))
            result.append((t, angle))
    return result


def compute_kpis(samples):
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
        time_at_s = sum(
            (samples[i + 1][0] - samples[i][0]) / 1000.0
            for i in range(len(samples) - 1)
            if abs(samples[i][1]) <= HORIZONTAL_THRESHOLD_DEG
            and abs(samples[i + 1][1]) <= HORIZONTAL_THRESHOLD_DEG
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

    k = compute_kpis(samples)
    start_flag = "ok" if k["start_ok"] else "!!"
    reached_str = "YES" if k["reached"] else "NO"
    t_to  = f"{k['time_to_s']:.1f}s" if k["reached"] else "-"
    h_mae = f"{k['hold_mae']:.2f}" if k["reached"] else "-"
    t_at  = f"{k['time_at_s']:.1f}s"

    header = (f"{'Run':<26} {'Start':>7} {'OK':>3} {'Reached':>8} "
              f"{'T->0 (s)':>10} {'HoldMAE':>9} {'T@0 (s)':>10} {'Dur (s)':>8}")
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
    else:
        mode = detect_power_cut(csv_path)
        if mode:
            print(f"\nWARNING: {mode}")

    if not k["start_ok"]:
        print(f"\nNon-standard start — lever was not at the restrictor "
              f"(expected {STANDARD_START_DEG}° ± {START_TOLERANCE_DEG}°, "
              f"got {k['start_angle']:+.1f}°). KPIs are not comparable to standard runs.")


if __name__ == "__main__":
    main()