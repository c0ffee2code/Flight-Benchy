"""
Step 1 of the analyse-flight pipeline. Run before plot and score to fail fast.

Detects hardware/software failure modes that would invalidate KPI analysis.
Exits 0 if all checks pass, 1 if any fail — callers can gate on the exit code.

Checks (run in order):

  missing       log.csv or config.json not present in the run folder.
                Exits immediately — remaining checks cannot run.

  truncated     log.csv has fewer than 5 rows — SD write likely interrupted.
                Exits immediately — remaining checks cannot run.

  start-angle   First encoder reading outside ±10° of +58°. Reset-position step
                was missed; KPIs are not comparable to standard runs.
                Exits immediately — pipeline stops.

  power-cut     Pico was alive and commanding but ESC power was lost.
                Signature: lever never reached setpoint AND encoder is flat
                (std < 3°) AND motor differential is active (mean |M2-M1| > 20).
                Exits immediately — pipeline stops.

  loop-meltdown Inner rate loop running in positive feedback — lever hits both
                mechanical stops continuously.
                Signature: encoder traverses near-full mechanical range (>90°)
                AND IMU persistently lags encoder direction (trail >40% of moving
                samples). Both conditions must fire to avoid false-positives from
                hard-slam startups that overshoot but then hold cleanly.
                Exits immediately — pipeline stops.

Run from project root:
  python .claude/skills/analyse-flight/scripts/smoke.py test_runs/flights/<flight_id>
"""

import csv
import json
import math
import sys
from pathlib import Path

# Start-angle thresholds
_STANDARD_START_DEG      = 58.0  # expected first encoder reading (M1-end on restrictor)
_START_TOLERANCE_DEG     = 10.0  # ±band around standard start

# Power-cut thresholds
_POWER_CUT_FLAT_STD_DEG  = 3.0   # encoder std below this → lever stationary
_POWER_CUT_ACTIVE_DIFF   = 20.0  # mean |M2-M1| above this → loop was commanding
_SETPOINT_BAND_DEG       = 10.0  # ±band used to decide "reached setpoint"

# Loop-meltdown thresholds — both must fire
_LOOP_MELTDOWN_ENC_RANGE_DEG = 90.0  # lever traversed near-full mechanical range
_LOOP_MELTDOWN_IMU_TRAIL_PCT = 40.0  # IMU persistently lags encoder direction


def _quat_to_angle(qr, qi):
    return math.degrees(2.0 * math.atan2(float(qi), float(qr)))


def _load(csv_path):
    with open(csv_path, newline="") as f:
        return list(csv.DictReader(f))


def check_start_angle(rows):
    """Return detail string if the first encoder reading is outside the standard start zone."""
    first = _quat_to_angle(rows[0]["ENC_QR"], rows[0]["ENC_QI"])
    if abs(first - _STANDARD_START_DEG) > _START_TOLERANCE_DEG:
        return (
            f"start angle={first:+.1f}° is outside "
            f"{_STANDARD_START_DEG}° ± {_START_TOLERANCE_DEG}°. "
            f"Reset-position step was missed — KPIs are not comparable to standard runs."
        )
    return None


def check_power_cut(rows, setpoint):
    """Return detail string on detection, None if clean."""
    enc  = [_quat_to_angle(r["ENC_QR"], r["ENC_QI"]) for r in rows]
    diffs = [abs(float(r["M2"]) - float(r["M1"])) for r in rows]

    # If the lever reached the setpoint zone, ESCs were clearly working.
    if any(abs(a - setpoint) <= _SETPOINT_BAND_DEG for a in enc):
        return None

    mean_a    = sum(enc) / len(enc)
    enc_std   = math.sqrt(sum((a - mean_a) ** 2 for a in enc) / len(enc))
    mean_diff = sum(diffs) / len(diffs)

    if enc_std < _POWER_CUT_FLAT_STD_DEG and mean_diff > _POWER_CUT_ACTIVE_DIFF:
        return (
            f"flat encoder std={enc_std:.1f}° (<{_POWER_CUT_FLAT_STD_DEG}°), "
            f"motor differential mean={mean_diff:.0f} (>{_POWER_CUT_ACTIVE_DIFF:.0f}). "
            f"Pico was alive and commanding; ESC power was lost."
        )
    return None


def check_loop_meltdown(rows):
    """Return detail string on detection, None if clean."""
    enc = [_quat_to_angle(r["ENC_QR"], r["ENC_QI"]) for r in rows]
    imu = [_quat_to_angle(r["IMU_QR"], r["IMU_QI"]) for r in rows]

    enc_range = max(enc) - min(enc)

    moving_total = trailing = 0
    for i in range(len(enc) - 1):
        motion = enc[i + 1] - enc[i]
        if abs(motion) <= 1.0:
            continue
        moving_total += 1
        # IMU trails when it lags behind encoder direction
        if motion * (enc[i + 1] - imu[i + 1]) > 0:
            trailing += 1
    trail_pct = (trailing / moving_total * 100.0) if moving_total > 0 else 0.0

    if enc_range > _LOOP_MELTDOWN_ENC_RANGE_DEG and trail_pct > _LOOP_MELTDOWN_IMU_TRAIL_PCT:
        return (
            f"encoder range={enc_range:.1f}° (>{_LOOP_MELTDOWN_ENC_RANGE_DEG:.0f}°), "
            f"IMU trail={trail_pct:.1f}% (>{_LOOP_MELTDOWN_IMU_TRAIL_PCT:.0f}%). "
            f"Rate loop in positive feedback — check sensor_orientation in config."
        )
    return None


def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: smoke.py test_runs/flights/<flight_id>")

    run_dir  = Path(sys.argv[1])
    csv_path = run_dir / "log.csv"
    cfg_path = run_dir / "config.json"

    missing = [name for name, path in [("log.csv", csv_path), ("config.json", cfg_path)]
               if not path.exists()]
    if missing:
        sys.exit(f"[FAIL] missing       — {', '.join(missing)} not found in {run_dir}")

    with open(cfg_path) as f:
        cfg = json.load(f)
    try:
        setpoint = float(cfg["bench"]["session"]["setpoint"]["roll_deg"])
    except (KeyError, TypeError) as e:
        sys.exit(f"config.json missing bench.session.setpoint.roll_deg: {e}")

    rows = _load(csv_path)
    if len(rows) < 5:
        sys.exit(f"[FAIL] truncated     — {len(rows)} rows in log.csv; SD write likely interrupted")

    for name, detail in [
        ("start-angle",   check_start_angle(rows)),
        ("power-cut",     check_power_cut(rows, setpoint)),
        ("loop-meltdown", check_loop_meltdown(rows)),
    ]:
        if detail:
            print(f"[FAIL] {name:<13} — {detail}")
            sys.exit(1)
        print(f"[PASS] {name}")

    sys.exit(0)


if __name__ == "__main__":
    main()
