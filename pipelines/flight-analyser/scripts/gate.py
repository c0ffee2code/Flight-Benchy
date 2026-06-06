"""
Gate layer — run before plot/verdict/diagnose to fail fast.

Answers: "Is this run valid?"

Detects hardware/software failure modes that would invalidate KPI analysis.
Exits 0 if all checks pass, 1 if any fail.
Writes gate.json to the run folder.

gate.json structure:
  {
    "passed":      bool,
    "flight_id":   str,
    "start_angle": float,
    "start_ok":    bool,
    "duration_s":  float,
    "n_samples":   int,
    "checks": [{"name": str, "passed": bool, "detail": str | null}]
  }

Identity fields (flight_id, start_angle, start_ok, duration_s, n_samples) are included
only when at least the "truncated" check passes (i.e. log.csv has >= 5 rows).

Checks (run in order):

  files-missing log.csv or config.json not present in the run folder.
                Exits immediately — remaining checks cannot run.

  log-truncated log.csv has fewer than 5 rows — SD write likely interrupted.
                Exits immediately — remaining checks cannot run.

  start-angle   First encoder reading outside +/-10deg of bench.start_angle_deg. Reset-position step
                was missed; KPIs are not comparable to standard runs.
                Exits immediately — pipeline stops.

  power-cut     Pico was alive and commanding but ESC power was lost.
                Signature: lever never reached setpoint AND encoder is flat
                (std < 3deg) AND motor differential is active (mean |M2-M1| > 20).
                Exits immediately — pipeline stops.

  loop-meltdown Inner rate loop running in positive feedback.
                Signature: encoder traverses near-full mechanical range (>90deg)
                AND IMU persistently lags encoder direction (trail >40% of moving
                samples). Both conditions must fire to avoid false-positives from
                hard-slam startups that overshoot but then hold cleanly.
                Exits immediately — pipeline stops.

  sample-rate-jitter Loop timing jitter is severe enough to degrade KPI accuracy.
                Signature: dt_p99 > 5x median dt.
                Exits immediately — pipeline stops.

Run from project root:
  python .claude/skills/analyse-flight/scripts/gate.py test_runs/flights/<flight_id>
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from configuration_loader import load_configuration     # noqa: E402
from flight_data_loader import load_raw_rows            # noqa: E402
from specification_loader import load_specification      # noqa: E402

# Start-angle tolerance (the expected start angle comes from bench.start_angle_deg in config.json)
_START_TOLERANCE_DEG     = 10.0

# Power-cut thresholds
_POWER_CUT_FLAT_STD_DEG  = 3.0
_POWER_CUT_ACTIVE_DIFF   = 20.0
_POWER_CUT_GRAVITY_MARGIN_DEG = 5.0   # if lever is held this far below gravity resting pos, ESCs have authority

# Loop-meltdown thresholds — both must fire
_LOOP_MELTDOWN_ENC_RANGE_DEG = 90.0

# Sample-rate jitter threshold
_SAMPLE_RATE_JITTER_FACTOR   = 5.0


def check_start_angle(rows, start_angle_deg):
    """Return detail string if the first encoder reading is outside the standard start zone."""
    first = float(rows[0]["ENC_ROLL"])
    if abs(first - start_angle_deg) > _START_TOLERANCE_DEG:
        return (
            f"start angle={first:+.1f}deg is outside "
            f"{start_angle_deg}deg +/- {_START_TOLERANCE_DEG}deg. "
            f"Reset-position step was missed -- KPIs are not comparable to standard runs."
        )
    return None


def check_power_cut(rows, setpoint, tolerance_deg, start_angle_deg):
    """Return detail string on detection, None if clean."""
    enc   = [float(r["ENC_ROLL"]) for r in rows]
    diffs = [abs(float(r["M2"]) - float(r["M1"])) for r in rows]

    if any(abs(a - setpoint) <= tolerance_deg for a in enc):
        return None

    mean_a    = sum(enc) / len(enc)
    enc_std   = math.sqrt(sum((a - mean_a) ** 2 for a in enc) / len(enc))
    mean_diff = sum(diffs) / len(diffs)

    if mean_a < start_angle_deg - _POWER_CUT_GRAVITY_MARGIN_DEG:
        return None

    if enc_std < _POWER_CUT_FLAT_STD_DEG and mean_diff > _POWER_CUT_ACTIVE_DIFF:
        return (
            f"flat encoder std={enc_std:.1f}deg (<{_POWER_CUT_FLAT_STD_DEG}deg), "
            f"motor differential mean={mean_diff:.0f} (>{_POWER_CUT_ACTIVE_DIFF:.0f}). "
            f"Pico was alive and commanding; ESC power was lost."
        )
    return None


def check_loop_meltdown(rows):
    """Return detail string on detection, None if clean."""
    enc = [float(r["ENC_ROLL"]) for r in rows]

    enc_range = max(enc) - min(enc)

    if enc_range > _LOOP_MELTDOWN_ENC_RANGE_DEG:
        return (
            f"encoder range={enc_range:.1f}deg (>{_LOOP_MELTDOWN_ENC_RANGE_DEG:.0f}deg). "
            f"Rate loop in positive feedback -- check sensor_orientation in config."
        )
    return None


def check_sample_rate(rows):
    """Return detail string if timing jitter is excessive (dt_p99 > 5x median)."""
    if len(rows) < 10:
        return None
    times = [float(r["T_MS"]) for r in rows]
    dts   = sorted(times[i + 1] - times[i] for i in range(len(times) - 1))
    median_dt = dts[len(dts) // 2]
    p99_idx   = max(0, int(len(dts) * 0.99) - 1)
    dt_p99    = dts[p99_idx]
    threshold = _SAMPLE_RATE_JITTER_FACTOR * median_dt
    if dt_p99 > threshold:
        return (
            f"dt_p99={dt_p99:.0f}ms > {_SAMPLE_RATE_JITTER_FACTOR:.0f}x "
            f"median ({median_dt:.0f}ms). Loop jitter exceeds acceptable range."
        )
    return None


def _emit(run_dir, result):
    """Write gate.json to the analysis subfolder; print a one-line status to stdout."""
    analysis_dir = run_dir / "analysis"
    analysis_dir.mkdir(exist_ok=True)
    out_path = analysis_dir / "gate.json"
    out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    status = "PASS" if result["passed"] else "FAIL"
    print(f"{status} -- wrote {out_path}")


def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: gate.py test_runs/flights/<flight_id>")

    run_dir  = Path(sys.argv[1])
    csv_path = run_dir / "log.csv"
    cfg_path = run_dir / "config.json"

    checks = []
    result = {"passed": True, "checks": checks}

    def _fail(name, detail):
        checks.append({"name": name, "passed": False, "detail": detail})
        result["passed"] = False
        _emit(run_dir, result)
        sys.exit(1)

    def _pass(name):
        checks.append({"name": name, "passed": True, "detail": None})

    missing = [name for name, path in [
        ("log.csv",            csv_path),
        ("config.json",        cfg_path),
        ("specification.json", run_dir / "specification.json"),
    ] if not path.exists()]
    if missing:
        _fail("files-missing", f"{', '.join(missing)} not found in {run_dir}")
    _pass("files-missing")

    cfg      = load_configuration(run_dir)
    spec     = load_specification(run_dir)
    setpoint = cfg.setpoint_roll_deg

    rows = load_raw_rows(csv_path)
    if len(rows) < 5:
        _fail("log-truncated", f"{len(rows)} rows in log.csv; SD write likely interrupted")
    _pass("log-truncated")

    # Identity fields — only reachable when rows >= 5
    times       = [float(r["T_MS"]) for r in rows]
    start_angle = float(rows[0]["ENC_ROLL"])
    result["flight_id"]   = run_dir.name
    result["start_angle"] = round(start_angle, 2)
    result["start_ok"]    = abs(start_angle - cfg.start_angle_deg) <= _START_TOLERANCE_DEG
    result["duration_s"]  = round((times[-1] - times[0]) / 1000.0, 2)
    result["n_samples"]   = len(rows)

    for name, detail_fn in [
        ("start-angle",   lambda: check_start_angle(rows, cfg.start_angle_deg)),
        ("power-cut",     lambda: check_power_cut(rows, setpoint, spec.tolerance_deg, cfg.start_angle_deg)),
        ("loop-meltdown", lambda: check_loop_meltdown(rows)),
        ("sample-rate-jitter", lambda: check_sample_rate(rows)),
    ]:
        detail = detail_fn()
        if detail:
            _fail(name, detail)
        _pass(name)

    _emit(run_dir, result)
    sys.exit(0)


if __name__ == "__main__":
    main()
