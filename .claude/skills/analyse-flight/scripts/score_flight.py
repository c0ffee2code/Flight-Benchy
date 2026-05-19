"""
Step 3 of the analyse-flight pipeline. Scores a single flight against the standard test convention.

Standard test convention:
  Start position : M1-end on the restrictor, encoder ≈ +58°
  Goal           : reach within ±tolerance_deg of the configured setpoint and hold there
  Start OK flag  : start encoder within ±10° of +58°

KPIs (all measured from encoder — ground truth):
  T->SP          Seconds from run start to first entry into the tolerance band around setpoint.
  Rise time     10–90% of the initial step — rate of approach (excludes pre-spin phase).
  Overshoot     Max excursion past setpoint as % of initial step after first setpoint crossing.
  T_s           Settling time — first moment the encoder enters the band and stays inside for
                at least SETTLING_MIN_HOLD_S seconds continuously through end of run.
                Returns None if the run ends before confirming SETTLING_MIN_HOLD_S of hold.
  HoldMAE_s     Encoder MAE from settling time onward — pure hold quality, no overshoot.
                None if the run never confirms SETTLING_MIN_HOLD_S of settled hold.

Acceptance criteria are read from specification.json in the run folder (mandatory).

Run from project root:
  python .claude/skills/analyse-flight/scripts/score_flight.py test_runs/flights/<flight_id>
"""

# Python >= 3.11

import csv
import json
import math
import sys
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

STANDARD_START_DEG  = 58.0
START_TOLERANCE_DEG = 10.0
SETTLING_MIN_HOLD_S = 5.0


# ---------------------------------------------------------------------------
# Specification model
# ---------------------------------------------------------------------------

class KpiDirection(StrEnum):
    LOWER_IS_BETTER  = "lower_is_better"
    HIGHER_IS_BETTER = "higher_is_better"


@dataclass
class KpiSpec:
    direction:           KpiDirection
    threshold_pass:      float
    threshold_good:      float
    threshold_excellent: float


@dataclass
class Specification:
    tolerance_deg:   float
    hold_mae_deg:    KpiSpec
    time_to_sp_s:    KpiSpec
    settling_time_s: KpiSpec
    hold_duration_s: KpiSpec
    overshoot_pct:   KpiSpec


@dataclass
class ScoredKpi:
    value: float | None
    level: str | None   # "excellent" | "good" | "pass" | "below_pass" | None
    spec:  KpiSpec

    def to_dict(self):
        return {
            "value": self.value,
            "level": self.level,
            "thresholds": {
                "pass":      self.spec.threshold_pass,
                "good":      self.spec.threshold_good,
                "excellent": self.spec.threshold_excellent,
            },
        }


@dataclass
class ScoredRun:
    reached:         bool
    hold_mae_deg:    ScoredKpi
    time_to_sp_s:    ScoredKpi
    settling_time_s: ScoredKpi
    hold_duration_s: ScoredKpi
    overshoot_pct:   ScoredKpi

    def to_dict(self):
        return {
            "reached":         self.reached,
            "hold_mae_deg":    self.hold_mae_deg.to_dict(),
            "time_to_sp_s":    self.time_to_sp_s.to_dict(),
            "settling_time_s": self.settling_time_s.to_dict(),
            "hold_duration_s": self.hold_duration_s.to_dict(),
            "overshoot_pct":   self.overshoot_pct.to_dict(),
        }


def load_specification(run_dir):
    """Load and parse specification.json from run_dir. Exits immediately if absent or malformed."""
    path = Path(run_dir) / "specification.json"
    if not path.exists():
        sys.exit(f"specification.json not found in {run_dir}")
    with open(path) as f:
        raw = json.load(f)

    def _kpi(key):
        try:
            entry = raw["kpis"][key]
            return KpiSpec(
                direction=KpiDirection(entry["direction"]),
                threshold_pass=float(entry["thresholds"]["pass"]),
                threshold_good=float(entry["thresholds"]["good"]),
                threshold_excellent=float(entry["thresholds"]["excellent"]),
            )
        except (KeyError, TypeError, ValueError) as err:
            sys.exit(f"specification.json malformed at kpis.{key}: {err}")

    try:
        tolerance_deg = float(raw["tolerance_deg"])
    except (KeyError, TypeError) as e:
        sys.exit(f"specification.json missing tolerance_deg: {e}")

    return Specification(
        tolerance_deg=tolerance_deg,
        hold_mae_deg=_kpi("hold_mae_deg"),
        time_to_sp_s=_kpi("time_to_sp_s"),
        settling_time_s=_kpi("settling_time_s"),
        hold_duration_s=_kpi("hold_duration_s"),
        overshoot_pct=_kpi("overshoot_pct"),
    )


def score_kpi_levels(kpis, spec):
    """Score a KpiResult against a Specification. Returns dict[str, ScoredKpi]."""
    def _score(value, kpi_spec):
        if value is None:
            return ScoredKpi(value=None, level=None, spec=kpi_spec)
        if kpi_spec.direction == KpiDirection.LOWER_IS_BETTER:
            if value <= kpi_spec.threshold_excellent:
                level = "excellent"
            elif value <= kpi_spec.threshold_good:
                level = "good"
            elif value <= kpi_spec.threshold_pass:
                level = "pass"
            else:
                level = "below_pass"
        else:
            if value >= kpi_spec.threshold_excellent:
                level = "excellent"
            elif value >= kpi_spec.threshold_good:
                level = "good"
            elif value >= kpi_spec.threshold_pass:
                level = "pass"
            else:
                level = "below_pass"
        return ScoredKpi(value=value, level=level, spec=kpi_spec)

    return ScoredRun(
        reached=kpis.reached,
        hold_mae_deg=    _score(kpis.hold_mae_settled,  spec.hold_mae_deg),
        time_to_sp_s=    _score(kpis.time_to_s,         spec.time_to_sp_s),
        settling_time_s= _score(kpis.settling_time_s,   spec.settling_time_s),
        hold_duration_s= _score(kpis.hold_duration_s,   spec.hold_duration_s),
        overshoot_pct=   _score(kpis.overshoot_pct,     spec.overshoot_pct),
    )




@dataclass
class KpiResult:
    """
    KPI and diagnostic values for one flight run, computed by compute_kpis().

    Fields
    ------
    start_angle      : Encoder angle at the first sample, degrees.
    start_ok         : True if start_angle is within ±START_TOLERANCE_DEG of
                       STANDARD_START_DEG (standard M1-end-down position).
    reached          : True if the encoder entered the ±HORIZONTAL_THRESHOLD_DEG
                       band around setpoint at least once.
    time_to_s        : Seconds from run start to first band entry (T->SP); None if
                       not reached.
    rise_time_s      : 10–90% rise time in seconds; None if step is negligible.
    overshoot_pct    : Max excursion past setpoint as % of initial step after first
                       crossing; None if not reached.
    damping_ratio    : Damping ratio ζ derived from overshoot via the canonical
                       2nd-order formula; None if overshoot is 0 or not reached.
    settling_time_s  : Time of first entry into the band that is never left for
                       ≥ SETTLING_MIN_HOLD_S seconds; None if not confirmed.
    hold_duration_s  : duration_s − settling_time_s — clean hold length; None if
                       settling not confirmed.
    hold_mae_settled : Encoder MAE from settling_time_s onward; None if settling
                       not confirmed.
    hold_start_idx   : Sample index of the first band entry; None if not reached.
    settle_start_idx : Sample index of settling start; None if settling not confirmed.
    duration_s       : Total run duration, seconds.
    n                : Total number of samples.
    """
    start_angle:      float
    start_ok:         bool
    reached:          bool
    time_to_s:        float | None
    rise_time_s:      float | None
    overshoot_pct:    float | None
    damping_ratio:    float | None
    settling_time_s:  float | None
    hold_duration_s:  float | None
    hold_mae_settled: float | None
    hold_start_idx:   int | None
    settle_start_idx: int | None
    duration_s:       float
    n:                int


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


def load_motor_limits(run_dir):
    """Return (throttle_min, throttle_max) from config.json in run_dir. Exits on any error."""
    cfg_path = Path(run_dir) / "config.json"
    if not cfg_path.exists():
        sys.exit(f"config.json not found in {run_dir}")
    with open(cfg_path) as f:
        cfg = json.load(f)
    try:
        motor = cfg["vehicle"]["motor"]
        return float(motor["throttle_min"]), float(motor["throttle_max"])
    except (KeyError, TypeError) as e:
        sys.exit(f"config.json missing vehicle.motor throttle_min/throttle_max: {e}")


def enc_angle(qr, qi):
    return math.degrees(2.0 * math.atan2(qi, qr))


def load_angles(csv_path):
    result = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            result.append((float(row["T_MS"]),
                           enc_angle(float(row["ENC_QR"]), float(row["ENC_QI"]))))
    return result


def compute_kpis(samples, setpoint, tolerance_deg):
    if not samples:
        return None

    t0, a0       = samples[0]
    start_angle  = a0
    start_ok     = abs(start_angle - STANDARD_START_DEG) <= START_TOLERANCE_DEG

    initial_step     = setpoint - start_angle   # e.g. 0 − 58 = −58
    initial_step_abs = abs(initial_step)

    # T->SP: first entry into ±tolerance_deg band
    reached_idx = None
    for i, (t, a) in enumerate(samples):
        if abs(a - setpoint) <= tolerance_deg:
            reached_idx = i
            break
    reached = reached_idx is not None

    # Rise time 10–90%
    rise_time_s = None
    if initial_step_abs > 1e-6:
        mark_10 = start_angle + 0.10 * initial_step
        mark_90 = start_angle + 0.90 * initial_step
        t_10 = t_90 = None
        for t, a in samples:
            if t_10 is None:
                if (initial_step < 0 and a <= mark_10) or (initial_step > 0 and a >= mark_10):
                    t_10 = t
            if t_90 is None:
                if (initial_step < 0 and a <= mark_90) or (initial_step > 0 and a >= mark_90):
                    t_90 = t
        if t_10 is not None and t_90 is not None:
            rise_time_s = (t_90 - t_10) / 1000.0

    # Overshoot % — max excursion past setpoint as % of initial step
    overshoot_pct = None
    if reached and initial_step_abs > 1e-6:
        crossed_idx = None
        for i, (t, a) in enumerate(samples):
            if (initial_step < 0 and a <= setpoint) or (initial_step > 0 and a >= setpoint):
                crossed_idx = i
                break
        if crossed_idx is not None:
            if initial_step < 0:
                max_exc = max(setpoint - a for _, a in samples[crossed_idx:])
            else:
                max_exc = max(a - setpoint for _, a in samples[crossed_idx:])
            overshoot_pct = max(0.0, max_exc / initial_step_abs * 100.0)
        else:
            overshoot_pct = 0.0

    # Damping ratio ζ — derived from overshoot via the canonical 2nd-order formula.
    # Valid only when overshoot > 0 (underdamped). OS=0 implies ζ ≥ 1 (overdamped or
    # critically damped); the exact value cannot be recovered from step response alone.
    damping_ratio = None
    if overshoot_pct is not None and overshoot_pct > 0.0:
        ln_os = math.log(overshoot_pct / 100.0)
        damping_ratio = -ln_os / math.sqrt(math.pi ** 2 + ln_os ** 2)

    time_to_s = (samples[reached_idx][0] - t0) / 1000.0 if reached else None

    # Settling time + HoldMAE_settled
    # Scan backward from end to find last sample outside the band (at or after reach)
    settling_time_s  = None
    hold_duration_s  = None
    hold_mae_settled = None
    settle_idx       = None
    if reached:
        last_out = None
        for i in range(len(samples) - 1, reached_idx - 1, -1):
            if abs(samples[i][1] - setpoint) > tolerance_deg:
                last_out = i
                break
        if last_out is None:
            settle_idx = reached_idx          # never left the band after reach
        elif last_out + 1 < len(samples):
            settle_idx = last_out + 1
        # else: run ended with encoder outside the band — settle_idx stays None

        if settle_idx is not None:
            t_settle    = samples[settle_idx][0]
            settled_dur = (samples[-1][0] - t_settle) / 1000.0
            if settled_dur >= SETTLING_MIN_HOLD_S:
                settling_time_s  = (t_settle - t0) / 1000.0
                hold_duration_s  = settled_dur
                post_s           = [a for _, a in samples[settle_idx:]]
                hold_mae_settled = sum(abs(a - setpoint) for a in post_s) / len(post_s)

    duration_s = (samples[-1][0] - t0) / 1000.0

    return KpiResult(
        start_angle=start_angle,
        start_ok=start_ok,
        reached=reached,
        time_to_s=time_to_s,
        rise_time_s=rise_time_s,
        overshoot_pct=overshoot_pct,
        damping_ratio=damping_ratio,
        settling_time_s=settling_time_s,
        hold_duration_s=hold_duration_s,
        hold_mae_settled=hold_mae_settled,
        hold_start_idx=reached_idx,
        settle_start_idx=settle_idx,
        duration_s=duration_s,
        n=len(samples),
    )


def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: score_flight.py test_runs/flights/<flight_id>")

    run_dir  = Path(sys.argv[1])
    csv_path = run_dir / "log.csv"

    if not csv_path.exists():
        sys.exit(f"log.csv not found in {run_dir}")

    spec = load_specification(run_dir)
    samples  = load_angles(csv_path)
    if len(samples) < 5:
        sys.exit(f"Too few samples ({len(samples)}) — run may be corrupt")

    setpoint = load_setpoint(run_dir)
    k        = compute_kpis(samples, setpoint, spec.tolerance_deg)
    scored   = score_kpi_levels(k, spec)

    with open(run_dir / "kpis.json", "w") as f:
        json.dump(scored.to_dict(), f, indent=2)

    def _fmt(val, fmt, suffix=""):
        return f"{val:{fmt}}{suffix}" if val is not None else "-"

    start_flag  = "ok" if k.start_ok else "!!"
    reached_str = "YES" if k.reached else "NO"
    t_to        = _fmt(k.time_to_s,        ".1f", "s")
    h_mae_s     = _fmt(k.hold_mae_settled, ".2f", "deg")

    header = (f"{'Run':<26} {'Start':>8} {'OK':>3} {'Reached':>8} "
              f"{'T->SP (s)':>10} {'HoldMAE_s (deg)':>16} {'Dur (s)':>8}")
    sep = "-" * len(header)
    print(header)
    print(sep)
    print(f"{run_dir.name:<26} {k.start_angle:>4.1f}deg {start_flag:>3} "
          f"{reached_str:>8} {t_to:>10} {h_mae_s:>16} "
          f"{k.duration_s:>7.1f}s")
    print(sep)

    print(f"\n  Acceptance levels (tolerance: +/-{spec.tolerance_deg:.0f} deg):")
    for name, sk in [
        ("hold_mae_deg",    scored.hold_mae_deg),
        ("time_to_sp_s",    scored.time_to_sp_s),
        ("settling_time_s", scored.settling_time_s),
        ("hold_duration_s", scored.hold_duration_s),
        ("overshoot_pct",   scored.overshoot_pct),
    ]:
        val_str = f"{sk.value:.2f}" if sk.value is not None else "-"
        lvl_str = sk.level if sk.level is not None else "-"
        print(f"    {name:<20} {val_str:>8}  ->  {lvl_str}")

    if k.reached:
        rise  = _fmt(k.rise_time_s,    ".1f", "s")
        os_   = _fmt(k.overshoot_pct,  ".1f", "%")
        ts    = _fmt(k.settling_time_s, ".1f", "s")
        if k.damping_ratio is not None:
            zeta_str = f"{k.damping_ratio:.3f}"
        elif k.overshoot_pct is not None and k.overshoot_pct == 0.0:
            zeta_str = ">=1 (no OS)"
        else:
            zeta_str = "-"
        print(f"\n  Rise 10-90%: {rise:<8}  Overshoot: {os_:<8}  T_s (settling): {ts}")
        print(f"  Damping ratio zeta: {zeta_str}")
        print(f"\nPassed - use profile_flight.py for deep dive:")
        print(f"  python .claude/skills/analyse-flight/scripts/profile_flight.py {run_dir}")

    if not k.start_ok:
        print(f"\nNon-standard start - lever was not at the restrictor "
              f"(expected {STANDARD_START_DEG}deg +/- {START_TOLERANCE_DEG}deg, "
              f"got {k.start_angle:+.1f}deg). KPIs are not comparable to standard runs.")


if __name__ == "__main__":
    main()
