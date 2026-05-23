"""
Verdict layer -- what did this run achieve?

Answers: "Did the run reach setpoint? How well did it hold?"

Computes KPIs from encoder telemetry and writes verdict.json to the run folder.
Thresholds and acceptance levels live in specification.json -- not in this output.

verdict.json structure:
  {
    "reached":         bool,
    "time_to_sp_s":    float | null,
    "rise_time_s":     float | null,
    "overshoot_pct":   float | null,
    "damping_ratio":   float | null,
    "settling_time_s": float | null,
    "hold_duration_s": float | null,
    "hold_mae_deg":    float | null
  }

KPI definitions:
  reached        True if encoder entered the tolerance band at least once.
  time_to_sp_s   Seconds from run start to first band entry (T->SP). Null if not reached.
  rise_time_s    10-90% rise time in seconds. Null if step is negligible.
  overshoot_pct  Max excursion past setpoint as % of initial step after first crossing.
                 Null if not reached.
  damping_ratio  Zeta derived from overshoot via canonical 2nd-order formula.
                 Null when overshoot is 0 (overdamped/critically damped -- cannot be
                 recovered from step response alone) or not reached.
  settling_time_s  Confirmed settle start time. Null if not confirmed.
  hold_duration_s  Settled hold duration. Null if not settled.
  hold_mae_deg   Encoder MAE from settling_time_s onward. Null if not settled.

Field names match specification.json KPI keys exactly:
  time_to_sp_s, overshoot_pct, settling_time_s, hold_duration_s, hold_mae_deg
  -- compare verdict.json against specification.json directly, no mapping needed.

Run from project root:
  python .claude/skills/analyse-flight/scripts/verdict.py test_runs/flights/<flight_id>
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from specification_loader import load_specification                          # noqa: E402
from configuration_loader import load_configuration                         # noqa: E402
from flight_data_loader import load_flight, detect_hold_window, detect_settle_window  # noqa: E402


def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: verdict.py test_runs/flights/<flight_id>")

    run_dir = Path(sys.argv[1])
    if not run_dir.is_dir():
        sys.exit(f"Run folder not found: {run_dir}")

    cfg      = load_configuration(run_dir)
    spec     = load_specification(run_dir)
    setpoint = cfg.setpoint_roll_deg

    fd       = load_flight(run_dir / "log.csv")
    hold_w   = detect_hold_window(fd, setpoint, spec.tolerance_deg)
    settle_w = detect_settle_window(fd, hold_w, setpoint, spec.tolerance_deg)

    enc              = fd.enc_roll
    t_ms             = fd.t_ms
    start_angle      = float(enc[0])
    initial_step     = setpoint - start_angle
    initial_step_abs = abs(initial_step)

    # --- Rise time (10-90% of initial step) ---
    rise_time_s = None
    if initial_step_abs > 1e-6:
        mark_10 = start_angle + 0.10 * initial_step
        mark_90 = start_angle + 0.90 * initial_step
        t_10 = t_90 = None
        for i in range(len(enc)):
            a = float(enc[i])
            if t_10 is None:
                if (initial_step < 0 and a <= mark_10) or (initial_step > 0 and a >= mark_10):
                    t_10 = float(t_ms[i])
            if t_90 is None:
                if (initial_step < 0 and a <= mark_90) or (initial_step > 0 and a >= mark_90):
                    t_90 = float(t_ms[i])
            if t_10 is not None and t_90 is not None:
                break
        if t_10 is not None and t_90 is not None:
            rise_time_s = (t_90 - t_10) / 1000.0

    # --- Overshoot (max excursion past setpoint after first crossing) ---
    overshoot_pct = None
    if hold_w is not None and initial_step_abs > 1e-6:
        crossed_idx = None
        for i in range(len(enc)):
            a = float(enc[i])
            if (initial_step < 0 and a <= setpoint) or (initial_step > 0 and a >= setpoint):
                crossed_idx = i
                break
        if crossed_idx is not None:
            post    = enc[crossed_idx:]
            max_exc = float(
                np.max(setpoint - post) if initial_step < 0 else np.max(post - setpoint)
            )
            overshoot_pct = max(0.0, max_exc / initial_step_abs * 100.0)
        else:
            overshoot_pct = 0.0

    # --- Damping ratio zeta from overshoot (valid only when overshoot > 0) ---
    damping_ratio = None
    if overshoot_pct is not None and overshoot_pct > 0.0:
        ln_os = math.log(overshoot_pct / 100.0)
        damping_ratio = -ln_os / math.sqrt(math.pi ** 2 + ln_os ** 2)

    # --- Hold MAE from confirmed settle onward ---
    hold_mae_deg = None
    if settle_w is not None:
        post         = enc[settle_w.start_idx:]
        hold_mae_deg = float(np.mean(np.abs(post - setpoint)))

    result = {
        "reached":         hold_w   is not None,
        "time_to_sp_s":    hold_w.start_time_s   if hold_w   is not None else None,
        "rise_time_s":     rise_time_s,
        "overshoot_pct":   overshoot_pct,
        "damping_ratio":   damping_ratio,
        "settling_time_s": settle_w.start_time_s if settle_w is not None else None,
        "hold_duration_s": settle_w.duration_s   if settle_w is not None else None,
        "hold_mae_deg":    hold_mae_deg,
    }

    analysis_dir = run_dir / "analysis"
    analysis_dir.mkdir(exist_ok=True)
    out_path = analysis_dir / "verdict.json"
    out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"PASS -- wrote {out_path}")


if __name__ == "__main__":
    main()
