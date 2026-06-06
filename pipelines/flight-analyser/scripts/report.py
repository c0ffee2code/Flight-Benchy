"""
Report layer -- human-readable summary for the operator.

Human view only. All data is available in gate.json, verdict.json, and diagnose.json.
This file adds no information beyond what those files contain.

Reads gate.json, verdict.json, diagnose.json, config.json, and specification.json from the
run folder. Renders templates/flight_analysis.md via string.Template substitution.
Acceptance levels are computed on-the-fly from specification.json -- never persisted.

Writes test_runs/flights/<id>/analysis/summary.md.

Prerequisites: gate.py, plots.py, verdict.py, and diagnose.py must have already been run.

Usage:
  python .claude/skills/analyse-flight/scripts/report.py <run_folder>
"""

import argparse
import json
import sys
from pathlib import Path
from string import Template

sys.path.insert(0, str(Path(__file__).parent))
from specification_loader import load_specification, KpiDirection  # noqa: E402
from configuration_loader import load_configuration               # noqa: E402

_TEMPLATE = Path(__file__).parent.parent / "templates" / "summary.md"


def _fmt(val, fmt=".2f", suffix=""):
    if val is None:
        return "-"
    return f"{val:{fmt}}{suffix}"


def _load_json(path):
    if not path.exists():
        sys.exit(f"Required file not found: {path} -- run the pipeline scripts first")
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _score_level(value, kpi_spec):
    if value is None:
        return "-"
    if kpi_spec.direction == KpiDirection.LOWER_IS_BETTER:
        if value <= kpi_spec.threshold_excellent:
            return "excellent"
        if value <= kpi_spec.threshold_good:
            return "good"
        if value <= kpi_spec.threshold_pass:
            return "pass"
        return "below_pass"
    else:
        if value >= kpi_spec.threshold_excellent:
            return "excellent"
        if value >= kpi_spec.threshold_good:
            return "good"
        if value >= kpi_spec.threshold_pass:
            return "pass"
        return "below_pass"


def _zeta_str(kpis):
    dr = kpis.get("damping_ratio")
    op = kpis.get("overshoot_pct")
    if dr is not None:
        return f"{dr:.3f}"
    if op is not None and op == 0.0:
        return ">=1 (no OS)"
    return "-"


def _fft_str(hold):
    f = hold.get("fft_freq_hz")
    r = hold.get("fft_freq_resolution_hz")
    if f is None:
        return "-"
    s = f"{f:.3f}"
    if r is not None and f < 3.0 * r:
        s += " (advisory)"
    return s


def _iterm_sign_str(effort):
    v = effort.get("iterm_sign_ok")
    if v is None:
        return "N/A (P-term dominant)"
    return "OK" if v else "FLIP - sign error in control chain"


def build_report(run_folder_str):
    run_dir = Path(run_folder_str)

    gate    = _load_json(run_dir / "analysis" / "gate.json")
    kpis    = _load_json(run_dir / "analysis" / "verdict.json")
    profile = _load_json(run_dir / "analysis" / "diagnose.json")
    config  = load_configuration(run_dir)
    spec    = load_specification(run_dir)

    sr = profile["sample_rate"]
    ct = profile.get("cycle_timing") or {}
    sh = profile["sensor_health"]
    ht = profile["hold_tracking"]
    at = profile.get("approach_tracking") or {}
    ce = profile["control_effort"]
    il = profile["inner_loop"]
    wu = profile["windup"]

    t_to_sp   = kpis.get("time_to_sp_s")
    overshoot = kpis.get("overshoot_pct")
    settling  = kpis.get("settling_time_s")
    hold_dur  = kpis.get("hold_duration_s")
    hold_mae  = kpis.get("hold_mae_deg")
    ff_lead   = _fmt(config.feedforward_lead_ms, ".0f") if config.feedforward_lead_ms is not None else "-"

    return Template(_TEMPLATE.read_text(encoding="utf-8")).substitute(
        flight_id      = gate["flight_id"],
        duration_s     = _fmt(gate.get("duration_s"), ".1f", " s"),
        n_samples      = str(gate.get("n_samples", "-")),
        start_angle    = _fmt(gate.get("start_angle"), ".1f", " deg"),
        standard_start = "YES" if gate.get("start_ok") else "NO",

        angle_pid_row  = (f"kp={config.loops.angle.pid.kp}, ki={config.loops.angle.pid.ki}, "
                          f"kd={config.loops.angle.pid.kd}, iterm_limit={config.loops.angle.pid.iterm_limit}"),
        rate_pid_row   = (f"kp={config.loops.rate.pid.kp}, ki={config.loops.rate.pid.ki}, "
                          f"kd={config.loops.rate.pid.kd}, iterm_limit={config.loops.rate.pid.iterm_limit}"),
        motor_row      = (f"base={config.motor.base_throttle}, "
                          f"min={config.motor.throttle_min}, max={config.motor.throttle_max}"),
        ff_lead_ms     = ff_lead,
        angle_report   = f"{config.loops.angle.imu_report} @ {config.loops.angle.frequency_hz} Hz",
        rate_report    = f"{config.loops.rate.imu_report} @ {config.loops.rate.frequency_hz} Hz",

        reached              = "YES" if kpis.get("reached") else "NO",
        t_to_sp              = _fmt(t_to_sp,  ".1f"),
        t_to_sp_level        = _score_level(t_to_sp,   spec.time_to_sp_s),
        rise_time            = _fmt(kpis.get("rise_time_s"), ".1f"),
        overshoot            = _fmt(overshoot, ".1f", "%"),
        overshoot_level      = _score_level(overshoot,  spec.overshoot_pct),
        damping_ratio        = _zeta_str(kpis),
        settling_time        = _fmt(settling,  ".1f"),
        settling_time_level  = _score_level(settling,   spec.settling_time_s),
        hold_duration        = _fmt(hold_dur,  ".1f"),
        hold_duration_level  = _score_level(hold_dur,   spec.hold_duration_s),
        hold_mae             = _fmt(hold_mae,  ".2f"),
        hold_mae_level       = _score_level(hold_mae,   spec.hold_mae_deg),

        nominal_hz   = _fmt(sr["nominal_hz"],   ".1f"),
        actual_hz    = _fmt(sr["actual_hz"],    ".1f"),
        dt_mean_ms   = _fmt(sr["dt_mean_ms"],   ".1f"),
        dt_median_ms = _fmt(sr["dt_median_ms"], ".1f"),
        dt_p99_ms    = _fmt(sr["dt_p99_ms"],    ".1f"),
        dt_max_ms    = _fmt(sr["dt_max_ms"],    ".1f"),

        ct_nominal_ms        = _fmt(ct.get("nominal_inner_ms"), ".2f"),
        ct_dt_mean_ms        = _fmt(ct.get("dt_mean_ms"),       ".1f"),
        ct_dt_p99_ms         = _fmt(ct.get("dt_p99_ms"),        ".1f"),
        ct_dt_max_ms         = _fmt(ct.get("dt_max_ms"),        ".1f"),
        ct_max_dt_mean_ms    = _fmt(ct.get("max_dt_mean_ms"),   ".1f"),
        ct_max_dt_p99_ms     = _fmt(ct.get("max_dt_p99_ms"),    ".1f"),
        ct_max_dt_max_ms     = _fmt(ct.get("max_dt_max_ms"),    ".1f"),
        ct_spike_count       = str(ct.get("spike_count", "-")),
        ct_spike_pct         = _fmt(ct.get("spike_pct"),        ".1f"),
        ct_spike_interval_ms = _fmt(ct.get("spike_interval_ms"), ".0f"),

        sh_mae       = _fmt(sh["mae"]),
        sh_mae_fast  = _fmt(sh["mae_fast"]),
        sh_mae_slow  = _fmt(sh["mae_slow"]),
        sh_bias      = _fmt(sh["bias"]),

        ht_bias      = f"{ht['bias']:+.2f}",
        ht_std       = _fmt(ht["std"]),
        ht_p95       = _fmt(ht["p95"]),
        ht_max_ae    = _fmt(ht["max_ae"]),
        ht_fft_freq  = _fft_str(ht),

        at_bias      = _fmt(at.get("bias"), "+.2f") if at.get("bias") is not None else "-",
        at_std       = _fmt(at.get("std")),
        at_p95       = _fmt(at.get("p95")),
        at_max_ae    = _fmt(at.get("max_ae")),
        at_pearson_r = _fmt(at.get("pearson_r"), ".4f"),
        at_duration  = _fmt(at.get("duration_s"), ".1f"),

        ce_mean_throttle = _fmt(ce["mean_throttle"], ".1f"),
        ce_sat_upper     = _fmt(ce["saturation_upper_pct"], ".1f"),
        ce_sat_lower     = _fmt(ce["saturation_lower_pct"], ".1f"),
        ce_rms_dm1       = _fmt(ce["rms_dm1_dt"], ".1f"),
        ce_rms_dm2       = _fmt(ce["rms_dm2_dt"], ".1f"),
        ce_ang_i_mean    = _fmt(ce["ang_i_mean"]),
        ce_m2_m1_mean    = f"{ce['m2_m1_mean']:+.1f}",
        ce_iterm_sign    = _iterm_sign_str(ce),

        il_rate_rms    = _fmt(il["rate_tracking_rms"]),

        wu_ang_events  = str(wu["ang_windup_events"]),
        wu_rate_events = str(wu["rate_windup_events"]),
    )


def main():
    parser = argparse.ArgumentParser(
        description="Write summary.md from gate.json, verdict.json, diagnose.json, and config.json",
    )
    parser.add_argument("run_folder",
                        help="Path to run folder, e.g. test_runs/flights/2026-05-20_14-30-00")
    args = parser.parse_args()

    run_dir = Path(args.run_folder)
    if not run_dir.is_dir():
        sys.exit(f"Run folder not found: {run_dir}")

    report       = build_report(args.run_folder)
    analysis_dir = run_dir / "analysis"
    analysis_dir.mkdir(exist_ok=True)
    out_path     = analysis_dir / "summary.md"
    out_path.write_text(report, encoding="utf-8")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
