---
name: analyse-flight
description: Analyse a single Flight Benchy stabilisation run end-to-end — generate the diagnostic plot, compute KPIs (T→SP, HoldMAE_s), profile sensor and control-loop health, and write a structured analysis.md report. Use this whenever the user points at a single run folder under test_runs/ (typically a YYYY-MM-DD_hh-mm-ss timestamp) and wants to know how it went. Triggers on phrases like "analyse flight", "check this flight". Does NOT cover comparison across multiple runs, PID tuning recommendations, or general drone control theory — for those, answer directly without invoking this skill.
---

## Arguments

```
<flight_id>
```

- `<flight_id>` — required. The run folder name under `test_runs/flights/` (e.g. `2026-05-01_19-19-05`).

# Analyse Flight

Procedural skill for evaluating a single Flight Benchy stabilisation run. Every flight goes through the same pipeline and produces a consistent `flight_analysis.md`, so runs stay directly comparable over time.

## What is a flight

A flight is one stabilisation session stored in `test_runs/flights/YYYY-MM-DD_hh-mm-ss/`. The folder contains:

- `log.csv` — 22-column telemetry, typically ~50 ms / 20 Hz (inner loop runs faster; `TELEMETRY_SAMPLE_EVERY` decimates).
- `config.json` — full system snapshot at run time. Structure: `vehicle` (imu, angle_pid, rate_pid, motor, feedforward), `bench` (encoder; session with duration_s and setpoint.roll_deg/pitch_deg/yaw_deg), `telemetry` (sample_every).
- `specification.json` — acceptance spec in effect at the time of this run. Copied from the project root by `SdSink` at session start. **Mandatory** — all pipeline scripts exit immediately if absent.

Standard test convention: M1-end resting on the restrictor at ≈ **+58°**, algorithm lifts the lever to within `±tolerance_deg` of horizontal (0°) and holds (tolerance read from `specification.json`). Non-standard starts are analysed for the paper trail but are **discarded** — their KPIs cannot be compared to standard runs and must not inform tuning decisions.

## KPI reference

| `specification.json` key | KPI name | Definition | Direction |
|---------------------|----------|------------|-----------|
| `hold_mae_deg` | HoldMAE | Mean absolute encoder error during the settled hold phase (T_s onward) | lower is better |
| `time_to_sp_s` | T→SP | Seconds from run start to first entry into the tolerance band | lower is better |
| `settling_time_s` | T_s | Seconds from run start to confirmed settled hold (no band exits through end of run) | lower is better |
| `hold_duration_s` | Hold Duration | Seconds of confirmed settled hold from T_s to end of run | higher is better |
| `overshoot_pct` | Overshoot | Peak excursion past setpoint after first crossing, as % of initial step | lower is better |

Scoring levels: **excellent → good → pass → below_pass**. Level is `null` when the value cannot be computed (run did not reach setpoint). Thresholds and descriptions live in `specification.json` — the table above is the lookup key between code field names and human-readable KPI names.

## Hard rules — apply to report AND conversation

These apply everywhere during this skill — in the analysis.md Observations and in any conversational text:

- **No battery/power speculation.** Do not assume or speculate about battery state, voltage sag, or supply condition as a cause. The user will say so if it happened.
- **No mechanical speculation.** Do not assume or speculate about loose connections, frame balance, wire tension, or mechanical changes as causes. The user will say so if it happened.
- **No IMU speculation.** Do not assume or speculate about tare drift, calibration quality, or heading error as causes. The user will say so if it happened.
- **No run-to-run attribution.** When variance is observed between runs at identical config, state it as variance. Do not list hypothetical causes. Describe what the telemetry shows; stop there.

The user is the domain expert on physical changes. They will volunteer that information — do not fish for it.

## Pipeline contract

Scripts are run in order. Each writes a structured JSON file; stdout is a one-line confirmation.
Read the JSON files (not stdout) to understand the results.

```
Layer       Script       Output          Question answered
──────────────────────────────────────────────────────────────────────────
gate        gate.py      gate.json       "Is this run valid?"
verdict     verdict.py   verdict.json    "What did it achieve?"
diagnosis   diagnose.py  diagnose.json   "Why did it perform that way?"
            plots.py      PNGs            (visual reference, no JSON)
            report.py    summary.md      (human-readable view, no extra information)
```

gate.json     {passed, flight_id, start_angle, start_ok, duration_s, n_samples,
               checks:[{name, passed, detail}]}; exits 1 on any failure

verdict.json  {reached, time_to_sp_s, rise_time_s, overshoot_pct, damping_ratio,
               settling_time_s, hold_duration_s, hold_mae_deg}
              Raw measured values only. Field names match specification.json KPI keys exactly.
              Acceptance levels are not stored here -- compare against specification.json directly.

diagnose.json {sample_rate, sensor_health, hold_tracking, control_effort, inner_loop, windup}

Run all of them in order. If gate exits 1, stop -- read gate.json to see which check failed.

## Step 1 — Gate checks

```
python .claude/skills/analyse-flight/scripts/gate.py test_runs/flights/<flight_id>
```

Run first. Validates the run folder and detects failure modes that would invalidate KPI analysis. Exits 0 if all checks pass, 1 if any fail. Writes gate.json. Thresholds are in the script's module-level constants.

| Check | Signature |
|-------|-----------|
| **files-missing** | `log.csv` or `config.json` not present. Exits immediately. |
| **log-truncated** | `log.csv` has fewer than 5 rows — SD write interrupted. Exits immediately. |
| **start-angle** | First encoder reading outside 58° ± 10°. Reset-position step was missed. Exits immediately. |
| **power-cut** | Lever never reached setpoint + flat encoder (std < 3°) + active motor differential. Exits immediately. |
| **loop-meltdown** | Encoder range > 90° AND IMU trail > 40%. Rate loop in positive feedback. Exits immediately. |
| **sample-rate-jitter** | dt_p99 > 5× median dt. Loop jitter severe enough to degrade KPI accuracy. Exits immediately. |

If gate exits 1, **stop**. Do not run Steps 2–5. Report the check name and detail to the user. A `postmortem` skill is planned for investigating these failures (not yet available).

## Step 2 — Plot

```
python .claude/skills/analyse-flight/scripts/plots.py test_runs/flights/<flight_id>
```

Produces the following numbered PNGs, ordered by diagnostic importance:
- `01_timeseries.png` — five subplots: angle tracking (ENC vs IMU), rate tracking (setpoint vs gyro), outer PID terms, inner PID terms, motor outputs. Primary reference.
- `02_step_response.png` — full run with milestones (T→SP, T_s, OS) + transient zoom with rise time.
- `03_spectrum.png` — PSD of hold-window error. Skipped if no confirmed settled hold.
- `04_hold_error_distribution.png` — hold-error histogram. Skipped if no confirmed settled hold.
- `05_phase_portrait.png` — phase portrait (angle vs angular rate), time-coloured trajectory.

The plot docstrings have the full per-subplot guide.

## Step 3 — Verdict

```
python .claude/skills/analyse-flight/scripts/verdict.py test_runs/flights/<flight_id>
```

Computes the transient KPIs and writes `verdict.json` to the run folder. KPI definitions live in the script's docstring. The headline number is **HoldMAE_s** — encoder MAE from settling time onward, excluding the overshoot phase. The transient quartet (T->SP, rise time 10-90%, overshoot %, settling time T_s) characterises the approach; HoldMAE_s characterises the hold.

`verdict.json` contains raw measured values only — flat floats, null when not computable. Field names match `specification.json` KPI keys exactly. Acceptance levels are not stored in verdict.json; compare against specification.json directly when needed.

If `Reached = NO`, skip steps 4 and the hold sections of the report. State the fact plainly: "Did not reach setpoint." The plot and KPI table show what happened — no further interpretation needed in Observations.

## Step 4 — Diagnose

```
python .claude/skills/analyse-flight/scripts/diagnose.py test_runs/flights/<flight_id>
```

Prints grouped diagnostics across six sections: sample rate, sensor health (IMU vs ENC, whole-run), hold-window tracking (bias/std/P95/Pearson r/FFT frequency), control effort (throttle mean/RMS/saturation/dM·dt), inner-loop tracking RMS, and windup events. Group meanings are in the script's docstring.

Two things to watch for in the output:

- The "Whole-run ENC MAE includes the rise" note in the hold-window section. It is shown for reference only — use HoldMAE_s (post-settle) as the headline hold-accuracy number.
- Windup event counts are computed against the `iterm_limit` values from `config.json`. They cover the whole run, not just the hold window.

## Step 5 — Report

```
python .claude/skills/analyse-flight/scripts/report.py test_runs/flights/<flight_id>
```

Reads gate.json, verdict.json, diagnose.json, and config.json. Computes acceptance levels
on-the-fly from specification.json and writes `test_runs/flights/<flight_id>/analysis/summary.md`.

`summary.md` is a human-readable view only — it contains no information beyond what the JSON
files already hold. The main model reads the JSON files directly when reasoning about a flight.