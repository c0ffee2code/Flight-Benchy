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

Standard test convention: M1-end resting on the restrictor at ≈ **+58°**, algorithm lifts the lever to within ±10° of horizontal (0°) and holds. Non-standard starts are analysed for the paper trail but are **discarded** — their KPIs cannot be compared to standard runs and must not inform tuning decisions.

## Hard rules — apply to report AND conversation

These apply everywhere during this skill — in the analysis.md Observations and in any conversational text:

- **No battery/power speculation.** Do not assume or speculate about battery state, voltage sag, or supply condition as a cause. The user will say so if it happened.
- **No mechanical speculation.** Do not assume or speculate about loose connections, frame balance, wire tension, or mechanical changes as causes. The user will say so if it happened.
- **No IMU speculation.** Do not assume or speculate about tare drift, calibration quality, or heading error as causes. The user will say so if it happened.
- **No run-to-run attribution.** When variance is observed between runs at identical config, state it as variance. Do not list hypothetical causes. Describe what the telemetry shows; stop there.

The user is the domain expert on physical changes. They will volunteer that information — do not fish for it.

## Pipeline contract

Following scripts are designed to be run in order:

```
smoke.py          → stdout ([PASS]/[FAIL] per check); exits 1 on any failure
plots.py          → 01_timeseries.png, 02_step_response.png, 03_spectrum.png, 04_hold_error_distribution.png, 05_phase_portrait.png
score_flight.py   → stdout (KPI pass/fail table);  compute_kpis() importable by profile_flight.py
profile_flight.py → imports compute_kpis directly;  prints canonical KPIs then grouped diagnostics
```

Run all of them in order. If smoke exits 1, stop — do not run next steps.

## Step 1 — Smoke checks

```
python .claude/skills/analyse-flight/scripts/smoke.py test_runs/flights/<flight_id>
```

Run first. Validates the run folder and detects failure modes that would invalidate KPI analysis. Exits 0 if all checks pass, 1 if any fail. Thresholds are in the script's module-level constants.

| Check | Signature |
|-------|-----------|
| **missing** | `log.csv` or `config.json` not present. Exits immediately. |
| **truncated** | `log.csv` has fewer than 5 rows — SD write interrupted. Exits immediately. |
| **start-angle** | First encoder reading outside 58° ± 10°. Reset-position step was missed. Exits immediately. |
| **power-cut** | Lever never reached setpoint + flat encoder (std < 3°) + active motor differential. Exits immediately. |
| **loop-meltdown** | Encoder range > 90° AND IMU trail > 40%. Rate loop in positive feedback. Exits immediately. |
| **sample-rate** | dt_p99 > 5× median dt. Loop jitter severe enough to degrade KPI accuracy. Exits immediately. |

If smoke exits 1, **stop**. Do not run Steps 2–4. Report the check name and detail to the user. A `postmortem` skill is planned for investigating these failures (not yet available).

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

## Step 3 — Score

```
python .claude/skills/analyse-flight/scripts/score_flight.py test_runs/flights/<flight_id>
```

Computes the transient KPIs and prints them to stdout. KPI definitions live in the script's docstring. The headline number is **HoldMAE_s** — encoder MAE from settling time onward, excluding the overshoot phase. The transient quartet (T→SP, rise time 10-90%, overshoot %, settling time T_s) characterises the approach; HoldMAE_s characterises the hold.

If `Reached = NO`, skip steps 4 and the hold sections of the report. State the fact plainly: "Did not reach setpoint." The plot and KPI table show what happened — no further interpretation needed in Observations.

## Step 4 — Deep-dive stats

```
python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs/flights/<flight_id>
```

Prints grouped diagnostics across six sections: sample rate, sensor health (IMU vs ENC, whole-run), hold-window tracking (bias/std/P95/Pearson r/FFT frequency), control effort (throttle mean/RMS/saturation/dM·dt), inner-loop tracking RMS, and windup events. Group meanings are in the script's docstring.

Two things to watch for in the output:

- The "Whole-run ENC MAE includes the rise" note in the hold-window section. It is shown for reference only — use HoldMAE_s (post-settle) as the headline hold-accuracy number.
- Windup event counts are computed against the `iterm_limit` values from `config.json`. They cover the whole run, not just the hold window.

## Writing the report

1. Read the template at `.claude/skills/analyse-flight/templates/flight_analysis.md`.
2. Fill each placeholder from `score_flight.py` stdout, the `profile_flight.py` stdout, and `config.json`. The profile output echoes canonical KPIs at the top — use that for the KPI Scorecard section. Placeholder names are unique — if the same value appears twice, fill both. The template covers decision-quality headline numbers; the Raw Output section is the complete archive. Not every profile metric appears in a template table — that is intentional.
3. Save the completed report to `test_runs/flights/<flight_id>/analysis.md`, overwriting any existing file.
4. Tell the user where the file was saved.

The Raw Output section must contain script stdout verbatim — that's the ground-truth record. Don't paraphrase, don't reformat tables, don't drop columns. Future-you will rerun a script and compare against this; tampering breaks that.

The Observations section is the part that earns its keep. Aim for a short paragraph (3–6 sentences) that says what *this specific run* is doing. Useful patterns:

- Tie a KPI to a profile stat ("HoldMAE_s of 1.2° with oscillation frequency 0.8 Hz suggests a low-frequency wobble rather than noise").
- Note when numbers disagree with each other ("whole-run MAE is high but HoldMAE_s is low — long rise or significant overshoot").
- Flag sensor or control signals ("M1/M2 asymmetric by ~50 throttle units during hold — angle I-term is compensating for a steady disturbance").
- Call out anything unexpected: gaps in the timeline, a spike, an unexpected mode change.
- Apply the same lens to absences: no I-term buildup despite a persistent offset, symmetric motors when asymmetry is expected — missing signals are as diagnostic as unexpected ones.

Avoid: tuning recommendations ("try lowering ki"), comparisons to other runs (out of scope for this skill), restating numbers that are already in the tables above, and hypothesising causes that involve mechanical state, battery/power-supply condition, or IMU tare quality — the user will always explicitly flag those if they apply. Describe what the telemetry shows; do not append an unconfirmed cause.