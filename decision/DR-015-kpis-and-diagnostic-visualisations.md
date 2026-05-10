# DR-015: KPIs and Diagnostic Visualisations — Analysis Framework

**Status:** Implemented
**Date:** 2026-05-10
**Context:** Establish the standard metrics and visual analysis suite for evaluating single-axis stabilisation runs, so results are directly comparable across tuning iterations.

## Context

Each stabilisation run produces a timestamped folder with `log.csv` (22-column telemetry at ~20 Hz) and `config.json` (full parameter snapshot). Without a defined measurement framework, comparing "this run felt better" across sessions is unreliable — PID gains change, start conditions vary, and subjective impression is dominated by the transient rather than the hold.

Three requirements shaped the framework:

1. **Repeatability** — the same run, analysed twice, must produce identical numbers.
2. **Comparability** — a number from run A must mean the same thing as the same number from run B, regardless of when or with what gains it was collected.
3. **Decomposability** — transient behaviour and hold behaviour must be measured separately, because a fast approach with poor hold and a slow approach with excellent hold are both "partially good" and need distinct handling.

## Decision

### Standard test convention

Every comparable run starts with M1-end resting on the restrictor at approximately **+58°** (encoder reading). The algorithm lifts the lever to within ±10° of the configured setpoint and holds. Runs that do not start within ±10° of +58° are **non-standard** — their KPIs are recorded for the paper trail but must not inform tuning decisions, because the initial error and hence the control authority required is different.

The **±10° acceptance band** is the operational definition of "close enough to setpoint". It is wide enough to be achievable under reasonable gains and narrow enough to be a meaningful hold target at this bench scale.

### Sensor used for KPIs

All KPIs are computed from the **AS5600 magnetic encoder**, not the IMU. The encoder is the ground truth reference: ~0.09° resolution, no fusion filter, no drift. IMU angle is what the controller sees; encoder angle is what is actually happening. Using different sensors for different runs would conflate control accuracy with sensor accuracy.

### KPI definitions

All times are measured from the start of the run (first telemetry row).

| KPI | Definition | What it measures |
|-----|-----------|-----------------|
| **T→SP** | Seconds from run start to first entry into ±10° band around setpoint | Total time-to-reach, including pre-spin ramp |
| **Rise time (10→90%)** | Time for the encoder angle to travel from 10% to 90% of the initial step | Rate of approach in the linear region, excluding startup and deceleration |
| **Overshoot (% of step)** | Maximum excursion past setpoint as % of initial step, after first setpoint crossing | How aggressively the controller overshoots |
| **T_s (settling time)** | First moment encoder enters ±10° band and stays inside for ≥5s continuously through end of run | When the transient is definitively over |
| **HoldMAE_s** | Encoder MAE from T_s onward | Pure hold quality, no transient contamination |

**Why 5s for settling confirmation:** a momentary crossing of the band followed by departure is not settling. Five seconds at ~20 Hz gives ~100 samples — enough to distinguish a genuine hold from a transient excursion into the band.

**Why HoldMAE_s not whole-run MAE:** the approach phase dominates whole-run MAE on short runs and obscures hold quality. HoldMAE_s is the number that tracks tuning progress for the hold behaviour.

### Analysis pipeline

```
smoke.py          → validates run folder; exits 1 on any structural failure
plots.py          → all diagnostic figures (see below)
score_flight.py   → KPI pass/fail table; compute_kpis() importable by profile_flight.py
profile_flight.py → canonical KPIs + grouped diagnostics (sample rate, sensor health,
                    hold tracking, control effort, inner loop, windup)
```

`smoke.py` runs first and gates the rest. It rejects runs with: missing files, fewer than 5 rows, non-standard start angle, power-cut signature, loop meltdown, or severe loop jitter. A run that passes smoke is structurally valid for KPI analysis.

### Diagnostic figures and their lenses

Each figure answers a different question. They are numbered by diagnostic priority.

**01_timeseries** — chronological black-box view across the full run. Five subplots: angle tracking (ENC vs IMU with setpoint band), rate tracking (desired vs measured), outer PID terms (P/I/D contributions), inner PID terms, motor throttle outputs (M1/M2 with saturation limits). This is always the first figure to open. It catches: control loop instability, IMU-encoder divergence, motor saturation, I-term windup, sudden mode changes, and timeline gaps.

**02_step_response** — canonical control-theory view. Two panels: full run with T→SP, T_s, and overshoot peak marked; transient zoom (first ~6–15s) with rise time (10→90%) dots and the T→SP reference line. Answers: how fast did it get there, how badly did it overshoot, when did it settle. The full-run panel makes T_s visible in context of the hold tail.

**03_spectrum** — power spectral density of the hold-window error (Welch method, numpy-only). Reveals the dominant oscillation frequency during the hold. The frequency band maps directly to PID terms: low-frequency oscillation (< 0.1 Hz) points to I-term, mid-range (0.1–1 Hz) points to P-term, high-frequency points to D-term or sensor noise. Skipped if no confirmed settled hold.

**04_hold_error_distribution** — density histogram of encoder error during the settled hold window, with Gaussian overlay and bias/std/P95 annotations. Answers: is the error centred (bias near zero) or systematically offset (I-term not fully compensating)? Is the shape Gaussian (noise-dominated) or bimodal/skewed (oscillation-dominated or asymmetric disturbance)? Skipped if no confirmed settled hold.

**05_phase_portrait** — encoder angle (x) vs angular rate / gyro_x (y), coloured by time using a plasma colormap. Shows the state-space trajectory of the system. Limit cycles appear as closed loops; settling appears as a spiral converging toward the target state (setpoint, 0 °/s). The time colouring distinguishes approach trajectory from hold behaviour. Rate axis uses gyro_x directly (what the inner loop sees) rather than numerically differentiating the encoder.

## Consequences

- All tuning decisions reference HoldMAE_s as the headline hold metric, not whole-run MAE.
- Non-standard starts are retained in the folder and get a full paper trail, but are explicitly excluded from tuning comparisons.
- The analysis pipeline is deterministic and version-controlled alongside the flight control source — re-running it on any archived run produces the same numbers.
- Adding a new KPI requires updating `score_flight.py` or `profile_flight.py` and this record.
- Adding a new figure requires updating `plots.py`, this record, and the `flight_analysis.md` template.