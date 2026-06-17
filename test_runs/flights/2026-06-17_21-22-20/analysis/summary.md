# Flight Summary: 2026-06-17_21-22-20

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-06-17_21-22-20 |
| Duration | 119.9 s |
| Samples | 1543 |
| Start angle | 50.6 deg |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.5, ki=0.1, kd=0.65, iterm_limit=5.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
| motor | base=600.0, min=90.0, max=900.0 |
| feedforward_lead_ms | 12 |
| angle_report | game_rotation_vector @ 100 Hz |
| rate_report | calibrated_gyroscope @ 300 Hz |

---

## KPI Scorecard

| Metric | Value | Level |
|--------|-------|-------|
| Reached setpoint | YES | - |
| T->SP (s) | 1.8 | excellent |
| Rise time 10-90% (s) | 4.3 | - |
| Overshoot (% of step) | 17.0% | good |
| Damping ratio zeta | 0.491 | - |
| Settling time T_s (s) | 1.8 | excellent |
| Hold duration (s) | 118.1 | excellent |
| HoldMAE_s (deg), post-settle | 2.60 | good |

## Sample Rate

| Metric | Value |
|--------|-------|
| Nominal Hz | 15.0 |
| Achieved Hz | 12.9 |
| Mean dt (ms) | 77.8 |
| Median dt (ms) | 76.0 |
| dt_p99 (ms) | 107.0 |
| dt_max (ms) | 122.0 |

## Cycle Timing (inner loop)

| Metric | Value |
|--------|-------|
| Nominal inner period (ms) | 3.33 |
| DT_MS mean / p99 / max (ms) | 3.8 / 6.0 / 30.0 |
| MAX_DT_MS mean / p99 / max (ms) | 6.4 / 29.0 / 30.0 |
| Spikes >2x nominal | 205 (13.3%) |
| Median spike interval (ms) | 768 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (deg) | 0.75 |
| MAE fast motion (deg) | 0.77 |
| MAE slow motion (deg) | 0.75 |
| Bias IMU-ENC (deg) | 0.74 |

## Hold Tracking (from T_s, confirmed hold)

| Metric | Value |
|--------|-------|
| Hold bias (deg, signed) | -1.06 |
| Hold std (deg) | 3.11 |
| Hold P95 error (deg) | 6.77 |
| Hold max error (deg) | 9.67 |
| FFT dominant freq (Hz) | 0.068 |

## Approach Tracking (T->SP to T_s)

| Metric | Value |
|--------|-------|
| Approach bias (deg, signed) | - |
| Approach std (deg) | - |
| Approach P95 error (deg) | - |
| Approach max error (deg) | - |
| Pearson r (approach) | - |
| Approach duration (s) | - |

## Control Effort (from T_s, confirmed hold)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 600.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 148.1 |
| RMS dM2/dt (throttle/s) | 148.1 |
| ANG_I mean (hold, deg/s) | -0.11 |
| M2-M1 mean (hold, throttle) | -0.8 |
| I-term sign vs dM | N/A (P-term dominant) |

## Inner Loop (from T_s, confirmed hold)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (deg/s) | 14.58 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

### Plots

- `test_runs/flights/2026-06-17_21-22-20/analysis/01_timeseries.png` -- full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-06-17_21-22-20/analysis/02_step_response.png` -- full run milestones + transient zoom
- `test_runs/flights/2026-06-17_21-22-20/analysis/03_spectrum.png` -- PSD of hold-window error (omitted if no settled hold)
- `test_runs/flights/2026-06-17_21-22-20/analysis/04_hold_error_distribution.png` -- hold-error histogram (omitted if no settled hold)
- `test_runs/flights/2026-06-17_21-22-20/analysis/05_phase_portrait.png` -- phase portrait, time-coloured trajectory
- `test_runs/flights/2026-06-17_21-22-20/analysis/06_cycle_timing.png` -- cycle timing (MAX_DT_MS time series + DT_MS distribution)
