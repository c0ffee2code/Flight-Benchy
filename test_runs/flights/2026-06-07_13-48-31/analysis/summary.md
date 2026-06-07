# Flight Summary: 2026-06-07_13-48-31

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-06-07_13-48-31 |
| Duration | 119.9 s |
| Samples | 1545 |
| Start angle | 49.8 deg |
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
| T->SP (s) | 1.4 | excellent |
| Rise time 10-90% (s) | 2.0 | - |
| Overshoot (% of step) | 13.1% | good |
| Damping ratio zeta | 0.544 | - |
| Settling time T_s (s) | 1.4 | excellent |
| Hold duration (s) | 118.5 | excellent |
| HoldMAE_s (deg), post-settle | 2.62 | good |

## Sample Rate

| Metric | Value |
|--------|-------|
| Nominal Hz | 15.0 |
| Achieved Hz | 12.9 |
| Mean dt (ms) | 77.6 |
| Median dt (ms) | 76.0 |
| dt_p99 (ms) | 106.6 |
| dt_max (ms) | 118.0 |

## Cycle Timing (inner loop)

| Metric | Value |
|--------|-------|
| Nominal inner period (ms) | 3.33 |
| DT_MS mean / p99 / max (ms) | 3.8 / 5.0 / 18.0 |
| MAX_DT_MS mean / p99 / max (ms) | 6.4 / 29.0 / 30.0 |
| Spikes >2x nominal | 192 (12.4%) |
| Median spike interval (ms) | 771 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (deg) | 0.73 |
| MAE fast motion (deg) | 0.72 |
| MAE slow motion (deg) | 0.72 |
| Bias IMU-ENC (deg) | 0.72 |

## Hold Tracking (from T_s, confirmed hold)

| Metric | Value |
|--------|-------|
| Hold bias (deg, signed) | -0.67 |
| Hold std (deg) | 3.15 |
| Hold P95 error (deg) | 6.06 |
| Hold max error (deg) | 9.58 |
| FFT dominant freq (Hz) | 0.008 (advisory) |

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
| RMS dM1/dt (throttle/s) | 154.1 |
| RMS dM2/dt (throttle/s) | 154.1 |
| ANG_I mean (hold, deg/s) | -0.19 |
| M2-M1 mean (hold, throttle) | -0.5 |
| I-term sign vs dM | N/A (P-term dominant) |

## Inner Loop (from T_s, confirmed hold)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (deg/s) | 15.49 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

### Plots

- `test_runs/flights/2026-06-07_13-48-31/analysis/01_timeseries.png` -- full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-06-07_13-48-31/analysis/02_step_response.png` -- full run milestones + transient zoom
- `test_runs/flights/2026-06-07_13-48-31/analysis/03_spectrum.png` -- PSD of hold-window error (omitted if no settled hold)
- `test_runs/flights/2026-06-07_13-48-31/analysis/04_hold_error_distribution.png` -- hold-error histogram (omitted if no settled hold)
- `test_runs/flights/2026-06-07_13-48-31/analysis/05_phase_portrait.png` -- phase portrait, time-coloured trajectory
- `test_runs/flights/2026-06-07_13-48-31/analysis/06_cycle_timing.png` -- cycle timing (MAX_DT_MS time series + DT_MS distribution)
