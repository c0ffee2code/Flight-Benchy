# Flight Summary: 2026-05-26_22-26-54

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-26_22-26-54 |
| Duration | 120.0 s |
| Samples | 2248 |
| Start angle | 51.9 deg |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.4, iterm_limit=5.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
| motor | base=500.0, min=100.0, max=900.0 |
| feedforward_lead_ms | 15 |
| angle_report | game_rotation_vector @ 50 Hz |
| rate_report | calibrated_gyroscope @ 200 Hz |

---

## KPI Scorecard

| Metric | Value | Level |
|--------|-------|-------|
| Reached setpoint | YES | - |
| T->SP (s) | 3.9 | good |
| Rise time 10-90% (s) | 4.2 | - |
| Overshoot (% of step) | 28.4% | below_pass |
| Damping ratio zeta | 0.372 | - |
| Settling time T_s (s) | - | - |
| Hold duration (s) | - | - |
| HoldMAE_s (deg), post-settle | - | - |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 18.7 |
| Mean dt (ms) | 53.4 |
| Median dt (ms) | 51.0 |
| dt_p99 (ms) | 97.0 |
| dt_max (ms) | 123.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (deg) | 0.55 |
| MAE fast motion (deg) | 0.67 |
| MAE slow motion (deg) | 0.50 |
| Bias IMU-ENC (deg) | -0.55 |
| IMU trails motion (%) | 8.7 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (deg, signed) | -11.44 |
| Hold std (deg) | 3.59 |
| Hold P95 error (deg) | 16.47 |
| Hold max error (deg) | 19.01 |
| Pearson r (hold window) | 0.9989 |
| FFT dominant freq (Hz) | 0.009 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 53.6 |
| RMS dM2/dt (throttle/s) | 53.6 |
| ANG_I mean (hold, deg/s) | -0.23 |
| M2-M1 mean (hold, throttle) | -35.2 |
| I-term sign vs dM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (deg/s) | 38.01 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

### Plots

- `test_runs/flights/2026-05-26_22-26-54/analysis/01_timeseries.png` -- full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-05-26_22-26-54/analysis/02_step_response.png` -- full run milestones + transient zoom
- `test_runs/flights/2026-05-26_22-26-54/analysis/03_spectrum.png` -- PSD of hold-window error (omitted if no settled hold)
- `test_runs/flights/2026-05-26_22-26-54/analysis/04_hold_error_distribution.png` -- hold-error histogram (omitted if no settled hold)
- `test_runs/flights/2026-05-26_22-26-54/analysis/05_phase_portrait.png` -- phase portrait, time-coloured trajectory
