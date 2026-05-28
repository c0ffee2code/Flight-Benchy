# Flight Summary: 2026-05-26_21-56-42

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-26_21-56-42 |
| Duration | 119.9 s |
| Samples | 2289 |
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
| Reached setpoint | NO | - |
| T->SP (s) | - | - |
| Rise time 10-90% (s) | - | - |
| Overshoot (% of step) | - | - |
| Damping ratio zeta | - | - |
| Settling time T_s (s) | - | - |
| Hold duration (s) | - | - |
| HoldMAE_s (deg), post-settle | - | - |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.4 |
| Median dt (ms) | 51.0 |
| dt_p99 (ms) | 70.0 |
| dt_max (ms) | 83.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (deg) | 4.53 |
| MAE fast motion (deg) | 4.56 |
| MAE slow motion (deg) | 4.50 |
| Bias IMU-ENC (deg) | -4.53 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (deg, signed) | +28.49 |
| Hold std (deg) | 2.04 |
| Hold P95 error (deg) | 32.02 |
| Hold max error (deg) | 36.94 |
| Pearson r (hold window) | 0.9981 |
| FFT dominant freq (Hz) | 0.008 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 46.0 |
| RMS dM2/dt (throttle/s) | 46.0 |
| ANG_I mean (hold, deg/s) | 0.25 |
| M2-M1 mean (hold, throttle) | +71.1 |
| I-term sign vs dM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (deg/s) | 72.34 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

### Plots

- `test_runs/flights/2026-05-26_21-56-42/analysis/01_timeseries.png` -- full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-05-26_21-56-42/analysis/02_step_response.png` -- full run milestones + transient zoom
- `test_runs/flights/2026-05-26_21-56-42/analysis/03_spectrum.png` -- PSD of hold-window error (omitted if no settled hold)
- `test_runs/flights/2026-05-26_21-56-42/analysis/04_hold_error_distribution.png` -- hold-error histogram (omitted if no settled hold)
- `test_runs/flights/2026-05-26_21-56-42/analysis/05_phase_portrait.png` -- phase portrait, time-coloured trajectory
