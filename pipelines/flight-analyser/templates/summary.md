# Flight Summary: $flight_id

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | $flight_id |
| Duration | $duration_s |
| Samples | $n_samples |
| Start angle | $start_angle |
| Standard start | $standard_start |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | $angle_pid_row |
| rate_pid | $rate_pid_row |
| motor | $motor_row |
| feedforward_lead_ms | $ff_lead_ms |
| angle_report | $angle_report |
| rate_report | $rate_report |

---

## KPI Scorecard

| Metric | Value | Level |
|--------|-------|-------|
| Reached setpoint | $reached | - |
| T->SP (s) | $t_to_sp | $t_to_sp_level |
| Rise time 10-90% (s) | $rise_time | - |
| Overshoot (% of step) | $overshoot | $overshoot_level |
| Damping ratio zeta | $damping_ratio | - |
| Settling time T_s (s) | $settling_time | $settling_time_level |
| Hold duration (s) | $hold_duration | $hold_duration_level |
| HoldMAE_s (deg), post-settle | $hold_mae | $hold_mae_level |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | $actual_hz |
| Mean dt (ms) | $dt_mean_ms |
| Median dt (ms) | $dt_median_ms |
| dt_p99 (ms) | $dt_p99_ms |
| dt_max (ms) | $dt_max_ms |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (deg) | $sh_mae |
| MAE fast motion (deg) | $sh_mae_fast |
| MAE slow motion (deg) | $sh_mae_slow |
| Bias IMU-ENC (deg) | $sh_bias |
| IMU trails motion (%) | $sh_trail_pct |

## Hold Tracking (from T_s, confirmed hold)

| Metric | Value |
|--------|-------|
| Hold bias (deg, signed) | $ht_bias |
| Hold std (deg) | $ht_std |
| Hold P95 error (deg) | $ht_p95 |
| Hold max error (deg) | $ht_max_ae |
| FFT dominant freq (Hz) | $ht_fft_freq |

## Approach Tracking (T->SP to T_s)

| Metric | Value |
|--------|-------|
| Approach bias (deg, signed) | $at_bias |
| Approach std (deg) | $at_std |
| Approach P95 error (deg) | $at_p95 |
| Approach max error (deg) | $at_max_ae |
| Pearson r (approach) | $at_pearson_r |
| Approach duration (s) | $at_duration |

## Control Effort (from T_s, confirmed hold)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | $ce_mean_throttle |
| Saturation upper % (>= throttle_max) | $ce_sat_upper |
| Saturation lower % (<= throttle_min) | $ce_sat_lower |
| RMS dM1/dt (throttle/s) | $ce_rms_dm1 |
| RMS dM2/dt (throttle/s) | $ce_rms_dm2 |
| ANG_I mean (hold, deg/s) | $ce_ang_i_mean |
| M2-M1 mean (hold, throttle) | $ce_m2_m1_mean |
| I-term sign vs dM | $ce_iterm_sign |

## Inner Loop (from T_s, confirmed hold)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (deg/s) | $il_rate_rms |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | $wu_ang_events |
| Rate windup events | $wu_rate_events |

### Plots

- `test_runs/flights/$flight_id/analysis/01_timeseries.png` -- full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/$flight_id/analysis/02_step_response.png` -- full run milestones + transient zoom
- `test_runs/flights/$flight_id/analysis/03_spectrum.png` -- PSD of hold-window error (omitted if no settled hold)
- `test_runs/flights/$flight_id/analysis/04_hold_error_distribution.png` -- hold-error histogram (omitted if no settled hold)
- `test_runs/flights/$flight_id/analysis/05_phase_portrait.png` -- phase portrait, time-coloured trajectory
