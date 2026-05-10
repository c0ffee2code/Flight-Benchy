# Flight Analysis: {FLIGHT_ID}

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | {FLIGHT_ID} |
| Duration | {DURATION_S} s |
| Samples | {N_SAMPLES} |
| Start angle | {START_ANGLE}° |
| Standard start | {STANDARD_START_YN} |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp={ANGLE_KP}, ki={ANGLE_KI}, kd={ANGLE_KD}, iterm_limit={ANGLE_ITERM_LIMIT} |
| rate_pid | kp={RATE_KP}, ki={RATE_KI}, kd={RATE_KD}, iterm_limit={RATE_ITERM_LIMIT} |
| motor | base={BASE_THROTTLE}, min={THROTTLE_MIN}, max={THROTTLE_MAX} |
| feedforward_lead_ms | {FF_LEAD_MS} |
| angle_report | {ANGLE_REPORT} @ {ANGLE_REPORT_HZ} Hz |
| rate_report | {RATE_REPORT} @ {RATE_REPORT_HZ} Hz |

---

## Raw Tool Output

### score_flight.py

```
{SCORE_OUTPUT_VERBATIM}
```

### profile_flight.py

```
{PROFILE_OUTPUT_VERBATIM}
```

### Plots

- `test_runs/flights/{FLIGHT_ID}/01_timeseries.png` — full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/{FLIGHT_ID}/02_step_response.png` — full run milestones + transient zoom
- `test_runs/flights/{FLIGHT_ID}/03_spectrum.png` — PSD of hold-window error (omitted if no settled hold)
- `test_runs/flights/{FLIGHT_ID}/04_hold_error_distribution.png` — hold-error histogram (omitted if no settled hold)
- `test_runs/flights/{FLIGHT_ID}/05_phase_portrait.png` — phase portrait, time-coloured trajectory

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | {REACHED_YN} |
| T→SP (s) | {T_TO_SP_S} |
| Rise time 10-90% (s) | {RISE_TIME_S} |
| Overshoot (% of step) | {OVERSHOOT_PCT} |
| Settling time T_s (s) | {SETTLING_TIME_S} |
| HoldMAE_s (°), post-settle | {HOLD_MAE_S_DEG} |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | {ACTUAL_HZ} |
| Mean dt (ms) | {DT_MEAN_MS} |
| Median dt (ms) | {DT_MEDIAN_MS} |
| dt_p99 (ms) | {DT_P99_MS} |
| dt_max (ms) | {DT_MAX_MS} |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | {IMU_ENC_MAE} |
| MAE fast motion (°) | {IMU_ENC_MAE_FAST} |
| MAE slow motion (°) | {IMU_ENC_MAE_SLOW} |
| Bias IMU-ENC (°) | {IMU_ENC_BIAS} |
| IMU trails motion (%) | {TRAIL_PCT} |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | {HOLD_BIAS} |
| Hold std (°) | {HOLD_STD} |
| Hold P95 \|error\| (°) | {HOLD_P95} |
| Hold max \|error\| (°) | {HOLD_MAX_AE} |
| Pearson r (hold window) | {HOLD_PEARSON_R} |
| FFT dominant freq (Hz) | {FFT_FREQ_HZ} |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | {MEAN_THROTTLE} |
| Saturation upper % (>= throttle_max) | {SATURATION_UPPER_PCT} |
| Saturation lower % (<= throttle_min) | {SATURATION_LOWER_PCT} |
| RMS dM1/dt (throttle/s) | {RMS_DM1_DT} |
| RMS dM2/dt (throttle/s) | {RMS_DM2_DT} |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | {RATE_TRACKING_RMS} |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | {ANG_WINDUP_EVENTS} |
| Rate windup events | {RATE_WINDUP_EVENTS} |

---

## Observations

{One bullet per diagnostic area. Only include areas that have something worth saying —
omit a bullet if there is nothing to add beyond what the tables already show.

- **Hold quality** — characterise the hold using bias + std rather than a single MAE.
  A large signed bias means steady-state offset (insufficient integral or steady
  disturbance). A large std relative to bias means oscillation-dominated error. Note
  FFT dominant freq is shown only when SNR >= 3x noise floor — '-' means the hold is
  too flat to resolve a dominant frequency, which is a good sign.

- **Transient response** — tie rise time and overshoot together. High overshoot with
  short rise time is aggressive gain; high overshoot with long rise time suggests the
  rate loop is saturating or the feedforward is mismatched.

- **Control effort** — flag saturation_upper_pct if non-zero (hitting throttle_max — no
  upward headroom). saturation_lower_pct at throttle_min is normal in a differential hold;
  flag only if significantly asymmetric versus saturation_upper_pct. Compare RMS dM1/dt
  vs dM2/dt — asymmetric chattering is diagnostic. Note M1/M2 mean asymmetry if mean
  throttle is far from base (angle I-term compensating a steady disturbance).

- **Inner loop** — rate tracking RMS above ~5 °/s during a stable hold suggests the rate
  loop is not keeping up; pair with overshoot and settling time to distinguish a tuning
  issue from sensor noise.

- **Sensor health** — IMU-ENC bias, trail %, Pearson r (hold window). Call out anything
  that deviates from "small, consistent, well-correlated". Note if hold-window Pearson r
  differs significantly from expectations.

- **Timeline artefacts** — gaps, spikes, sudden mode changes, anything that does not fit
  the categories above.

Rules: tie every number to an arithmetic explanation or another stat. Apply the absence
lens — missing signals are as diagnostic as unexpected ones. No tuning advice; no
comparisons to other runs; no speculation about mechanical state, battery, or IMU tare.}