# Flight Analysis: 2026-05-14_14-45-10

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_14-45-10 |
| Duration | 119.9 s |
| Samples | 2279 |
| Start angle | 52.0° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.5, iterm_limit=100.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
| motor | base=500.0, min=100.0, max=900.0, expo=0.3 |
| feedforward_lead_ms | 15.0 |
| angle_report | game_rotation_vector @ 50 Hz |
| rate_report | calibrated_gyroscope @ 200 Hz |

---

## Raw Tool Output

### score_flight.py

```
Run                           Start  OK  Reached  T->SP (s)  HoldMAE_s (deg)  Dur (s)
-------------------------------------------------------------------------------------
2026-05-14_14-45-10        52.0deg  ok      YES      15.5s          3.32deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 16.3s     Overshoot: 24.5%     T_s (settling): 79.0s
  Damping ratio zeta: 0.409

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_14-45-10
```

### profile_flight.py

```
Loaded 2026-05-14_14-45-10: 2279 samples, 119.9s

==========================================================
  2026-05-14_14-45-10
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.5, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   15.5s
  Rise time 10-90% (s)                        16.3s
  Overshoot (% of step)                       24.5%
  Damping ratio zeta                          0.409
  Settling time T_s (s)                       79.0s
  HoldMAE_s (deg), post-settle              3.32deg

  --- Sample Rate ---
  Samples                                      2279
  Duration (s)                                119.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.6
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  71.0
  dt_max (ms)                                  89.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                0.87
  MAE (fast motion)                            0.96
  MAE (slow motion)                            0.83
  Max AE                                       3.41
  RMS error                                    1.00
  Bias (IMU-ENC)                              -0.87
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          63.4
  IMU range (deg)                              58.7

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      5.32
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                      0.30
  Hold std (deg)                               4.00
  Hold P95 |error| (deg)                       7.76
  Hold max |error| (deg)                      13.83
  Pearson r (hold window)                    0.9998
  FFT dominant freq (Hz)                      0.025
    (freq resolution Hz)                      0.010
  (advisory - peak within 3x resolution; may be drift rather than oscillation)

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      39.4
  RMS dM2/dt (throttle/s)                      39.4
  ANG_I mean (hold, deg/s)                     2.45
  M2-M1 mean (hold, throttle)                   1.1
  I-term sign vs dM                              OK

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   12.97

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_14-45-10/01_timeseries.png`
- `test_runs/flights/2026-05-14_14-45-10/02_step_response.png`
- `test_runs/flights/2026-05-14_14-45-10/03_spectrum.png`
- `test_runs/flights/2026-05-14_14-45-10/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_14-45-10/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 15.5 |
| Rise time 10-90% (s) | 16.3 |
| Overshoot (% of step) | 24.5% |
| Damping ratio ζ | 0.409 |
| Settling time T_s (s) | 79.0 |
| HoldMAE_s (°), post-settle | 3.32 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.0 |
| Mean dt (ms) | 52.6 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 71.0 |
| dt_max (ms) | 89.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 0.87 |
| MAE fast motion (°) | 0.96 |
| MAE slow motion (°) | 0.83 |
| Bias IMU-ENC (°) | −0.87 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | +0.30 |
| Hold std (°) | 4.00 |
| Hold P95 \|error\| (°) | 7.76 |
| Hold max \|error\| (°) | 13.83 |
| Pearson r (hold window) | 0.9998 |
| FFT dominant freq (Hz) | 0.025 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 39.4 |
| RMS dM2/dt (throttle/s) | 39.4 |
| ANG_I mean (hold, deg/s) | +2.45 |
| M2−M1 mean (hold, throttle) | +1.1 |
| I-term sign vs ΔM | OK |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 12.97 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **expo=0.3 increased overshoot**: 24.5% is the highest overshoot of any run in this session, up from 14.4–19.8% at expo=0.0. The slow-settle mode reappeared (T_s=79s). expo reduces near-setpoint motor authority, which removes braking deceleration as the lever approaches setpoint — the lever arrives at setpoint with more remaining velocity, not less.

- **T_s slightly shorter than prior slow runs**: 79.0s vs ~91s at expo=0.0. expo's authority reduction also limits oscillation amplitude during the ring-down phase, so settling completes modestly faster even while the initial overshoot is worse. These are opposing effects and the net result is still a failed run.

- **Hold quality within ring-down window**: HoldMAE_s 3.32° covers the ring-down period (hold start at T→SP=15.5s, settling at 79s). RMS dM/dt of 39.4 is slightly lower than previous runs, consistent with expo reducing peak motor differentials.
