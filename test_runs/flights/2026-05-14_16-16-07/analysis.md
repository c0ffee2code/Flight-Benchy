# Flight Analysis: 2026-05-14_16-16-07

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_16-16-07 |
| Duration | 119.9 s |
| Samples | 2291 |
| Start angle | 52.0° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.5, iterm_limit=5.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
| motor | base=500.0, min=100.0, max=900.0, expo=0.0 |
| feedforward_lead_ms | 15.0 |
| angle_report | game_rotation_vector @ 50 Hz |
| rate_report | calibrated_gyroscope @ 200 Hz |

---

## Raw Tool Output

### score_flight.py

```
Run                           Start  OK  Reached  T->SP (s)  HoldMAE_s (deg)  Dur (s)
-------------------------------------------------------------------------------------
2026-05-14_16-16-07        52.0deg  ok      YES      23.3s          3.25deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 26.9s     Overshoot: 14.4%     T_s (settling): 23.3s
  Damping ratio zeta: 0.526

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_16-16-07
```

### profile_flight.py

```
Loaded 2026-05-14_16-16-07: 2291 samples, 119.9s

==========================================================
  2026-05-14_16-16-07
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.5, iterm_limit=5.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   23.3s
  Rise time 10-90% (s)                        26.9s
  Overshoot (% of step)                       14.4%
  Damping ratio zeta                          0.526
  Settling time T_s (s)                       23.3s
  HoldMAE_s (deg), post-settle              3.25deg

  --- Sample Rate ---
  Samples                                      2291
  Duration (s)                                119.9
  Achieved Hz                                  19.1
  Mean dt (ms)                                 52.4
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  70.1
  dt_max (ms)                                  86.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                1.14
  MAE (fast motion)                            1.15
  MAE (slow motion)                            1.21
  Max AE                                       4.88
  RMS error                                    1.34
  Bias (IMU-ENC)                              -1.14
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          59.5
  IMU range (deg)                              55.1

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      6.61
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                     -0.15
  Hold std (deg)                               3.82
  Hold P95 |error| (deg)                       6.59
  Hold max |error| (deg)                       9.84
  Pearson r (hold window)                    0.9997
  FFT dominant freq (Hz)                      0.021
    (freq resolution Hz)                      0.010
  (advisory - peak within 3x resolution; may be drift rather than oscillation)

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      43.8
  RMS dM2/dt (throttle/s)                      43.8
  ANG_I mean (hold, deg/s)                    -0.03
  M2-M1 mean (hold, throttle)                  -2.9
  I-term sign vs dM                      N/A (P-term dominant)

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   11.88

  --- Windup (whole-run)---
  Angle windup events                             0
  Angle windup threshold                        2.5
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_16-16-07/01_timeseries.png`
- `test_runs/flights/2026-05-14_16-16-07/02_step_response.png`
- `test_runs/flights/2026-05-14_16-16-07/03_spectrum.png`
- `test_runs/flights/2026-05-14_16-16-07/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_16-16-07/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 23.3 |
| Rise time 10-90% (s) | 26.9 |
| Overshoot (% of step) | 14.4% |
| Damping ratio ζ | 0.526 |
| Settling time T_s (s) | 23.3 |
| HoldMAE_s (°), post-settle | 3.25 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.4 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 70.1 |
| dt_max (ms) | 86.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 1.14 |
| MAE fast motion (°) | 1.15 |
| MAE slow motion (°) | 1.21 |
| Bias IMU-ENC (°) | −1.14 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | −0.15 |
| Hold std (°) | 3.82 |
| Hold P95 \|error\| (°) | 6.59 |
| Hold max \|error\| (°) | 9.84 |
| Pearson r (hold window) | 0.9997 |
| FFT dominant freq (Hz) | 0.021 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 43.8 |
| RMS dM2/dt (throttle/s) | 43.8 |
| ANG_I mean (hold, deg/s) | −0.03 |
| M2−M1 mean (hold, throttle) | −2.9 |
| I-term sign vs ΔM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 11.88 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Overshoot at the fast-settle boundary**: 14.4% overshoot with T_s = T→SP = 23.3s — no ring-down. Damping ratio ζ = 0.526 is consistent with prior fast-settle runs. The tighter iterm_limit (5.0 vs 100.0) reduced approach I-term accumulation; the net effect is a return to the ≤14.4% overshoot zone. iterm_limit=5 caps the crossing I-term residual at ≤5 deg/s vs ≤10 deg/s at the previous limit.

- **Integrator near-zero throughout**: ANG_I mean = −0.03 deg/s during hold — effectively zero with iterm_limit=5. Zero windup events confirm the limit is never reached at this hold equilibrium. The motor asymmetry (M2−M1 = −2.9 throttle) is being corrected by the P-term alone, not the integrator. Hold quality (HoldMAE_s 3.25°, hold std 3.82°) is comparable to prior fast-settle runs at iterm_limit=100, demonstrating that the integrator's equilibrium contribution during hold was negligible.

- **Rate tracking RMS normalised**: 11.88°/s — back in the same range as prior fast-settle runs, down from 19.24°/s at iterm_limit=10. The lower crossing I-term residual at iterm_limit=5 reduces the initial hold oscillation as the integrator drains from +5 to 0, which cuts the commanded rate variance in the early hold window.

- **Timing clean**: dt_p99 = 70.1ms, dt_max = 86.0ms — consistent with the best timing seen across this session. No timing-related KPI degradation.
