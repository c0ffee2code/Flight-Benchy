# Flight Analysis: 2026-05-03_17-13-32

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-03_17-13-32 |
| Duration | 119.9 s |
| Samples | 2276 |
| Start angle | 51.9° |
| Standard start | Yes |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
| motor | base=600.0, min=100.0, max=800.0 |
| feedforward_lead_ms | 15.0 |
| angle_report | game_rotation_vector @ 50 Hz |
| rate_report | calibrated_gyroscope @ 200 Hz |

---

## Raw Tool Output

### score_flight.py

```
Run                           Start  OK  Reached  T->SP (s)  HoldMAE_s (deg)  Dur (s)
-------------------------------------------------------------------------------------
2026-05-03_17-13-32        51.9deg  ok      YES       1.3s          3.89deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 1.3s      Overshoot: 19.6%     T_s (settling): 101.7s
  Damping ratio zeta: 0.460

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-03_17-13-32
```

### profile_flight.py

```
Loaded 2026-05-03_17-13-32: 2276 samples, 119.9s

==========================================================
  2026-05-03_17-13-32
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600.0, min=100.0, max=800.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                    1.3s
  Rise time 10-90% (s)                         1.3s
  Overshoot (% of step)                       19.6%
  Damping ratio zeta                          0.460
  Settling time T_s (s)                      101.7s
  HoldMAE_s (deg), post-settle              3.89deg

  --- Sample Rate ---
  Samples                                      2276
  Duration (s)                                119.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.7
  Median dt (ms)                               51.0
  dt_p99 (ms)                                  71.3
  dt_max (ms)                                  84.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                3.30
  MAE (fast motion)                            3.49
  MAE (slow motion)                            3.05
  Max AE                                       9.32
  RMS error                                    3.84
  Bias (IMU-ENC)                              -3.29
  IMU trails motion (%)                        39.5
  Encoder range (deg)                          80.1
  IMU range (deg)                              81.5

  --- Hold-Window Tracking (ENC vs -15deg, post-reach) ---
  Whole-run ENC MAE (deg)                      5.63
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                     -1.47
  Hold std (deg)                               6.06
  Hold P95 |error| (deg)                      11.12
  Hold max |error| (deg)                      13.68
  Pearson r (hold window)                    0.9502
  FFT dominant freq (Hz)                      0.093
    (freq resolution Hz)                      0.008

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   600.0
  RMS throttle                                600.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                     107.4
  RMS dM2/dt (throttle/s)                     107.4
  ANG_I mean (hold, deg/s)                    -4.29
  M2-M1 mean (hold, throttle)                 -18.2
  I-term sign vs dM                              OK

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   27.13

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-03_17-13-32/01_timeseries.png` — full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-05-03_17-13-32/02_step_response.png` — full run milestones + transient zoom
- `test_runs/flights/2026-05-03_17-13-32/03_spectrum.png` — PSD of hold-window error
- `test_runs/flights/2026-05-03_17-13-32/04_hold_error_distribution.png` — hold-error histogram
- `test_runs/flights/2026-05-03_17-13-32/05_phase_portrait.png` — phase portrait, time-coloured trajectory

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | Yes |
| T→SP (s) | 1.3 |
| Rise time 10-90% (s) | 1.3 |
| Overshoot (% of step) | 19.6% |
| Damping ratio ζ | 0.460 |
| Settling time T_s (s) | 101.7 |
| HoldMAE_s (°), post-settle | 3.89 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.0 |
| Mean dt (ms) | 52.7 |
| Median dt (ms) | 51.0 |
| dt_p99 (ms) | 71.3 |
| dt_max (ms) | 84.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 3.30 |
| MAE fast motion (°) | 3.49 |
| MAE slow motion (°) | 3.05 |
| Bias IMU-ENC (°) | -3.29 |
| IMU trails motion (%) | 39.5 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | -1.47 |
| Hold std (°) | 6.06 |
| Hold P95 \|error\| (°) | 11.12 |
| Hold max \|error\| (°) | 13.68 |
| Pearson r (hold window) | 0.9502 |
| FFT dominant freq (Hz) | 0.093 |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 600.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 107.4 |
| RMS dM2/dt (throttle/s) | 107.4 |
| ANG_I mean (hold, deg/s) | -4.29 |
| M2−M1 mean (hold, throttle) | -18.2 |
| I-term sign vs ΔM | OK |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 27.13 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Hold quality**: Setpoint for this run is −15° (not standard 0°). Post-reach hold std of 6.06° with bias of −1.47° is oscillation-dominated — variance far exceeds mean offset. The FFT dominant frequency of 0.093 Hz (~10.7 s period) marks a slow, persistent wobble that continued from first reach at 1.3 s until confirmed settling at 101.7 s, consuming 84% of the run. HoldMAE_s of 3.89° covers only the final 18.2 s (119.9 − 101.7); its improvement over the 6.06° post-reach std confirms the oscillation eventually damped within the run rather than being cut off at run end.

- **Transient**: T→SP and rise time are both 1.3 s — the lever crossed the ±10° band immediately on the first approach. Overshoot of 19.6% (ζ = 0.460) is moderate; the issue is not the approach itself but the subsequent slow ring at 0.093 Hz that took another 100 s to resolve.

- **Inner loop**: Rate tracking RMS of 27.13 °/s during hold is high. The rate PID has ki = 0, so the inner loop has no integral action to correct persistent rate error. The 27.13 °/s RMS is consistent with the ongoing 0.093 Hz oscillation driving large, continuously changing rate setpoints that the proportional-only inner loop cannot fully track.

- **Control effort**: Mean throttle of 600.0 equals base_throttle exactly with RMS = mean — no net throttle bias. No saturation on either boundary. RMS dM1/dt = RMS dM2/dt = 107.4 confirms symmetric motor activity throughout. ANG_I mean = −4.29 °/s with M2−M1 mean = −18.2 (sign OK) — the integral is maintaining a differential appropriate for holding below 0°, with no windup events.

- **Sensor health**: IMU−ENC bias of −3.29° is nearly equal to the MAE of 3.30°, indicating an almost purely systematic offset with negligible additional scatter. Trail% of 39.5% is near the 40% meltdown threshold but did not trigger it. Hold-window Pearson r = 0.950 shows the two sensors track relative motion well despite the fixed offset.
