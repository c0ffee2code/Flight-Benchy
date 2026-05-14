# Flight Analysis: 2026-05-14_13-55-27

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_13-55-27 |
| Duration | 119.9 s |
| Samples | 2259 |
| Start angle | 51.9° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
| motor | base=500.0, min=100.0, max=900.0 |
| feedforward_lead_ms | 15.0 |
| angle_report | game_rotation_vector @ 50 Hz |
| rate_report | calibrated_gyroscope @ 200 Hz |

---

## Raw Tool Output

### score_flight.py

```
Run                           Start  OK  Reached  T->SP (s)  HoldMAE_s (deg)  Dur (s)
-------------------------------------------------------------------------------------
2026-05-14_13-55-27        51.9deg  ok      YES      31.8s          4.74deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 33.0s     Overshoot: 19.1%     T_s (settling): 91.0s
  Damping ratio zeta: 0.466

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_13-55-27
```

### profile_flight.py

```
Loaded 2026-05-14_13-55-27: 2259 samples, 119.9s

==========================================================
  2026-05-14_13-55-27
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   31.8s
  Rise time 10-90% (s)                        33.0s
  Overshoot (% of step)                       19.1%
  Damping ratio zeta                          0.466
  Settling time T_s (s)                       91.0s
  HoldMAE_s (deg), post-settle              4.74deg

  --- Sample Rate ---
  Samples                                      2259
  Duration (s)                                119.9
  Achieved Hz                                  18.8
  Mean dt (ms)                                 53.1
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  97.0
  dt_max (ms)                                 117.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                1.25
  MAE (fast motion)                            1.22
  MAE (slow motion)                            1.34
  Max AE                                       4.88
  RMS error                                    1.43
  Bias (IMU-ENC)                              -1.25
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          61.9
  IMU range (deg)                              57.4

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      7.68
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                      0.26
  Hold std (deg)                               4.49
  Hold P95 |error| (deg)                       9.05
  Hold max |error| (deg)                      13.10
  Pearson r (hold window)                    0.9997
  FFT dominant freq (Hz)                      0.045

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      41.9
  RMS dM2/dt (throttle/s)                      41.9
  ANG_I mean (hold, deg/s)                     3.59
  M2-M1 mean (hold, throttle)                   1.4
  I-term sign vs dM                              OK

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   13.80

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_13-55-27/01_timeseries.png` — full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-05-14_13-55-27/02_step_response.png` — full run milestones + transient zoom
- `test_runs/flights/2026-05-14_13-55-27/03_spectrum.png` — PSD of hold-window error
- `test_runs/flights/2026-05-14_13-55-27/04_hold_error_distribution.png` — hold-error histogram
- `test_runs/flights/2026-05-14_13-55-27/05_phase_portrait.png` — phase portrait, time-coloured trajectory

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 31.8 |
| Rise time 10-90% (s) | 33.0 |
| Overshoot (% of step) | 19.1% |
| Damping ratio ζ | 0.466 |
| Settling time T_s (s) | 91.0 |
| HoldMAE_s (°), post-settle | 4.74 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 18.8 |
| Mean dt (ms) | 53.1 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 97.0 |
| dt_max (ms) | 117.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 1.25 |
| MAE fast motion (°) | 1.22 |
| MAE slow motion (°) | 1.34 |
| Bias IMU-ENC (°) | −1.25 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | +0.26 |
| Hold std (°) | 4.49 |
| Hold P95 \|error\| (°) | 9.05 |
| Hold max \|error\| (°) | 13.10 |
| Pearson r (hold window) | 0.9997 |
| FFT dominant freq (Hz) | 0.045 |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 41.9 |
| RMS dM2/dt (throttle/s) | 41.9 |
| ANG_I mean (hold, deg/s) | 3.59 |
| M2−M1 mean (hold, throttle) | +1.4 |
| I-term sign vs ΔM | OK |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 13.80 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Hold quality**: HoldMAE_s of 4.74° is oscillation-dominated — hold std of 4.49° is 17× the signed bias of +0.26°. The FFT dominant frequency of 0.045 Hz (≈22s period) is a very slow, large-amplitude wobble, with P95 reaching 9.05°.

- **Transient response**: Settling time of 91.0s out of a 120s run leaves only 29s of hold window — the system spent 76% of the run converging. Rise time (33.0s) and T→SP (31.8s) are near-identical (same pattern as the previous run: slow approach, no distinct ring-down). Damping ratio 0.466 is more underdamped than expected for a clean hold.

- **Timing jitter**: dt_p99 of 97ms is nearly 2× the median (50ms), compared to 70ms in the previous run. The max of 117ms is visible. This level of jitter doesn't breach the smoke threshold but is large enough to affect D-term accuracy on individual cycles.

- **Inner loop**: Rate tracking RMS of 13.80°/s during hold is substantially elevated — consistent with the large-amplitude 0.045 Hz oscillation driving correspondingly large rate commands that the inner loop must track.

- **I-term sign check**: Passes OK for the first time — ANG_I mean (+3.59) and M2−M1 mean (+1.4) are both positive and the guard condition is met (|ANG_I| > |ANG_P|). The I-term is correctly reflected in the motor differential direction.