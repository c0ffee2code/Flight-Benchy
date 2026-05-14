# Flight Analysis: 2026-05-14_14-37-52

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_14-37-52 |
| Duration | 119.9 s |
| Samples | 2291 |
| Start angle | 52.0° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.5, iterm_limit=100.0 |
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
2026-05-14_14-37-52        52.0deg  ok      YES      12.3s          4.58deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 19.8s     Overshoot: 19.8%     T_s (settling): 91.5s
  Damping ratio zeta: 0.459

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_14-37-52
```

### profile_flight.py

```
Loaded 2026-05-14_14-37-52: 2291 samples, 119.9s

==========================================================
  2026-05-14_14-37-52
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.5, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   12.3s
  Rise time 10-90% (s)                        19.8s
  Overshoot (% of step)                       19.8%
  Damping ratio zeta                          0.459
  Settling time T_s (s)                       91.5s
  HoldMAE_s (deg), post-settle              4.58deg

  --- Sample Rate ---
  Samples                                      2291
  Duration (s)                                119.9
  Achieved Hz                                  19.1
  Mean dt (ms)                                 52.4
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  70.0
  dt_max (ms)                                   74.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                0.93
  MAE (fast motion)                            1.05
  MAE (slow motion)                            0.90
  Max AE                                       4.79
  RMS error                                    1.10
  Bias (IMU-ENC)                              -0.93
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          62.3
  IMU range (deg)                              57.8

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      5.10
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                     -0.41
  Hold std (deg)                               4.58
  Hold P95 |error| (deg)                      10.02
  Hold max |error| (deg)                      13.27
  Pearson r (hold window)                    0.9998
  FFT dominant freq (Hz)                      0.019 (advisory)

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      42.5
  RMS dM2/dt (throttle/s)                      42.5
  ANG_I mean (hold, deg/s)                     1.10
  M2-M1 mean (hold, throttle)                  -2.4
  I-term sign vs dM                      N/A (P-term dominant)

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   14.88

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_14-37-52/01_timeseries.png`
- `test_runs/flights/2026-05-14_14-37-52/02_step_response.png`
- `test_runs/flights/2026-05-14_14-37-52/03_spectrum.png`
- `test_runs/flights/2026-05-14_14-37-52/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_14-37-52/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 12.3 |
| Rise time 10-90% (s) | 19.8 |
| Overshoot (% of step) | 19.8% |
| Damping ratio ζ | 0.459 |
| Settling time T_s (s) | 91.5 |
| HoldMAE_s (°), post-settle | 4.58 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.4 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 70.0 |
| dt_max (ms) | 74.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 0.93 |
| MAE fast motion (°) | 1.05 |
| MAE slow motion (°) | 0.90 |
| Bias IMU-ENC (°) | −0.93 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | −0.41 |
| Hold std (°) | 4.58 |
| Hold P95 \|error\| (°) | 10.02 |
| Hold max \|error\| (°) | 13.27 |
| Pearson r (hold window) | 0.9998 |
| FFT dominant freq (Hz) | 0.019 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 42.5 |
| RMS dM2/dt (throttle/s) | 42.5 |
| ANG_I mean (hold, deg/s) | +1.10 |
| M2−M1 mean (hold, throttle) | −2.4 |
| I-term sign vs ΔM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 14.88 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Slow-settle mode reappeared**: T→SP = 12.3s (fastest reach across all runs) but T_s = 91.5s — the lever reached setpoint quickly then oscillated for 79s before settling. This is the slowest T_s observed, and it occurred at kd=0.5, falsifying the hypothesis that the slow mode is caused by insufficient D-term damping.

- **Overshoot as correlate**: overshoot of 19.8% matches the other slow-settle run (19.1%) and is distinctly higher than all fast-settle runs (9.5–14.4%). The slow mode has now appeared twice, both times with overshoot ≥19%. All fast-settle runs had overshoot ≤14.4%.

- **Hold quality during ring-down window**: HoldMAE_s of 4.58° reflects the oscillating ring-down phase (hold window starts at T→SP=12.3s, but settling isn't complete until T_s=91.5s). Rate tracking RMS of 14.88°/s is the highest across all runs, consistent with the inner loop chasing large oscillations.

- **Timing clean**: dt_p99 = 70.0ms, dt_max = 74.0ms — tightest jitter of any run in this session. The slow mode is not a loop-timing phenomenon.
