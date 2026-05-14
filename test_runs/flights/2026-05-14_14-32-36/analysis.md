# Flight Analysis: 2026-05-14_14-32-36

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_14-32-36 |
| Duration | 119.9 s |
| Samples | 2289 |
| Start angle | 51.9° |
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
2026-05-14_14-32-36        51.9deg  ok      YES      27.3s          3.37deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 31.0s     Overshoot: 13.7%     T_s (settling): 27.3s
  Damping ratio zeta: 0.535

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_14-32-36
```

### profile_flight.py

```
Loaded 2026-05-14_14-32-36: 2289 samples, 119.9s

==========================================================
  2026-05-14_14-32-36
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.5, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   27.3s
  Rise time 10-90% (s)                        31.0s
  Overshoot (% of step)                       13.7%
  Damping ratio zeta                          0.535
  Settling time T_s (s)                       27.3s
  HoldMAE_s (deg), post-settle              3.37deg

  --- Sample Rate ---
  Samples                                      2289
  Duration (s)                                119.9
  Achieved Hz                                  19.1
  Mean dt (ms)                                 52.4
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  70.1
  dt_max (ms)                                   89.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                1.09
  MAE (fast motion)                            1.26
  MAE (slow motion)                            0.99
  Max AE                                       4.93
  RMS error                                    1.27
  Bias (IMU-ENC)                              -1.09
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          59.1
  IMU range (deg)                              54.7

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      6.40
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                     -0.74
  Hold std (deg)                               3.89
  Hold P95 |error| (deg)                       7.00
  Hold max |error| (deg)                       9.84
  Pearson r (hold window)                    0.9996
  FFT dominant freq (Hz)                      0.011 (advisory)

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      42.4
  RMS dM2/dt (throttle/s)                      42.4
  ANG_I mean (hold, deg/s)                    -1.17
  M2-M1 mean (hold, throttle)                  -5.4
  I-term sign vs dM                      N/A (P-term dominant)

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   13.01

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_14-32-36/01_timeseries.png`
- `test_runs/flights/2026-05-14_14-32-36/02_step_response.png`
- `test_runs/flights/2026-05-14_14-32-36/03_spectrum.png`
- `test_runs/flights/2026-05-14_14-32-36/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_14-32-36/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 27.3 |
| Rise time 10-90% (s) | 31.0 |
| Overshoot (% of step) | 13.7% |
| Damping ratio ζ | 0.535 |
| Settling time T_s (s) | 27.3 |
| HoldMAE_s (°), post-settle | 3.37 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.4 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 70.1 |
| dt_max (ms) | 89.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 1.09 |
| MAE fast motion (°) | 1.26 |
| MAE slow motion (°) | 0.99 |
| Bias IMU-ENC (°) | −1.09 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | −0.74 |
| Hold std (°) | 3.89 |
| Hold P95 \|error\| (°) | 7.00 |
| Hold max \|error\| (°) | 9.84 |
| Pearson r (hold window) | 0.9996 |
| FFT dominant freq (Hz) | 0.011 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 42.4 |
| RMS dM2/dt (throttle/s) | 42.4 |
| ANG_I mean (hold, deg/s) | −1.17 |
| M2−M1 mean (hold, throttle) | −5.4 |
| I-term sign vs ΔM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 13.01 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Hold quality**: HoldMAE_s of 3.37° with hold std 3.89° and bias −0.74°. FFT at 0.011 Hz is advisory (within one bin of resolution), indicating no resolved oscillation; the hold is drift-limited.

- **Transient response**: Rise of 31.0s is slower than the previous kd=0.5 run (17.2s) — more comparable to baseline fast-settle runs (20–22s). T_s = T→SP = 27.3s, no ring-down. Damping ratio 0.535 consistent with iteration 2 (0.525). The slower rise at the same config produces a slightly higher T_s but still far inside the ≤60s goal.

- **Noise and effort**: dt_p99 70.1ms and rate tracking RMS 13.01°/s — both within the expected range for kd=0.5, no degradation versus C1 qualifying run.
