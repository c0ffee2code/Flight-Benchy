# Flight Analysis: 2026-05-14_14-26-21

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_14-26-21 |
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
2026-05-14_14-26-21        51.9deg  ok      YES      14.4s          2.82deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 17.2s     Overshoot: 14.4%     T_s (settling): 14.4s
  Damping ratio zeta: 0.525

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_14-26-21
```

### profile_flight.py

```
Loaded 2026-05-14_14-26-21: 2289 samples, 119.9s

==========================================================
  2026-05-14_14-26-21
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.5, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   14.4s
  Rise time 10-90% (s)                        17.2s
  Overshoot (% of step)                       14.4%
  Damping ratio zeta                          0.525
  Settling time T_s (s)                       14.4s
  HoldMAE_s (deg), post-settle              2.82deg

  --- Sample Rate ---
  Samples                                      2289
  Duration (s)                                119.9
  Achieved Hz                                  19.1
  Mean dt (ms)                                 52.4
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  70.1
  dt_max (ms)                                   88.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                0.93
  MAE (fast motion)                            1.01
  MAE (slow motion)                            0.92
  Max AE                                       4.75
  RMS error                                    1.05
  Bias (IMU-ENC)                              -0.93
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          59.4
  IMU range (deg)                              55.1

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      4.14
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                     -0.44
  Hold std (deg)                               3.44
  Hold P95 |error| (deg)                       6.50
  Hold max |error| (deg)                       9.84
  Pearson r (hold window)                    0.9995
  FFT dominant freq (Hz)                      0.009 (advisory)

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      42.6
  RMS dM2/dt (throttle/s)                      42.6
  ANG_I mean (hold, deg/s)                    -1.39
  M2-M1 mean (hold, throttle)                  -4.8
  I-term sign vs dM                      N/A (P-term dominant)

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   11.09

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_14-26-21/01_timeseries.png`
- `test_runs/flights/2026-05-14_14-26-21/02_step_response.png`
- `test_runs/flights/2026-05-14_14-26-21/03_spectrum.png`
- `test_runs/flights/2026-05-14_14-26-21/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_14-26-21/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 14.4 |
| Rise time 10-90% (s) | 17.2 |
| Overshoot (% of step) | 14.4% |
| Damping ratio ζ | 0.525 |
| Settling time T_s (s) | 14.4 |
| HoldMAE_s (°), post-settle | 2.82 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.4 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 70.1 |
| dt_max (ms) | 88.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 0.93 |
| MAE fast motion (°) | 1.01 |
| MAE slow motion (°) | 0.92 |
| Bias IMU-ENC (°) | −0.93 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | −0.44 |
| Hold std (°) | 3.44 |
| Hold P95 \|error\| (°) | 6.50 |
| Hold max \|error\| (°) | 9.84 |
| Pearson r (hold window) | 0.9995 |
| FFT dominant freq (Hz) | 0.009 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 42.6 |
| RMS dM2/dt (throttle/s) | 42.6 |
| ANG_I mean (hold, deg/s) | −1.39 |
| M2−M1 mean (hold, throttle) | −4.8 |
| I-term sign vs ΔM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 11.09 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Hold quality**: HoldMAE_s of 2.82° with hold std 3.44° and bias −0.44°. FFT dominant frequency is 0.009 Hz (advisory — within one bin of resolution), indicating no resolved oscillation in the hold window; error is drift-limited rather than oscillation-dominated.

- **Transient response**: Faster rise than the baseline fast-settle runs (17.2s vs 20–22s) with T_s = T→SP = 14.4s — no ring-down. Overshoot of 14.4% is slightly higher than the baseline fast runs (9.5–12.2%), giving a damping ratio of 0.525. The faster approach at kd=0.5 trades a small overshoot increase for a quicker rise.

- **Noise check**: dt_p99 of 70.1ms is consistent with baseline fast-settle runs (70–71ms) — no jitter increase from the higher D-term. Rate tracking RMS of 11.09°/s is slightly above the baseline fast-run range (9.35–9.50°/s) but well below the 15°/s falsifier threshold; no noise amplification signal.

- **Control effort**: Symmetric motors (RMS dM/dt 42.6 both sides), no saturation. I-term sign N/A (P-term dominant) — consistent with a fast-settling run where the I-term residual hasn't drained.
