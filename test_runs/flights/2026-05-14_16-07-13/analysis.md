# Flight Analysis: 2026-05-14_16-07-13

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_16-07-13 |
| Duration | 119.9 s |
| Samples | 2260 |
| Start angle | 51.9° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.5, iterm_limit=10.0 |
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
2026-05-14_16-07-13        51.9deg  ok      YES       9.8s          5.01deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 28.5s     Overshoot: 18.6%     T_s (settling): 20.6s
  Damping ratio zeta: 0.472

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_16-07-13
```

### profile_flight.py

```
Loaded 2026-05-14_16-07-13: 2260 samples, 119.9s

==========================================================
  2026-05-14_16-07-13
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.5, iterm_limit=10.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                    9.8s
  Rise time 10-90% (s)                        28.5s
  Overshoot (% of step)                       18.6%
  Damping ratio zeta                          0.472
  Settling time T_s (s)                       20.6s
  HoldMAE_s (deg), post-settle              5.01deg

  --- Sample Rate ---
  Samples                                      2260
  Duration (s)                                119.9
  Achieved Hz                                  18.8
  Mean dt (ms)                                 53.1
  Median dt (ms)                               51.0
  dt_p99 (ms)                                  96.0
  dt_max (ms)                                 122.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                0.96
  MAE (fast motion)                            1.02
  MAE (slow motion)                            0.94
  Max AE                                       4.83
  RMS error                                    1.12
  Bias (IMU-ENC)                              -0.96
  IMU trails motion (%)                         7.1
  Encoder range (deg)                          61.6
  IMU range (deg)                              57.3

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      6.40
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                     -0.89
  Hold std (deg)                               6.24
  Hold P95 |error| (deg)                      10.11
  Hold max |error| (deg)                      12.48
  Pearson r (hold window)                    0.9998
  FFT dominant freq (Hz)                      0.009
    (freq resolution Hz)                      0.009
  (advisory - peak within 3x resolution; may be drift rather than oscillation)

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      47.2
  RMS dM2/dt (throttle/s)                      47.2
  ANG_I mean (hold, deg/s)                    -0.14
  M2-M1 mean (hold, throttle)                  -5.2
  I-term sign vs dM                      N/A (P-term dominant)

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   19.24

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                        5.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_16-07-13/01_timeseries.png`
- `test_runs/flights/2026-05-14_16-07-13/02_step_response.png`
- `test_runs/flights/2026-05-14_16-07-13/03_spectrum.png`
- `test_runs/flights/2026-05-14_16-07-13/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_16-07-13/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 9.8 |
| Rise time 10-90% (s) | 28.5 |
| Overshoot (% of step) | 18.6% |
| Damping ratio ζ | 0.472 |
| Settling time T_s (s) | 20.6 |
| HoldMAE_s (°), post-settle | 5.01 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 18.8 |
| Mean dt (ms) | 53.1 |
| Median dt (ms) | 51.0 |
| dt_p99 (ms) | 96.0 |
| dt_max (ms) | 122.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 0.96 |
| MAE fast motion (°) | 1.02 |
| MAE slow motion (°) | 0.94 |
| Bias IMU-ENC (°) | −0.96 |
| IMU trails motion (%) | 7.1 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | −0.89 |
| Hold std (°) | 6.24 |
| Hold P95 \|error\| (°) | 10.11 |
| Hold max \|error\| (°) | 12.48 |
| Pearson r (hold window) | 0.9998 |
| FFT dominant freq (Hz) | 0.009 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 47.2 |
| RMS dM2/dt (throttle/s) | 47.2 |
| ANG_I mean (hold, deg/s) | −0.14 |
| M2−M1 mean (hold, throttle) | −5.2 |
| I-term sign vs ΔM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 19.24 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **I-term effectively suppressed by iterm_limit=10**: ANG_I mean during hold is −0.14 deg/s — essentially zero. Zero windup events confirms the I-term never reached the 10 deg/s limit during this run, which implies approach accumulation was well below the cap. Despite this, M2−M1 mean is −5.2 throttle: the steady angular offset is being corrected by the P-term alone with no integrator contribution.

- **Hold quality degraded vs prior fast-settle runs**: HoldMAE_s = 5.01° with hold std = 6.24°. FFT at 0.009 Hz is advisory (at the resolution floor), consistent with drift rather than a resolved oscillation. The high hold std with near-zero I-term indicates the hold is error-bounded by P-term tracking only — without the integrator accumulating to eliminate steady-state error, the lever is held in a dynamic balance by proportional correction.

- **Rate tracking RMS elevated**: 19.24°/s during the hold window is the highest of any run in this session. dt_p99 = 96ms and dt_max = 122ms are also the highest in this session (though still well below the smoke-check limit of 255ms). These are correlated — the higher P-only correction activity increases commanded rate setpoints, and the worse loop timing means the inner loop responds to those setpoints with more jitter.

- **Transient: fast T→SP, still borderline overshoot**: T→SP = 9.8s (fastest reach in the session) with 18.6% overshoot. The run fast-settled at T_s = 20.6s despite overshoot at the upper edge of the fast-settle zone. Damping ratio ζ = 0.472, consistent with prior fast-settle runs (0.525–0.600 fast, 0.459–0.466 slow).
