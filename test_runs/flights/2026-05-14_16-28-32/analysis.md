# Flight Analysis: 2026-05-14_16-28-32

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_16-28-32 |
| Duration | 119.9 s |
| Samples | 2290 |
| Start angle | 51.9° |
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
2026-05-14_16-28-32        51.9deg  ok      YES       3.9s          1.88deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 18.6s     Overshoot: 11.2%     T_s (settling): 17.7s
  Damping ratio zeta: 0.572

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_16-28-32
```

### profile_flight.py

```
Loaded 2026-05-14_16-28-32: 2290 samples, 119.9s

==========================================================
  2026-05-14_16-28-32
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.5, iterm_limit=5.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                    3.9s
  Rise time 10-90% (s)                        18.6s
  Overshoot (% of step)                       11.2%
  Damping ratio zeta                          0.572
  Settling time T_s (s)                       17.7s
  HoldMAE_s (deg), post-settle              1.88deg

  --- Sample Rate ---
  Samples                                      2290
  Duration (s)                                119.9
  Achieved Hz                                  19.1
  Mean dt (ms)                                 52.4
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  70.0
  dt_max (ms)                                  88.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                0.97
  MAE (fast motion)                            1.12
  MAE (slow motion)                            0.93
  Max AE                                       4.82
  RMS error                                    1.09
  Bias (IMU-ENC)                              -0.97
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          57.7
  IMU range (deg)                              53.5

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      3.57
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                      0.55
  Hold std (deg)                               4.53
  Hold P95 |error| (deg)                      12.57
  Hold max |error| (deg)                      15.03
  Pearson r (hold window)                    0.9997
  FFT dominant freq (Hz)                      0.034
    (freq resolution Hz)                      0.009

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      40.4
  RMS dM2/dt (throttle/s)                      40.4
  ANG_I mean (hold, deg/s)                    -0.15
  M2-M1 mean (hold, throttle)                  -0.9
  I-term sign vs dM                      N/A (P-term dominant)

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   13.34

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                        2.5
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_16-28-32/01_timeseries.png`
- `test_runs/flights/2026-05-14_16-28-32/02_step_response.png`
- `test_runs/flights/2026-05-14_16-28-32/03_spectrum.png`
- `test_runs/flights/2026-05-14_16-28-32/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_16-28-32/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 3.9 |
| Rise time 10-90% (s) | 18.6 |
| Overshoot (% of step) | 11.2% |
| Damping ratio ζ | 0.572 |
| Settling time T_s (s) | 17.7 |
| HoldMAE_s (°), post-settle | 1.88 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.4 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 70.0 |
| dt_max (ms) | 88.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 0.97 |
| MAE fast motion (°) | 1.12 |
| MAE slow motion (°) | 0.93 |
| Bias IMU-ENC (°) | −0.97 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | +0.55 |
| Hold std (°) | 4.53 |
| Hold P95 \|error\| (°) | 12.57 |
| Hold max \|error\| (°) | 15.03 |
| Pearson r (hold window) | 0.9997 |
| FFT dominant freq (Hz) | 0.034 |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 40.4 |
| RMS dM2/dt (throttle/s) | 40.4 |
| ANG_I mean (hold, deg/s) | −0.15 |
| M2−M1 mean (hold, throttle) | −0.9 |
| I-term sign vs ΔM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 13.34 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Best headline KPIs in session**: HoldMAE_s = 1.88° is the best hold accuracy of all runs in this session. T_s = 17.7s with overshoot = 11.2% — both the lowest overshoot and the fastest settling of the three iterm_limit=5 runs. T→SP = 3.9s is the fastest first-reach in the session, likely reflecting a favourable initial approach dynamic. The short rise (18.6s) with low overshoot (11.2%) and quick settle (17.7s) combine for the highest hold quality result.

- **Hold oscillation at 0.034 Hz**: FFT dominant freq = 0.034 Hz (period ≈ 29s), just above the 3× resolution threshold (0.027 Hz) — a resolved slow oscillation. Hold std = 4.53° and P95 = 12.57° reflect its presence. The headline HoldMAE_s = 1.88° is low because the lever spends most time near setpoint; the P95 value shows occasional excursions to ±12°. This slow wobble is not a settling issue (T_s = 17.7s is clean) but a residual hold dynamic.

- **Integrator near-zero, near-symmetric motors**: ANG_I mean = −0.15 deg/s, M2−M1 mean = −0.9 throttle — the most symmetric motor pairing in this session. The lever is holding very close to its mechanical balance point, requiring minimal P-term correction and no integrator contribution.

- **Timing consistent**: dt_p99 = 70.0ms, the best in the iterm_limit=5 group. Rate tracking RMS = 13.34°/s — slightly higher than iterations 5–6 (11.52–11.88°/s), consistent with the resolved 0.034 Hz oscillation creating slightly more rate setpoint demand during hold.
