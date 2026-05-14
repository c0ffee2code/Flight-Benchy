# Flight Analysis: 2026-05-14_14-10-18

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_14-10-18 |
| Duration | 119.9 s |
| Samples | 2289 |
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
2026-05-14_14-10-18        51.9deg  ok      YES      19.0s          2.58deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 22.2s     Overshoot: 9.5%      T_s (settling): 19.0s
  Damping ratio zeta: 0.600

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_14-10-18
```

### profile_flight.py

```
Loaded 2026-05-14_14-10-18: 2289 samples, 119.9s

==========================================================
  2026-05-14_14-10-18
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   19.0s
  Rise time 10-90% (s)                        22.2s
  Overshoot (% of step)                        9.5%
  Damping ratio zeta                          0.600
  Settling time T_s (s)                       19.0s
  HoldMAE_s (deg), post-settle              2.58deg

  --- Sample Rate ---
  Samples                                      2289
  Duration (s)                                119.9
  Achieved Hz                                  19.1
  Mean dt (ms)                                 52.4
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  71.0
  dt_max (ms)                                 102.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                1.10
  MAE (fast motion)                            1.19
  MAE (slow motion)                            1.05
  Max AE                                       4.66
  RMS error                                    1.27
  Bias (IMU-ENC)                              -1.10
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          56.9
  IMU range (deg)                              52.9

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      5.16
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                      0.33
  Hold std (deg)                               3.07
  Hold P95 |error| (deg)                       6.24
  Hold max |error| (deg)                       9.93
  Pearson r (hold window)                    0.9994
  FFT dominant freq (Hz)                      0.010
  (advisory - peak within 3x resolution; may be drift rather than oscillation)

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      40.2
  RMS dM2/dt (throttle/s)                      40.2
  ANG_I mean (hold, deg/s)                     2.07
  M2-M1 mean (hold, throttle)                   0.4
  I-term sign vs dM                              OK

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                    9.50

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_14-10-18/01_timeseries.png`
- `test_runs/flights/2026-05-14_14-10-18/02_step_response.png`
- `test_runs/flights/2026-05-14_14-10-18/03_spectrum.png`
- `test_runs/flights/2026-05-14_14-10-18/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_14-10-18/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 19.0 |
| Rise time 10-90% (s) | 22.2 |
| Overshoot (% of step) | 9.5% |
| Damping ratio ζ | 0.600 |
| Settling time T_s (s) | 19.0 |
| HoldMAE_s (°), post-settle | 2.58 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.4 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 71.0 |
| dt_max (ms) | 102.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 1.10 |
| MAE fast motion (°) | 1.19 |
| MAE slow motion (°) | 1.05 |
| Bias IMU-ENC (°) | −1.10 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | +0.33 |
| Hold std (°) | 3.07 |
| Hold P95 \|error\| (°) | 6.24 |
| Hold max \|error\| (°) | 9.93 |
| Pearson r (hold window) | 0.9994 |
| FFT dominant freq (Hz) | 0.010 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 40.2 |
| RMS dM2/dt (throttle/s) | 40.2 |
| ANG_I mean (hold, deg/s) | 2.07 |
| M2−M1 mean (hold, throttle) | +0.4 |
| I-term sign vs ΔM | OK |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 9.50 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Hold quality**: HoldMAE_s of 2.58° with hold std of 3.07° and a near-zero bias of +0.33°. The FFT dominant frequency of 0.010 Hz is flagged advisory (within 3× frequency resolution), suggesting no resolved dominant oscillation — the hold is close to drift-limited rather than oscillation-dominated.

- **Transient response**: Fast, clean approach — T_s = T→SP = 19.0s with 9.5% overshoot and damping ratio 0.600. The settling coincided with first reach: no ring-down phase visible.

- **Control effort**: Throttle symmetric (M2−M1 = +0.4), no saturation, RMS dM/dt symmetric at 40.2. I-term sign OK with ANG_I mean = +2.07 deg/s and M2−M1 = +0.4 both positive.

- **Inner loop**: Rate tracking RMS of 9.50°/s during hold, consistent with a quiet hold (no large oscillation to chase).
