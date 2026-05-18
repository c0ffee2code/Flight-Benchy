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
| motor | base=500, min=100, max=900 |
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

  Acceptance levels (tolerance: +/-10 deg):
    hold_mae_deg             3.25  ->  good
    time_to_sp_s            23.35  ->  below_pass
    settling_time_s         23.35  ->  good
    hold_duration_s         96.60  ->  good
    overshoot_pct           14.36  ->  good

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

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                        2.5
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_16-16-07/01_timeseries.png` — full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-05-14_16-16-07/02_step_response.png` — full run milestones + transient zoom
- `test_runs/flights/2026-05-14_16-16-07/03_spectrum.png` — PSD of hold-window error
- `test_runs/flights/2026-05-14_16-16-07/04_hold_error_distribution.png` — hold-error histogram
- `test_runs/flights/2026-05-14_16-16-07/05_phase_portrait.png` — phase portrait, time-coloured trajectory

---

## KPI Scorecard

| Metric | Value | Level |
|--------|-------|-------|
| Reached setpoint | YES | — |
| T->SP (s) | 23.3 | below_pass |
| Rise time 10-90% (s) | 26.9 | — |
| Overshoot (% of step) | 14.4% | good |
| Damping ratio zeta | 0.526 | — |
| Settling time T_s (s) | 23.3 | good |
| Hold Duration (s) | 96.6 | good |
| HoldMAE_s (°), post-settle | 3.25 | good |

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
| Bias IMU-ENC (°) | -1.14 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | -0.15 |
| Hold std (°) | 3.82 |
| Hold P95 \|error\| (°) | 6.59 |
| Hold max \|error\| (°) | 9.84 |
| Pearson r (hold window) | 0.9997 |
| FFT dominant freq (Hz) | 0.021 (advisory) |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 43.8 |
| RMS dM2/dt (throttle/s) | 43.8 |
| ANG_I mean (hold, deg/s) | -0.03 |
| M2-M1 mean (hold, throttle) | -2.9 |
| I-term sign vs dM | N/A (P-term dominant) |

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

- **Transient response:** T->SP of 23.3 s is well below the 10 s pass threshold, indicating an unusually slow approach to the band. Notably T_s equals T->SP exactly (23.3 s), meaning the lever entered the band for the first time and immediately remained settled without any prior excursion — the crossing was the final one, not a bounce. Rise time 10-90% of 26.9 s is consistent with a gentle, low-velocity approach. Overshoot of 14.4% is good despite the slow rise.

- **Hold quality:** HoldMAE_s of 3.25° with bias -0.15° and std 3.82° is oscillation-dominated — the hold is nearly centered but cycles through the band. P95 of 6.59° and max of 9.84° both stay inside the ±10° band. The FFT advisory at 0.021 Hz (one cycle ~48 s) is within 3x the frequency resolution (0.010 Hz), so this may be slow drift rather than a resolved oscillation frequency.

- **Control effort:** Mean throttle exactly at base (500.0), zero saturation at either limit, and perfectly symmetric motor activity (RMS dM1/dt = RMS dM2/dt = 43.8 throttle/s). ANG_I mean of -0.03 deg/s and M2-M1 mean of -2.9 throttle units are negligible — no steady-state disturbance load evident.

- **Sensor health:** IMU-ENC bias of -1.14° is consistent across fast and slow motion phases (1.15° vs 1.21°), indicating a fixed systematic offset. IMU trails at 0.0% and Pearson r 0.9997 in the hold window confirm tight sensor agreement throughout.
