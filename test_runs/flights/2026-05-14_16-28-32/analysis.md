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
2026-05-14_16-28-32        51.9deg  ok      YES       3.9s          1.88deg   119.9s
-------------------------------------------------------------------------------------

  Acceptance levels (tolerance: ±10°):
    hold_mae_deg             1.88  →  excellent
    time_to_sp_s             3.88  →  good
    settling_time_s         17.72  →  excellent
    hold_duration_s        102.22  →  excellent
    overshoot_pct           11.17  →  excellent

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

- `test_runs/flights/2026-05-14_16-28-32/01_timeseries.png` — full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-05-14_16-28-32/02_step_response.png` — full run milestones + transient zoom
- `test_runs/flights/2026-05-14_16-28-32/03_spectrum.png` — PSD of hold-window error
- `test_runs/flights/2026-05-14_16-28-32/04_hold_error_distribution.png` — hold-error histogram
- `test_runs/flights/2026-05-14_16-28-32/05_phase_portrait.png` — phase portrait, time-coloured trajectory

---

## KPI Scorecard

| Metric | Value | Level |
|--------|-------|-------|
| Reached setpoint | YES | — |
| T→SP (s) | 3.9 | good |
| Rise time 10-90% (s) | 18.6 | — |
| Overshoot (% of step) | 11.2% | excellent |
| Damping ratio ζ | 0.572 | — |
| Settling time T_s (s) | 17.7 | excellent |
| Hold Duration (s) | 102.2 | excellent |
| HoldMAE_s (°), post-settle | 1.88 | excellent |

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
| Hold bias (°, signed) | 0.55 |
| Hold std (°) | 4.53 |
| Hold P95 \|error\| (°) | 12.57 |
| Hold max \|error\| (°) | 15.03 |
| Pearson r (hold window) | 0.9997 |
| FFT dominant freq (Hz) | 0.034 |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
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

- **Hold quality:** HoldMAE_s 1.88° is excellent, but hold std of 4.53° and P95 of 12.57° reveal that the hold is oscillation-dominated rather than bias-dominated — the signed bias is only 0.55°, so the average is close to setpoint, but excursions frequently breach the ±10° band (max 15.03°). The FFT dominant frequency of 0.034 Hz (one cycle every ~29 s) identifies a very slow oscillation, not noise.

- **Transient response:** T→SP of 3.9 s is good but the 10–90% rise takes 18.6 s, indicating a slow, controlled approach rather than an aggressive one. Overshoot of 11.2% is excellent (below the 12% excellent threshold), and T_s of 17.7 s confirms the controller settles before the first cycle of the slow oscillation begins.

- **Control effort:** Mean throttle 500.0 equals base with zero saturation at either limit — full authority range is available throughout the hold. Motor activity is perfectly symmetric: RMS dM1/dt = RMS dM2/dt = 40.4 throttle/s, and M2−M1 mean of −0.9 throttle units is negligible. ANG_I mean of −0.15 deg/s confirms no sustained steady-state disturbance requiring integral correction.

- **Sensor health:** IMU-ENC bias of −0.97° is consistent across fast and slow motion (1.12° vs 0.93°), indicating a systematic offset rather than lag. IMU trails at 0.0% and Pearson r of 0.9997 in the hold window confirm the two sensors track each other tightly throughout.

- **Inner loop:** Rate tracking RMS of 13.34 °/s during the hold is elevated relative to a perfectly still hold, which is consistent with the slow 0.034 Hz oscillation driving non-trivial angular rate commands into the inner loop. No windup events in either loop despite the slow oscillation cycling through the tolerance band boundaries.
