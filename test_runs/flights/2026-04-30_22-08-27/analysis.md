# Flight Analysis: 2026-04-30_22-08-27

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-04-30_22-08-27 |
| Duration | 95.4 s |
| Samples | 1803 |
| Start angle | 52.0° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.006, iterm_limit=50.0 |
| motor | base=600, min=100, max=800 |
| feedforward_lead_ms | 15 |
| angle_report | game_rotation_vector @ 50 Hz |
| rate_report | calibrated_gyroscope @ 200 Hz |

---

## Raw Tool Output

### score_flight.py

```
Run                          Start  OK  Reached   T->0 (s)   HoldMAE    T@0 (s)  Dur (s)
----------------------------------------------------------------------------------------
2026-04-30_22-08-27          52.0   ok      YES       3.8s      2.09      91.6s    95.4s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-04-30_22-08-27
```

### profile_flight.py

```
Loaded 2026-04-30_22-08-27: 1803 samples, 95.4s

==========================================================
  2026-04-30_22-08-27
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.006, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      3.8
  HoldMAE (°), post-reach                      2.09
  T@0 (s)                                      91.6

  --- Sample Rate ---
  Samples                                      1803
  Duration (s)                                 95.4
  Achieved Hz                                  18.9
  Mean dt (ms)                                 53.0
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.47
  MAE (fast motion)                            0.61
  MAE (slow motion)                            0.40
  Max AE                                       4.50
  RMS Error                                    0.68
  Bias (IMU-ENC)                               0.29

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        2.75
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  2.76
  ENC Bias (°)                                 0.73
  ENC Max AE (°)                              52.03

  --- Correlation & Tracking ---
  Pearson r                                  0.9931
  IMU trails motion (%)                        14.3
  Encoder range (°)                            59.4
  IMU range (°)                                54.4

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.67  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-04-30_22-08-27/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 3.8 |
| HoldMAE (°), post-reach | 2.09 |
| T@0 (s) | 91.6 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.9 |
| IMU-ENC MAE (°) | 0.47 |
| IMU-ENC bias (°) | 0.29 |
| Pearson r | 0.9931 |
| IMU trails motion (%) | 14.3 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.67 | |
| Whole-run ENC MAE (°) | 2.75 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy** — HoldMAE of 2.09° is consistent with the visible low-frequency wobble in subplot 1. The oscillation is slow at 0.67 Hz — roughly one full cycle every 1.5 s — which matches the broad, gentle swings visible during hold rather than a tight high-frequency ripple. ENC bias of +0.73° indicates the lever is hovering fractionally above horizontal on average, contributing a small bias-dominated component on top of the oscillation.

- **Control loop** — Angle I-term builds during the rise and plateaus, with zero windup events against the 100-unit limit. The pattern is identical in character to other runs on this frame; no unexpected behaviour.

- **Motor balance** — M1 runs higher than M2 during hold, same asymmetry seen on this frame. Consistent with I-term compensation for the wire-tension disturbance; no change from prior sessions.

- **Sensor health** — IMU-ENC MAE of 0.47° and bias of 0.29° are slightly elevated compared to the next-session run, and IMU trails motion 14.3% of samples — meaning the GRV update arrived after the encoder had already moved in a notable fraction of frames. This is consistent with the GRV's 50 Hz update rate occasionally losing a race against a fast encoder sample; it is not a calibration issue. Pearson r of 0.9931 remains high and overall tracking is good.