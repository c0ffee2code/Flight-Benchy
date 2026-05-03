# Flight Analysis: 2026-05-02_15-53-53

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_15-53-53 |
| Duration | 119.9 s |
| Samples | 2277 |
| Start angle | 51.9° |
| Standard start | YES (within ±10° of +58°) |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
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
2026-05-02_15-53-53          51.9   ok      YES       4.2s      2.80     115.4s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_15-53-53
```

### profile_flight.py

```
Loaded 2026-05-02_15-53-53: 2277 samples, 119.9s

==========================================================
  2026-05-02_15-53-53
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      4.2
  HoldMAE (°), post-reach                      2.80
  T@0 (s)                                     115.4

  --- Sample Rate ---
  Samples                                      2277
  Duration (s)                                119.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.7
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.34
  MAE (fast motion)                            0.51
  MAE (slow motion)                            0.26
  Max AE                                       4.13
  RMS Error                                    0.52
  Bias (IMU-ENC)                               0.03

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        3.38
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  3.40
  ENC Bias (°)                                 0.93
  ENC Max AE (°)                              51.94

  --- Correlation & Tracking ---
  Pearson r                                  0.9953
  IMU trails motion (%)                         0.0
  Encoder range (°)                            62.4
  IMU range (°)                                58.1

  --- Oscillation & Windup —
  Oscillation freq (Hz)                        1.05  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_15-53-53/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 4.2 |
| HoldMAE (°), post-reach | 2.80 |
| T@0 (s) | 115.4 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 0.34 |
| IMU-ENC bias (°) | 0.03 |
| Pearson r | 0.9953 |
| IMU trails motion (%) | 0.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 1.05 | ≈1.0s period — visible in subplot 1 throughout hold |
| Whole-run ENC MAE (°) | 3.38 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy:** HoldMAE of 2.80° is oscillation-dominated at 1.05 Hz. The oscillation is visibly smaller in amplitude in subplot 1 compared to prior kp=3.5 runs — tighter wobble around 0° throughout the hold. ENC bias of +0.93° shows the lever hovering slightly M1-low, with the angle I-term in subplot 3 accumulating positively to compensate.

- **Rise:** T→0 of 4.2s is slower than kp=3.5 runs (2.1–2.8s), consistent with reduced proportional drive at large angle errors. Encoder range of 62.4° confirms the lever reached horizontal with minimal overshoot — the controlled pre-spin rise profile is intact.

- **Control loop:** Angle PID P-term in subplot 3 oscillates at 1.05 Hz with noticeably smaller swing amplitude than at kp=3.5. Zero windup events on both loops. Rate PID D-term in subplot 4 active throughout.

- **Sensor health:** IMU-ENC MAE of 0.34° and bias of 0.03° are excellent. Pearson r=0.9953.