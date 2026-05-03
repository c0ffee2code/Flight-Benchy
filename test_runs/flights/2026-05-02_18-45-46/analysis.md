# Flight Analysis: 2026-05-02_18-45-46

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_18-45-46 |
| Duration | 119.9 s |
| Samples | 2268 |
| Start angle | 51.9° |
| Standard start | YES |

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
2026-05-02_18-45-46          51.9   ok      YES       4.1s      3.40     112.8s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_18-45-46
```

### profile_flight.py

```
Loaded 2026-05-02_18-45-46: 2268 samples, 119.9s

==========================================================
  2026-05-02_18-45-46
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      4.1
  HoldMAE (°), post-reach                      3.40
  T@0 (s)                                     112.8

  --- Sample Rate ---
  Samples                                      2268
  Duration (s)                                119.9
  Achieved Hz                                  18.9
  Mean dt (ms)                                 52.9
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.48
  MAE (fast motion)                            0.67
  MAE (slow motion)                            0.38
  Max AE                                       4.21
  RMS Error                                    0.71
  Bias (IMU-ENC)                              -0.09

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        3.93
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  4.05
  ENC Bias (°)                                -0.95
  ENC Max AE (°)                              51.94

  --- Correlation & Tracking ---
  Pearson r                                  0.9929
  IMU trails motion (%)                        21.1
  Encoder range (°)                            67.0
  IMU range (°)                                63.7

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.86  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_18-45-46/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 4.1 |
| HoldMAE (°), post-reach | 3.40 |
| T@0 (s) | 112.8 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.9 |
| IMU-ENC MAE (°) | 0.48 |
| IMU-ENC bias (°) | −0.09 |
| Pearson r | 0.9929 |
| IMU trails motion (%) | 21.1 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.86 | |
| Whole-run ENC MAE (°) | 3.93 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy** — 3.40° HoldMAE is oscillation-dominated rather than bias-dominated: ENC bias is only −0.95°, so the lever is centred close to 0° on average. The 0.86 Hz oscillation frequency matches the visible slow symmetric wobble in subplot 1, with the encoder swinging ±3–5° around zero throughout the hold.

- **Control loop** — zero windup events in both loops despite 112.8s of hold time. The angle I-term is flat and near-zero in subplot 3, consistent with the small −0.95° bias not driving sustained integrator accumulation at ki=0.05.

- **Motor balance** — M1 and M2 run visually symmetric during hold in subplot 5, consistent with the near-zero IMU-ENC bias (−0.09°) and the small ENC bias (−0.95°). No sustained asymmetry is visible.

- **Sensor health** — IMU-ENC MAE of 0.48° and Pearson r=0.993 indicate clean tracking throughout. The −0.09° IMU-ENC bias is negligible. Trail percentage of 21.1% is within the expected range for GRV lag at 50 Hz.