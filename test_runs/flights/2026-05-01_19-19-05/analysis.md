# Flight Analysis: 2026-05-01_19-19-05

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-01_19-19-05 |
| Duration | 114.7 s |
| Samples | 2166 |
| Start angle | 52.1° |
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
2026-05-01_19-19-05          52.1   ok      YES       2.6s      2.46     112.2s   114.7s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-01_19-19-05
```

### profile_flight.py

```
Loaded 2026-05-01_19-19-05: 2166 samples, 114.7s

==========================================================
  2026-05-01_19-19-05
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.006, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      2.6
  HoldMAE (°), post-reach                      2.46
  T@0 (s)                                     112.2

  --- Sample Rate ---
  Samples                                      2166
  Duration (s)                                114.7
  Achieved Hz                                  18.9
  Mean dt (ms)                                 53.0
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.31
  MAE (fast motion)                            0.46
  MAE (slow motion)                            0.23
  Max AE                                       4.67
  RMS Error                                    0.52
  Bias (IMU-ENC)                               0.02

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        2.88
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  2.90
  ENC Bias (°)                                 0.02
  ENC Max AE (°)                              52.12

  --- Correlation & Tracking ---
  Pearson r                                  0.9945
  IMU trails motion (%)                         0.0
  Encoder range (°)                            59.5
  IMU range (°)                                55.0

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        1.38  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-01_19-19-05/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 2.6 |
| HoldMAE (°), post-reach | 2.46 |
| T@0 (s) | 112.2 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.9 |
| IMU-ENC MAE (°) | 0.31 |
| IMU-ENC bias (°) | 0.02 |
| Pearson r | 0.9945 |
| IMU trails motion (%) | 0.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 1.38 | |
| Whole-run ENC MAE (°) | 2.88 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy**: HoldMAE is 2.46° with ENC bias of 0.02° — the error is oscillation-dominated, not bias-dominated. The encoder trace in subplot 1 shows a continuous symmetric wobble around 0° throughout the hold rather than a steady offset, consistent with the 1.38 Hz oscillation frequency. The near-zero bias confirms the angle I-term is finding true horizontal.

- **Control loop**: Angle I-term shows a gradual upward drift in subplot 3 (green line), accumulating over the 112s hold, which is expected for a small persistent mechanical disturbance (wire tension). Zero windup events on both loops — with iterm_limit=100 and ki=0.05, the angle I-term would need to accumulate to 50 before triggering, which it does not over this run duration. Rate ki=0.0 so the rate I-term is absent by design and flat throughout subplot 4.

- **Sensor health**: IMU-ENC MAE of 0.31° with 0.02° bias and Pearson r=0.9945 is clean across the full 59.5° encoder range. IMU trailing = 0.0% — the IMU is not lagging the encoder on any detected direction change. The max AE of 4.67° corresponds to the oscillation peaks visible in subplot 1 and is consistent with the 2.46° HoldMAE under a ±wobble pattern.

- **Motor balance**: M1 and M2 run approximately symmetric during hold (subplot 5), with throttle chatter tracking the oscillation in lockstep. No large sustained asymmetry is visible, which matches the near-zero ENC bias — the I-term is not having to compensate for a large mechanical imbalance.