# Flight Analysis: 2026-05-02_10-57-21

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_10-57-21 |
| Duration | 213.7 s |
| Samples | 4022 |
| Start angle | 52.1° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0 |
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
2026-05-02_10-57-21          52.1   ok      YES       3.4s      3.63     206.4s   213.7s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_10-57-21
```

### profile_flight.py

```
Loaded 2026-05-02_10-57-21: 4022 samples, 213.7s

==========================================================
  2026-05-02_10-57-21
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      3.4
  HoldMAE (°), post-reach                      3.63
  T@0 (s)                                     206.4

  --- Sample Rate ---
  Samples                                      4022
  Duration (s)                                213.7
  Achieved Hz                                  18.8
  Mean dt (ms)                                 53.1
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.40
  MAE (fast motion)                            0.51
  MAE (slow motion)                            0.35
  Max AE                                       4.70
  RMS Error                                    0.56
  Bias (IMU-ENC)                              -0.03

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        3.89
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  3.94
  ENC Bias (°)                                 1.08
  ENC Max AE (°)                              52.12

  --- Correlation & Tracking ---
  Pearson r                                  0.9940
  IMU trails motion (%)                        22.2
  Encoder range (°)                            60.4
  IMU range (°)                                55.3

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.83  (ENC-based)
  Angle windup events                           0
  Angle windup threshold                       50.0
  Rate windup events                            0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_10-57-21/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 3.4 |
| HoldMAE (°), post-reach | 3.63 |
| T@0 (s) | 206.4 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.8 |
| IMU-ENC MAE (°) | 0.40 |
| IMU-ENC bias (°) | −0.03 |
| Pearson r | 0.9940 |
| IMU trails motion (%) | 22.2 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.83 | |
| Whole-run ENC MAE (°) | 3.89 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy**: HoldMAE of 3.63° is bias-dominated: ENC bias of 1.08° accounts for roughly a third of the error budget, with the lever hovering ~1° below horizontal rather than oscillating symmetrically around 0°. The 0.83 Hz oscillation visible in subplot 1 contributes the remaining amplitude. IMU-ENC bias of −0.03° rules out sensor offset as the cause — the IMU and encoder agree closely, so the GRV is reporting the same ~1° offset as the encoder sees.

- **Control loop**: Zero windup events on both loops. The angle I-term in subplot 3 shows a small but persistent positive buildup during hold — the integrator is accumulating to compensate for the ~1° offset, but is delivering insufficient corrective force to fully eliminate it. This is the classic under-correction signature: integrator active, no windup, yet residual offset remains. With ki=0.05 and the output_limit=130, the I-term headroom is substantial — the gain is not the constraint.

- **Sensor health**: IMU-ENC MAE of 0.40°, bias −0.03°, Pearson r=0.9940 — all clean. IMU trails motion 22.2%, consistent with the slower dynamics during the 3.4s rise.

- **Motor balance**: M1 and M2 appear roughly symmetric in subplot 5 during hold. The persistent positive ENC bias (lever below horizontal, M1-side elevated) would normally produce a slight M2 lead to correct it; any such asymmetry is within the noise of the motor output plot.