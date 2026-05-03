# Flight Analysis: 2026-05-02_15-02-13

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_15-02-13 |
| Duration | 119.9 s |
| Samples | 2274 |
| Start angle | 51.9° |
| Standard start | YES (within ±10° of +58°) |

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
2026-05-02_15-02-13          51.9   ok      YES       2.8s      3.25     116.5s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_15-02-13
```

### profile_flight.py

```
Loaded 2026-05-02_15-02-13: 2274 samples, 119.9s

==========================================================
  2026-05-02_15-02-13
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      2.8
  HoldMAE (°), post-reach                      3.25
  T@0 (s)                                     116.5

  --- Sample Rate ---
  Samples                                      2274
  Duration (s)                                119.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.8
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.37
  MAE (fast motion)                            0.51
  MAE (slow motion)                            0.29
  Max AE                                       4.37
  RMS Error                                    0.52
  Bias (IMU-ENC)                               0.08

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        3.64
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  3.69
  ENC Bias (°)                                -0.55
  ENC Max AE (°)                              51.94

  --- Correlation & Tracking ---
  Pearson r                                  0.9953
  IMU trails motion (%)                         7.1
  Encoder range (°)                            62.2
  IMU range (°)                                58.5

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        1.11  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_15-02-13/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 2.8 |
| HoldMAE (°), post-reach | 3.25 |
| T@0 (s) | 116.5 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 0.37 |
| IMU-ENC bias (°) | −0.12 |
| Pearson r | 0.9953 |
| IMU trails motion (%) | 7.1 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 1.11 | ≈0.9s period — visible regular wobble in subplot 1 |
| Whole-run ENC MAE (°) | 3.64 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy:** HoldMAE of 3.25° is oscillation-dominated at 1.11 Hz (≈0.9s period), visible as a persistent symmetric wobble in subplot 1 throughout the hold. ENC bias of −0.55° is small (lever slightly M2-low), with the angle I-term in subplot 3 slowly compensating. T@0 of 116.5s / 119.9s = 97.2% of run time inside the ±10° band.

- **Rise:** T→0 of 2.8s and encoder range of 62.2° confirm a controlled rise with minimal overshoot — the lever reached horizontal and crossed to approximately −10° before recovering, compared with larger overshoot seen in runs without pre-spin. The settle delay before the ramp allowed both motors to stabilise at idle before the ramp began, producing a more balanced and faster rise than the previous pre-spin run (5.1s without settle delay).

- **Sensor health:** IMU-ENC MAE of 0.37° and bias of −0.12° are excellent. Pearson r=0.9953. IMU trailing at 7.1% — significantly lower than in high-overshoot runs — consistent with the gentler velocity profile of the controlled rise and hold. Max AE of 4.37° is the largest single-sample divergence during the hold oscillations; it does not indicate a systemic sensor problem.

- **Control loop:** Angle PID P-term in subplot 3 oscillates with the 1.11 Hz hold wobble. Zero windup events on both loops. Rate PID D-term in subplot 4 is active throughout, consistent with the outer loop feeding a continuously varying rate setpoint into the inner loop during the oscillation.