# Flight Analysis: 2026-05-02_15-37-18

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_15-37-18 |
| Duration | 119.9 s |
| Samples | 2273 |
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
2026-05-02_15-37-18          51.9   ok      YES       2.1s      3.89     117.5s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_15-37-18
```

### profile_flight.py

```
Loaded 2026-05-02_15-37-18: 2273 samples, 119.9s

==========================================================
  2026-05-02_15-37-18
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      2.1
  HoldMAE (°), post-reach                      3.89
  T@0 (s)                                     117.5

  --- Sample Rate ---
  Samples                                      2273
  Duration (s)                                119.9
  Achieved Hz                                  18.9
  Mean dt (ms)                                 52.8
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.42
  MAE (fast motion)                            0.58
  MAE (slow motion)                            0.35
  Max AE                                       3.94
  RMS Error                                    0.56
  Bias (IMU-ENC)                              -0.15

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        4.21
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  4.31
  ENC Bias (°)                                -1.42
  ENC Max AE (°)                              51.94

  --- Correlation & Tracking ---
  Pearson r                                  0.9951
  IMU trails motion (%)                         0.0
  Encoder range (°)                            62.7
  IMU range (°)                                58.7

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.78  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_15-37-18/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 2.1 |
| HoldMAE (°), post-reach | 3.89 |
| T@0 (s) | 117.5 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.9 |
| IMU-ENC MAE (°) | 0.42 |
| IMU-ENC bias (°) | −0.15 |
| Pearson r | 0.9951 |
| IMU trails motion (%) | 0.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.78 | ≈1.3s period — visible throughout hold |
| Whole-run ENC MAE (°) | 4.21 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy:** HoldMAE of 3.89° is a combination of oscillation and a negative bias component. ENC bias of −1.42° shows the lever hovering below horizontal for most of the hold — the angle I-term in subplot 3 is visibly drifting negative throughout, confirming it is accumulating to push M2 throttle down and correct the offset. The oscillation at 0.78 Hz (≈1.3s period) is the other contributor, visible as sustained swings in subplot 1 that grow somewhat in the second half of the hold.

- **Rise:** T→0 of 2.1s and encoder range of 62.7° are consistent with the pre-spin profile — controlled rise with minimal overshoot, reaching just inside the ±10° band before stabilising.

- **Control loop:** Angle PID P-term in subplot 3 oscillates at 0.78 Hz. Zero windup events on both loops. Rate PID D-term in subplot 4 is consistently active, tracking the oscillating rate setpoint from the outer loop.

- **Sensor health:** IMU-ENC MAE of 0.42° and bias of −0.15° are healthy. Pearson r=0.9951. IMU trailing 0.0% — the IMU is not lagging behind the encoder in this run; the feedforward lead compensation is matching or slightly ahead of actual motion throughout the hold dynamics.