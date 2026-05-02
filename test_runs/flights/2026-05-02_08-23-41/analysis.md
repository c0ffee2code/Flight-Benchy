# Flight Analysis: 2026-05-02_08-23-41

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_08-23-41 |
| Duration | 196.7 s |
| Samples | 3691 |
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
2026-05-02_08-23-41          52.1   ok      YES       9.0s      3.49     185.6s   196.7s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_08-23-41
```

### profile_flight.py

```
Loaded 2026-05-02_08-23-41: 3691 samples, 196.7s

==========================================================
  2026-05-02_08-23-41
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.006, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      9.0
  HoldMAE (°), post-reach                      3.49
  T@0 (s)                                     185.6

  --- Sample Rate ---
  Samples                                      3691
  Duration (s)                                196.7
  Achieved Hz                                  18.8
  Mean dt (ms)                                 53.3
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.51
  MAE (fast motion)                            0.60
  MAE (slow motion)                            0.47
  Max AE                                       4.54
  RMS Error                                    0.83
  Bias (IMU-ENC)                               0.31

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        4.40
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  4.48
  ENC Bias (°)                                 0.39
  ENC Max AE (°)                              52.12

  --- Correlation & Tracking ---
  Pearson r                                  0.9935
  IMU trails motion (%)                        47.1
  Encoder range (°)                            62.7
  IMU range (°)                                57.9

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.89  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_08-23-41/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 9.0 |
| HoldMAE (°), post-reach | 3.49 |
| T@0 (s) | 185.6 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.8 |
| IMU-ENC MAE (°) | 0.51 |
| IMU-ENC bias (°) | 0.31 |
| Pearson r | 0.9935 |
| IMU trails motion (%) | 47.1 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.89 | |
| Whole-run ENC MAE (°) | 4.40 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy**: HoldMAE of 3.49° is oscillation-dominated — the 0.89 Hz frequency is clearly visible in subplot 1 as a regular ≈2–4° symmetric wobble around 0°. ENC steady-state bias of 0.39° is small, so there is no meaningful offset component in the error.

- **Control loop**: Neither loop reached its iterm_limit across the full 196.7 s run (angle threshold 50.0, rate threshold 25.0 — both zero windup events). The outer I-term (ki=0.05) is active in subplot 3 but remains bounded, providing slow compensation without saturation. Rate ki=0.0, so no rate integrator is expected and none is visible.

- **Motor balance**: M1 and M2 sit roughly symmetrically in subplot 5, both hovering around 400–500 throttle during hold, consistent with the low 0.39° encoder bias. The small positive bias (lever sits slightly above 0°) is reflected in a marginal M2 lead visible during hold.

- **Sensor health**: IMU-ENC MAE of 0.51° and bias of 0.31° are within normal range; Pearson r = 0.9935 confirms tight coupling between encoder and IMU throughout the run. IMU trails motion 47.1% — essentially half, which is the expected value for a well-synchronized pair and requires no action.

- **Rise time**: T→0 = 9.0 s from a 52.1° start is notably slow. The expo=0.3 setting in the motor config attenuates small corrections near the setpoint but should not reduce authority at large angles like 52°, where expo output approaches the linear curve. The cause of the slow rise is not identifiable from telemetry alone without knowing what expo=0.3 is doing to the effective throttle range at large PID outputs.