# Flight Analysis: 2026-05-02_09-17-54

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_09-17-54 |
| Duration | 251.8 s |
| Samples | 4781 |
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
2026-05-02_09-17-54          52.1   ok      YES       4.4s      2.60     242.7s   251.8s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_09-17-54
```

### profile_flight.py

```
Loaded 2026-05-02_09-17-54: 4781 samples, 251.8s

==========================================================
  2026-05-02_09-17-54
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      4.4
  HoldMAE (°), post-reach                      2.60
  T@0 (s)                                     242.7

  --- Sample Rate ---
  Samples                                      4781
  Duration (s)                                251.8
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.7
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.37
  MAE (fast motion)                            0.49
  MAE (slow motion)                            0.30
  Max AE                                       4.60
  RMS Error                                    0.58
  Bias (IMU-ENC)                               0.15

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        2.89
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  2.97
  ENC Bias (°)                                 0.40
  ENC Max AE (°)                              52.12

  --- Correlation & Tracking ---
  Pearson r                                  0.9922
  IMU trails motion (%)                         0.0
  Encoder range (°)                            62.9
  IMU range (°)                                58.4

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.74  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_09-17-54/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 4.4 |
| HoldMAE (°), post-reach | 2.60 |
| T@0 (s) | 242.7 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 0.37 |
| IMU-ENC bias (°) | 0.15 |
| Pearson r | 0.9922 |
| IMU trails motion (%) | 0.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.74 | |
| Whole-run ENC MAE (°) | 2.89 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy**: HoldMAE of 2.60° is oscillation-dominated at 0.74 Hz, visible in subplot 1 as a regular ≈2–3° symmetric wobble. ENC bias of 0.40° is low, confirming no meaningful steady-state offset — the lever is hunting symmetrically around 0° rather than sitting off-center.

- **Oscillation damping**: The oscillation frequency dropped substantially compared to the expo=0.0 baseline (plot shows slower, less energetic swings during hold). The rate D term in subplot 4 is visibly more active than in the expo=0.3 runs, consistent with increased damping action from kd=0.009. However, a noticeable amplitude increase is visible in subplot 1 in the final ~40 seconds of the run, which is not reflected in the overall HoldMAE — the hold window averages a mix of well-damped and degraded sections.

- **Sensor health**: IMU-ENC MAE of 0.37° and bias of 0.15° are the best values seen across recent runs. IMU trails motion 0.0% — perfect synchronisation throughout. Pearson r=0.9922 remains solid. The late-run amplitude growth in subplot 1 is mirrored in both encoder and IMU, ruling out a sensor artifact.

- **Control loop**: Zero windup events on both loops across the full 251.8 s run. The angle I-term in subplot 3 remains near-flat during hold, consistent with the low 0.40° encoder bias requiring minimal integrator compensation. No I-term buildup or drift observed.

- **Motor balance**: M1 and M2 sit roughly symmetrically around 550–650 during the hold in subplot 5. The late-run amplitude increase is visible as larger M1/M2 excursions in subplot 5, suggesting the mechanical or battery state changed toward the end of the run rather than a steady-state control failure.