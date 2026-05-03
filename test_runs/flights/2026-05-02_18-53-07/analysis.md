# Flight Analysis: 2026-05-02_18-53-07

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_18-53-07 |
| Duration | 119.9 s |
| Samples | 2260 |
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
2026-05-02_18-53-07          51.9   ok      YES       2.1s      3.57     113.2s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_18-53-07
```

### profile_flight.py

```
Loaded 2026-05-02_18-53-07: 2260 samples, 119.9s

==========================================================
  2026-05-02_18-53-07
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      2.1
  HoldMAE (°), post-reach                      3.57
  T@0 (s)                                     113.2

  --- Sample Rate ---
  Samples                                      2260
  Duration (s)                                119.9
  Achieved Hz                                  18.8
  Mean dt (ms)                                 53.1
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.44
  MAE (fast motion)                            0.63
  MAE (slow motion)                            0.36
  Max AE                                       3.97
  RMS Error                                    0.63
  Bias (IMU-ENC)                              -0.02

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        3.94
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  4.04
  ENC Bias (°)                                -1.14
  ENC Max AE (°)                              51.94

  --- Correlation & Tracking ---
  Pearson r                                  0.9940
  IMU trails motion (%)                        21.1
  Encoder range (°)                            65.0
  IMU range (°)                                61.4

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.87  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_18-53-07/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 2.1 |
| HoldMAE (°), post-reach | 3.57 |
| T@0 (s) | 113.2 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.8 |
| IMU-ENC MAE (°) | 0.44 |
| IMU-ENC bias (°) | −0.02 |
| Pearson r | 0.9940 |
| IMU trails motion (%) | 21.1 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.87 | |
| Whole-run ENC MAE (°) | 3.94 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy** — 3.57° HoldMAE is oscillation-dominated: ENC bias is −1.14°, so the lever is centred just below zero, with the 0.87 Hz slow symmetric wobble visible in subplot 1 accounting for the bulk of the error. The oscillation frequency is essentially the same as the previous run at this config.

- **Control loop** — zero windup events across 113.2s of hold in both loops. The angle I-term is flat throughout subplot 3, consistent with the small negative bias not being large enough to drive integrator accumulation at ki=0.05.

- **Motor balance** — M1 and M2 run visually symmetric during hold in subplot 5, consistent with the near-zero IMU-ENC bias (−0.02°). No sustained asymmetry is present.

- **Sensor health** — IMU-ENC MAE 0.44°, Pearson r=0.994, IMU-ENC bias −0.02° — all within the expected range. Trail percentage of 21.1% is consistent with GRV lag at 50 Hz.