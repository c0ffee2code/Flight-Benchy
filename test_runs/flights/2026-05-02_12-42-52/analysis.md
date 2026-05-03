# Flight Analysis: 2026-05-02_12-42-52

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_12-42-52 |
| Duration | 14.9 s |
| Samples | 285 |
| Start angle | 52.2° |
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
2026-05-02_12-42-52          52.2   ok      YES       5.8s      9.38       5.5s    14.9s
----------------------------------------------------------------------------------------
```

### profile_flight.py

```
==========================================================
  2026-05-02_12-42-52
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      5.8
  HoldMAE (°), post-reach                      9.38
  T@0 (s)                                       5.5

  --- Sample Rate ---
  Samples                                       285
  Duration (s)                                 14.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.5
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                1.67
  MAE (fast motion)                            1.98
  MAE (slow motion)                            2.15
  Max AE                                       5.14
  RMS Error                                    2.11
  Bias (IMU-ENC)                              -0.27

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                       16.63
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                 16.36
  ENC Bias (°)                                16.63
  ENC Max AE (°)                              52.82

  --- Correlation & Tracking ---
  Pearson r                                  0.9976
  IMU trails motion (%)                         0.0
  Encoder range (°)                            49.3
  IMU range (°)                                43.5

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.10  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_12-42-52/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 5.8 |
| HoldMAE (°), post-reach | 9.38 |
| T@0 (s) | 5.5 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 1.67 |
| IMU-ENC bias (°) | −0.27 |
| Pearson r | 0.9976 |
| IMU trails motion (%) | 0.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.10 | Very slow drift, ~10s period |
| Whole-run ENC MAE (°) | 16.63 | Includes the rise — not comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy:** HoldMAE of 9.38° is marginal — the lever is hovering just inside the ±10° band. T@0 of 5.5s out of ~9.1s post-reach (60%) confirms the lever spends significant time at the edge of the hold zone. The 0.10 Hz oscillation frequency (≈10s period) matches the slow drift visible in subplot 1, where the encoder wanders between approximately −5° and +10° rather than settling.

- **Rise:** T→0 of 5.8s is slower than prior runs from a full +58° start, consistent with the 52.2° start angle being 6° closer to horizontal — yet still took nearly 6 seconds, suggesting the angle PID P-term was not fully saturated for the full rise.

- **Control loop:** Zero windup events on both loops — I-terms are well-behaved at this gain set. Angle PID D-term is visible but small during the hold; rate PID D-term shows the higher-frequency noise characteristic of kd=0.009 on the inner loop.

- **Sensor health:** IMU-ENC bias of −0.27° and Pearson r=0.9976 indicate clean sensor agreement throughout the run. IMU trailing at 0.0% is consistent with the feedforward compensating GRV lag correctly.

- **Pipeline note:** First fully autonomous flight to reach horizontal — reset, deploy, fly, fetch, and analyse completed without manual intervention. Duration termination clean at 14.9s vs configured 15s.