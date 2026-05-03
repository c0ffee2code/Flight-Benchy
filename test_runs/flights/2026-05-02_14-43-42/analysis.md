# Flight Analysis: 2026-05-02_14-43-42

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_14-43-42 |
| Duration | 119.9 s |
| Samples | 2241 |
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
2026-05-02_14-43-42          51.9   ok      YES       5.1s      4.31     111.3s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_14-43-42
```

### profile_flight.py

```
Loaded 2026-05-02_14-43-42: 2241 samples, 119.9s

==========================================================
  2026-05-02_14-43-42
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      5.1
  HoldMAE (°), post-reach                      4.31
  T@0 (s)                                     111.3

  --- Sample Rate ---
  Samples                                      2241
  Duration (s)                                119.9
  Achieved Hz                                  18.7
  Mean dt (ms)                                 53.5
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.54
  MAE (fast motion)                            0.64
  MAE (slow motion)                            0.53
  Max AE                                       3.65
  RMS Error                                    0.76
  Bias (IMU-ENC)                               0.08

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        4.88
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  5.10
  ENC Bias (°)                                 0.75
  ENC Max AE (°)                              51.94

  --- Correlation & Tracking ---
  Pearson r                                  0.9943
  IMU trails motion (%)                        31.2
  Encoder range (°)                            64.0
  IMU range (°)                                60.3

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.80  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_14-43-42/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 5.1 |
| HoldMAE (°), post-reach | 4.31 |
| T@0 (s) | 111.3 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.7 |
| IMU-ENC MAE (°) | 0.54 |
| IMU-ENC bias (°) | 0.08 |
| Pearson r | 0.9943 |
| IMU trails motion (%) | 31.2 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.80 | ~1.25s period — visible in subplot 1 throughout hold |
| Whole-run ENC MAE (°) | 4.88 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy:** HoldMAE of 4.31° over 111.3s of hold time, oscillation-dominated. The 0.80 Hz oscillation (≈1.25s period) is the dominant contribution — visible in subplot 1 as persistent swings throughout the hold. ENC bias of 0.75° indicates a small positive steady-state offset (lever slightly M1-low during hold), with the angle I-term in subplot 3 slowly compensating.

- **Rise:** T→0 of 5.1s is notably slower than runs without pre-spin. Encoder range of 64° confirms the lever did not significantly overshoot past horizontal — the pre-spin at base_throttle before the control loop opened produced a gentler, more controlled rise with minimal overshoot, at the cost of a longer rise time.

- **Control loop:** Angle PID P-term in subplot 3 shows sustained oscillation through the hold at the 0.80 Hz frequency, confirming the oscillation originates at the outer loop. Zero windup events on both loops. Rate PID D-term in subplot 4 is active and noisier than in quieter runs, consistent with the outer loop feeding a high-frequency rate setpoint into the inner loop.

- **Sensor health:** IMU-ENC MAE of 0.54°, bias of 0.08°, and Pearson r=0.9943 are all healthy. IMU trailing at 31.2% — significantly lower than in prior high-overshoot runs — consistent with the gentler, lower-velocity dynamics this run exhibited throughout rise and hold.