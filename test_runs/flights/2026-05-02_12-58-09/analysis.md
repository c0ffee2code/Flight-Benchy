# Flight Analysis: 2026-05-02_12-58-09

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_12-58-09 |
| Duration | 119.9 s |
| Samples | 2280 |
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
2026-05-02_12-58-09          52.2   ok      YES       2.2s      2.35     117.1s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_12-58-09
```

### profile_flight.py

```
Loaded 2026-05-02_12-58-09: 2280 samples, 119.9s

==========================================================
  2026-05-02_12-58-09
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      2.2
  HoldMAE (°), post-reach                      2.35
  T@0 (s)                                     117.1

  --- Sample Rate ---
  Samples                                      2280
  Duration (s)                                119.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.6
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.41
  MAE (fast motion)                            0.57
  MAE (slow motion)                            0.47
  Max AE                                      25.46
  RMS Error                                    1.04
  Bias (IMU-ENC)                               0.01

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        3.10
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  3.03
  ENC Bias (°)                                -0.19
  ENC Max AE (°)                              52.82

  --- Correlation & Tracking ---
  Pearson r                                  0.9900
  IMU trails motion (%)                        90.0
  Encoder range (°)                           105.0
  IMU range (°)                                91.6

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.95  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_12-58-09/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 2.2 |
| HoldMAE (°), post-reach | 2.35 |
| T@0 (s) | 117.1 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 0.41 |
| IMU-ENC bias (°) | 0.01 |
| Pearson r | 0.9900 |
| IMU trails motion (%) | 90.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.95 | ≈1s period — visible low-frequency wobble in subplot 1 |
| Whole-run ENC MAE (°) | 3.10 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy:** HoldMAE of 2.35° over 117.1s of hold time is oscillation-dominated, not bias-dominated — ENC bias of −0.19° is negligible, meaning the lever has no meaningful steady-state offset. The 0.95 Hz oscillation (≈1.05s period) is visible in subplot 1 as a regular small-amplitude symmetric wobble around 0°. T@0 of 117.1s out of 119.9s (97.7%) confirms the lever holds inside ±10° for virtually the entire run.

- **Control loop:** Angle I-term shows a small sustained buildup in subplot 3, correctly compensating the −0.19° residual bias. Zero windup events on both loops — I-term limits are comfortably above the operating range at this gain set. Rate PID D-term in subplot 4 shows the high-frequency noise characteristic of kd=0.009 on the inner loop; it is present but not driving the 0.95 Hz hold oscillation, which has encoder range 105.0° vs IMU range 91.6° — the GRV filter smooths out the fast encoder motion.

- **Sensor health:** IMU-ENC MAE of 0.41° and bias of 0.01° indicate excellent sensor agreement and an essentially perfect tare. IMU trailing motion at 90.0% is notably high: during the 0.95 Hz hold oscillation the GRV consistently lags the encoder in the direction of motion, consistent with the GRV filter's inherent smoothing at slow oscillation frequencies. Max AE of 25.46° originates from the initial rise phase as the encoder sweeps through +52° → 0° faster than the GRV can track; it is not present during the hold.

- **Motor balance:** M1 and M2 hold near base_throttle=600 during the hold in subplot 5, with small differentials driven by the control output tracking the 0.95 Hz oscillation. No large sustained asymmetry is visible, consistent with the near-zero ENC bias.