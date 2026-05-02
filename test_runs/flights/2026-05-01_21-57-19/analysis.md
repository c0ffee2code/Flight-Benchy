# Flight Analysis: 2026-05-01_21-57-19

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-01_21-57-19 |
| Duration | 107.6 s |
| Samples | 2040 |
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

**Note:** `motor.expo = 0.3` (first expo=0.3 run; second overall expo attempt — see DR-012 Amendment 2026-05-01)

---

## Raw Tool Output

### score_flight.py

```
Run                          Start  OK  Reached   T->0 (s)   HoldMAE    T@0 (s)  Dur (s)
----------------------------------------------------------------------------------------
2026-05-01_21-57-19          52.1   ok      YES      15.7s      3.56      89.6s   107.6s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-01_21-57-19
```

### profile_flight.py

```
Loaded 2026-05-01_21-57-19: 2040 samples, 107.6s

==========================================================
  2026-05-01_21-57-19
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.006, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                     15.7
  HoldMAE (°), post-reach                      3.56
  T@0 (s)                                      89.6

  --- Sample Rate ---
  Samples                                      2040
  Duration (s)                                107.6
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.8
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.78
  MAE (fast motion)                            0.79
  MAE (slow motion)                            0.79
  Max AE                                       4.59
  RMS Error                                    1.15
  Bias (IMU-ENC)                               0.36

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        5.74
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  6.11
  ENC Bias (°)                                 2.37
  ENC Max AE (°)                              52.12

  --- Correlation & Tracking ---
  Pearson r                                  0.9923
  IMU trails motion (%)                        20.0
  Encoder range (°)                            63.5
  IMU range (°)                                59.9

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.64  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-01_21-57-19/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 15.7 |
| HoldMAE (°), post-reach | 3.56 |
| T@0 (s) | 89.6 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 0.78 |
| IMU-ENC bias (°) | 0.36 |
| Pearson r | 0.9923 |
| IMU trails motion (%) | 20.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.64 | |
| Whole-run ENC MAE (°) | 5.74 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy** — HoldMAE is 3.56° with an encoder bias of 2.37°, meaning the error is bias-dominated rather than symmetric oscillation. The lever is hovering ~2.4° off horizontal on average, not wobbling around it. Oscillation frequency halved from the expo=0.0 run (0.64 Hz vs 1.38 Hz), consistent with reduced near-setpoint authority — but the lever settled at an offset rather than at 0°, which is not an improvement.

- **Recovery** — T→0 of 15.7s is the most significant regression: the previous best was 2.6s. The expo cubic shaping is most aggressive in the positive direction (authority_up=200 is small, so t=pid_output/200 reaches high values quickly and incurs greater cubic reduction). From a +52° start the lever must apply sustained positive corrections — exactly the direction expo attenuates hardest — explaining the slow crawl to horizontal visible in subplot 1.

- **Control loop** — Angle I-term builds during the 15.7s recovery to compensate for the expo-reduced P authority; by the time the lever reaches horizontal the integrator is carrying a load that overshoots zero before settling. No windup events, but the I-term is working harder than in the expo=0.0 run, which is consistent with reduced DC gain in the positive direction.

- **Sensor health** — IMU-ENC bias of 0.36° is elevated relative to the 0.02° in the expo=0.0 baseline. IMU trails motion 20% of the time (was 0%) — likely a consequence of the much slower dynamics during the 15.7s recovery, where the GRV filter has time to lag even moderate motion. Pearson r of 0.9923 remains excellent.

- **Motor balance** — M1/M2 appear broadly symmetric during hold in subplot 5, consistent with zero windup events and no sustained mechanical disturbance accumulating in the integrator.