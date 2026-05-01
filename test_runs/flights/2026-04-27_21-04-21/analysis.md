# Flight Analysis: 2026-04-27_21-04-21

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-04-27_21-04-21 |
| Duration | 70.2 s |
| Samples | 1301 |
| Start angle | 52.0° |
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
2026-04-27_21-04-21          52.0   ok      YES       3.9s      3.03      65.6s    70.2s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-04-27_21-04-21
```

### profile_flight.py

```
Loaded 2026-04-27_21-04-21: 1301 samples, 70.2s

==========================================================
  2026-04-27_21-04-21
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.006, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      3.9
  HoldMAE (°), post-reach                      3.03
  T@0 (s)                                      65.6

  --- Sample Rate ---
  Samples                                      1301
  Duration (s)                                 70.2
  Achieved Hz                                  18.5
  Mean dt (ms)                                 54.0
  Median dt (ms)                               52.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.97
  MAE (fast motion)                            0.95
  MAE (slow motion)                            0.99
  Max AE                                       4.31
  RMS Error                                    1.15
  Bias (IMU-ENC)                               0.78

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        3.99
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  4.24
  ENC Bias (°)                                 2.47
  ENC Max AE (°)                              52.03

  --- Correlation & Tracking ---
  Pearson r                                  0.9913
  IMU trails motion (%)                         7.7
  Encoder range (°)                            58.2
  IMU range (°)                                53.2

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.18  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-04-27_21-04-21/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 3.9 |
| HoldMAE (°), post-reach | 3.03 |
| T@0 (s) | 65.6 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.5 |
| IMU-ENC MAE (°) | 0.97 |
| IMU-ENC bias (°) | 0.78 |
| Pearson r | 0.9913 |
| IMU trails motion (%) | 7.7 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.18 | |
| Whole-run ENC MAE (°) | 3.99 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy** — HoldMAE of 3.03° is bias-dominated: the +2.47° encoder bias keeps the lever hovering above 0° throughout subplot 1, visible as both encoder and IMU lines sitting consistently above the setpoint. The 0.18 Hz oscillation adds a gentle slow wobble on top but is not the dominant error source.

- **Control loop** — Zero angle windup events is arithmetically expected: ki=0.05 × ~3° error × 50 Hz × 65s accumulates roughly 10 I-term units, well below the 50-unit threshold. The slow upward green I-term drift in subplot 3 is the correct response to the persistent offset — not a missing signal.

- **Motor balance** — M1 runs consistently higher than M2 during hold (subplot 5), providing the differential thrust that compensates for the positive encoder offset. Whether the source is mechanical resistance, wire tension, or motor/prop characteristics cannot be determined without thrust bench data.

- **Sensor health** — IMU-ENC bias of 0.78° is small and consistent, likely a residual tare or axis alignment offset. Pearson r of 0.9913 with 7.7% trail confirms faithful tracking; the 5° range gap (encoder 58.2° vs IMU 53.2°) is consistent with GRV filter smoothing at the extremes of motion.