# Flight Analysis: 2026-05-01_10-38-16

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-01_10-38-16 |
| Duration | 129.6 s |
| Samples | 2448 |
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
2026-05-01_10-38-16          52.1   ok      YES       2.1s      1.97     127.4s   129.6s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-01_10-38-16
```

### profile_flight.py

```
Loaded 2026-05-01_10-38-16: 2448 samples, 129.6s

==========================================================
  2026-05-01_10-38-16
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.006, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      2.1
  HoldMAE (°), post-reach                      1.97
  T@0 (s)                                     127.4

  --- Sample Rate ---
  Samples                                      2448
  Duration (s)                                129.6
  Achieved Hz                                  18.9
  Mean dt (ms)                                 53.0
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.27
  MAE (fast motion)                            0.42
  MAE (slow motion)                            0.22
  Max AE                                       4.66
  RMS Error                                    0.46
  Bias (IMU-ENC)                               0.05

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        2.28
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  2.22
  ENC Bias (°)                                -0.59
  ENC Max AE (°)                              52.12

  --- Correlation & Tracking ---
  Pearson r                                  0.9946
  IMU trails motion (%)                         0.0
  Encoder range (°)                            60.2
  IMU range (°)                                55.2

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        1.53  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-01_10-38-16/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 2.1 |
| HoldMAE (°), post-reach | 1.97 |
| T@0 (s) | 127.4 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.9 |
| IMU-ENC MAE (°) | 0.27 |
| IMU-ENC bias (°) | 0.05 |
| Pearson r | 0.9946 |
| IMU trails motion (%) | 0.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 1.53 | |
| Whole-run ENC MAE (°) | 2.28 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy** — HoldMAE of 1.97° is a meaningful improvement over the post-rebuild baseline of 3.03° (run 2026-04-27_21-04-21). The 1.53 Hz oscillation frequency matches the low-frequency wobble visible in subplot 1; the error is oscillation-dominated rather than bias-dominated (ENC bias is only −0.59°, which is small relative to the ±2° swing visible during hold).

- **Control loop** — Angle I-term builds during the rise then stabilises at a moderate positive value and holds there for the full 127 s, which is the expected behaviour for a persistent steady-state disturbance (wire tension). Zero windup events against a 100-unit limit confirms the I-term is well within authority and not saturating.

- **Motor balance** — M1 runs consistently higher than M2 throughout the hold (visually ~25–30 throttle units in subplot 5). This is the I-term compensation for the same steady-state disturbance noted above; without thrust bench data the per-unit contribution cannot be separated from mechanical bias.

- **Sensor health** — IMU-ENC MAE of 0.27° and bias of 0.05° are excellent; both sensors agree closely across the full run. Pearson r of 0.9946 and 0.0% trail confirm the GRV is tracking the encoder with no detectable lag at this bandwidth. The IMU range (55.2°) reads ~5° narrower than the encoder range (60.2°), consistent with mild GRV smoothing at the outer extremes of the rise — not a concern during hold.