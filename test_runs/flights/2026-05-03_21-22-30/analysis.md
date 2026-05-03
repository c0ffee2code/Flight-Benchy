# Flight Analysis: 2026-05-03_21-22-30

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-03_21-22-30 |
| Duration | 119.9 s |
| Samples | 2249 |
| Start angle | 51.9° |
| Standard start | Yes |

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
Run                          Start  OK  Reached  T->SP (s)   HoldMAE   T@SP (s)  Dur (s)
----------------------------------------------------------------------------------------
2026-05-03_21-22-30          51.9   ok      YES       2.3s      4.15     111.7s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-03_21-22-30
```

### profile_flight.py

```
Loaded 2026-05-03_21-22-30: 2249 samples, 119.9s

==========================================================
  2026-05-03_21-22-30
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->setpoint (s)                               2.3
  HoldMAE (°), post-reach                      4.15
  T@setpoint (s)                              111.7

  --- Sample Rate ---
  Samples                                      2249
  Duration (s)                                119.9
  Achieved Hz                                  18.7
  Mean dt (ms)                                 53.3
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.52
  MAE (fast motion)                            0.72
  MAE (slow motion)                            0.44
  Max AE                                       4.18
  RMS Error                                    0.74
  Bias (IMU-ENC)                              -0.27

  --- Setpoint Error (vs +0°) ---
  Whole-run ENC MAE (°)                        4.50
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  4.67
  ENC Bias (°)                                -2.16
  ENC Max AE (°)                              51.86

  --- Correlation & Tracking ---
  Pearson r                                  0.9928
  IMU trails motion (%)                        39.1
  Encoder range (°)                            66.4
  IMU range (°)                                62.1

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.83
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-03_21-22-30/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 2.3 |
| HoldMAE (°), post-reach | 4.15 |
| T@SP (s) | 111.7 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.7 |
| IMU-ENC MAE (°) | 0.52 |
| IMU-ENC bias (°) | -0.27 |
| Pearson r | 0.9928 |
| IMU trails motion (%) | 39.1 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.83 | |
| Whole-run ENC MAE (°) | 4.50 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy**: HoldMAE of 4.15° has two components visible in subplot 1: a consistent negative bias (ENC bias = −2.16°, lever hovering ~2° below setpoint) and a low-frequency oscillation at 0.83 Hz producing a regular symmetric wobble. Roughly half the error is from the offset, half from the oscillation amplitude.

- **Control loop**: The angle I-term in subplot 3 trends negative over the hold, accumulating to compensate for the persistent negative bias — this is expected behaviour with ki=0.05. Zero windup events against the 100.0 limit confirms the I-term never saturated; the slow drift is well within its operating range. Rate ki=0.0, so no I-term accumulation in the inner loop, as expected.

- **Motor balance**: M2 runs consistently higher than M1 throughout the hold (subplot 5, orange above blue by roughly 50–100 throttle units). This is the I-term compensation for the negative angle bias — to hold the lever at ~−2°, M2 requires elevated thrust relative to M1.

- **Sensor health**: IMU-ENC MAE of 0.52° and bias of −0.27° are small and consistent. Pearson r=0.9928 indicates clean correlation across the full run. IMU trailing motion 39.1% of samples is consistent with the known GRV filter lag at 50 Hz.