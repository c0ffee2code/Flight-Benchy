# Flight Analysis: 2026-05-03_17-13-32

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-03_17-13-32 |
| Duration | 119.9 s |
| Samples | 2276 |
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
Run                          Start  OK  Reached  T->SP (s)   HoldMAE   T@SP (s)  Dur (s)
----------------------------------------------------------------------------------------
2026-05-03_17-13-32          51.9   ok      YES       1.3s      5.35     105.9s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-03_17-13-32
```

### profile_flight.py

```
Loaded 2026-05-03_17-13-32: 2276 samples, 119.9s

==========================================================
  2026-05-03_17-13-32
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->setpoint (s)                               1.3
  HoldMAE (°), post-reach                      5.35
  T@setpoint (s)                              105.9

  --- Sample Rate ---
  Samples                                      2276
  Duration (s)                                119.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.7
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                3.30
  MAE (fast motion)                            3.49
  MAE (slow motion)                            3.05
  Max AE                                       9.32
  RMS Error                                    3.84
  Bias (IMU-ENC)                              -3.29

  --- Setpoint Error (vs -15°) ---
  Whole-run ENC MAE (°)                        5.63
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  7.02
  ENC Bias (°)                                -1.12
  ENC Max AE (°)                              66.94

  --- Correlation & Tracking ---
  Pearson r                                  0.9642
  IMU trails motion (%)                        39.5
  Encoder range (°)                            80.1
  IMU range (°)                                81.5

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.04
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-03_17-13-32/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→setpoint (s) | 1.3 |
| HoldMAE (°), post-reach | 5.35 |
| T@setpoint (s) | 105.9 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 3.30 |
| IMU-ENC bias (°) | −3.29 |
| Pearson r | 0.9642 |
| IMU trails motion (%) | 39.5 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.04 | |
| Whole-run ENC MAE (°) | 5.63 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy** — HoldMAE of 5.35° is oscillation-dominated rather than bias-dominated: the ENC bias is only −1.12° (lever averages −16.1°, barely off setpoint), but the encoder sweeps ±3–5° around setpoint at 0.04 Hz, consistent with the slow wobble visible in subplot 1. The 0.04 Hz oscillation is too slow to be inner-loop noise and too persistent to be transient settling.

- **Sensor health** — IMU-ENC bias of −3.29° is large and systematic: the IMU reads the lever as roughly 3° lower than the encoder throughout the run. Because the outer angle loop uses IMU as its input, it controls to approximately −18.3° IMU-equivalent while the encoder sits near −15°; the resulting 1.12° encoder undershoot is the arithmetic consequence. Pearson r of 0.9642 is lower than a typical zero-setpoint run, likely because the large consistent bias reduces the correlation statistic even when the two signals are structurally in phase. IMU trail of 39.5% sits just below the loop-meltdown threshold (40%) — within normal range for a filtered sensor during dynamic phases.

- **Control loop** — Zero angle and rate windup events despite 105.9 s of hold confirm the I-term is not hitting limits. Angle I-term remaining near zero throughout (subplot 3) means the 1.12° encoder undershoot is not being corrected by integral action — the I-term threshold for 50% of iterm_limit is 50°, so sub-degree persistent offsets generate no windup events by design. Rate ki=0.0 so rate windup is structurally impossible.

- **Motor balance** — M1 runs consistently higher than M2 throughout the hold (subplot 5). With the lever sitting at approximately −16° (M2 end lower), M1 pushing harder is the differential required to counteract the gravitational component pulling M2 down — the asymmetry is directionally expected at this setpoint rather than indicative of a mechanical offset.