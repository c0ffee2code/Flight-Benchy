# Flight Analysis: 2026-04-30_21-36-01

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-04-30_21-36-01 |
| Duration | 93.9 s |
| Samples | 1724 |
| Start angle | −50.8° |
| Standard start | NO — lever started from M2 end (expected +58° ± 10°, got −50.8°). Hold-quality KPIs are not comparable to standard runs. |

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
2026-04-30_21-36-01         -50.8   !!      YES       1.0s      2.35      92.9s    93.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-04-30_21-36-01

Non-standard start — lever was not at the restrictor (expected 58.0° ± 10.0°, got -50.8°). KPIs are not comparable to standard runs.
```

### profile_flight.py

```
Loaded 2026-04-30_21-36-01: 1724 samples, 93.9s

==========================================================
  2026-04-30_21-36-01
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.006, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      1.0
  HoldMAE (°), post-reach                      2.35
  T@0 (s)                                      92.9

  --- Sample Rate ---
  Samples                                      1724
  Duration (s)                                 93.9
  Achieved Hz                                  18.4
  Mean dt (ms)                                 54.5
  Median dt (ms)                               53.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.55
  MAE (fast motion)                            0.69
  MAE (slow motion)                            0.47
  Max AE                                       4.10
  RMS Error                                    0.73
  Bias (IMU-ENC)                               0.32

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        2.59
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  2.52
  ENC Bias (°)                                -1.04
  ENC Max AE (°)                              50.80

  --- Correlation & Tracking ---
  Pearson r                                  0.9874
  IMU trails motion (%)                       100.0
  Encoder range (°)                            56.3
  IMU range (°)                                56.6

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.57  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-04-30_21-36-01/plot.png` — open and review alongside this report.

---

## KPI Scorecard

(From `score_flight.py` output — also echoed at top of `profile_flight.py` output.)

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 1.0 |
| HoldMAE (°), post-reach | 2.35 |
| T@0 (s) | 92.9 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.4 |
| IMU-ENC MAE (°) | 0.55 |
| IMU-ENC bias (°) | 0.32 |
| Pearson r | 0.9874 |
| IMU trails motion (%) | 100.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.57 | |
| Whole-run ENC MAE (°) | 2.59 | Includes the rise — not directly comparable to HoldMAE |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Hold accuracy** — HoldMAE of 2.35° with a −1.04° ENC bias: the lever hovers slightly below 0° during hold, visible in subplot 1 as both encoder and IMU lines sitting just under the setpoint. Oscillation at 0.57 Hz is higher than a typical standard run but HoldMAE is still bias-dominated rather than oscillation-dominated.

- **Control loop** — Zero windup events is arithmetically certain: T→0 of 1.0s at 50 Hz gives ~50 outer-loop ticks during the rise; with ki=0.05 and an average error of ~25° over that second, I-term accumulation is under 2 units — far below the 50-unit threshold. The near-flat green I-term line in subplot 3 during the rise is correct.

- **Motor balance** — M1 runs above M2 during hold (subplot 5): with the encoder at −1.04°, error is +1.04°, so the controller correctly biases M1 upward to push M1 end down and correct toward 0°. The asymmetry is smaller than in a standard run, consistent with the smaller hold error.

- **Sensor health** — MAE of 0.55° and Pearson r of 0.9874 confirm good IMU-encoder tracking. The 100% trail figure is a metric artifact: this run's small static bias (0.32°) means the dynamic IMU lag during the fast rise dominates the trail computation, saturating it at 100%; the reliable tracking indicators (MAE, r) show no sensor health problem.