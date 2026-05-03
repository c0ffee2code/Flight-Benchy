# Flight Analysis: 2026-05-02_15-25-22

> **DISCARDED — non-standard start.** Lever was at 4.9° (already within ±10° of horizontal). The reset-position step was skipped before this run. HoldMAE and oscillation frequency are not comparable to standard runs.

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_15-25-22 |
| Duration | 119.9 s |
| Samples | 2277 |
| Start angle | 4.9° |
| Standard start | NO — lever was not at the restrictor (expected 58.0° ±10.0°, got +4.9°). KPIs not comparable to standard runs. |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.5, ki=0.05, kd=0.5, iterm_limit=100.0 |
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
2026-05-02_15-25-22           4.9   !!      YES       0.0s      3.17     119.9s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_15-25-22

Non-standard start — lever was not at the restrictor (expected 58.0° ± 10.0°, got +4.9°). KPIs are not comparable to standard runs.
```

### profile_flight.py

```
Loaded 2026-05-02_15-25-22: 2277 samples, 119.9s

==========================================================
  2026-05-02_15-25-22
----------------------------------------------------------
  Angle PID: kp=3.5, ki=0.05, kd=0.5, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached horizontal                            YES
  T->0 (s)                                      0.0
  HoldMAE (°), post-reach                      3.17
  T@0 (s)                                     119.9

  --- Sample Rate ---
  Samples                                      2277
  Duration (s)                                119.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.7
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                0.24
  MAE (fast motion)                            0.32
  MAE (slow motion)                            0.20
  Max AE                                       1.47
  RMS Error                                    0.32
  Bias (IMU-ENC)                              -0.02

  --- Setpoint Error (vs 0°) ---
  Whole-run ENC MAE (°)                        3.17
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  3.22
  ENC Bias (°)                                 0.01
  ENC Max AE (°)                               8.79

  --- Correlation & Tracking ---
  Pearson r                                  0.9963
  IMU trails motion (%)                       100.0
  Encoder range (°)                            15.1
  IMU range (°)                                14.8

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        1.70  (ENC-based)
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_15-25-22/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | YES |
| T→0 (s) | 0.0 (started in band) |
| HoldMAE (°), post-reach | 3.17 |
| T@0 (s) | 119.9 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 0.24 |
| IMU-ENC bias (°) | −0.02 |
| Pearson r | 0.9963 |
| IMU trails motion (%) | 100.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 1.70 | ENC-based — pure hold, no rise phase |
| Whole-run ENC MAE (°) | 3.17 | Equals HoldMAE — run started in band, no rise |
| Angle windup events | 0 | |
| Rate windup events | 0 | |

## Observations

- **Non-standard start / discarded.** Lever was at 4.9° at run start — already within the ±10° hold band. T→0=0.0s and the full 119.9s is the hold window. Rise dynamics (overshoot, settle) are absent, so this run measures pure hold behaviour from horizontal, not the standard recovery + hold sequence. KPIs are not comparable to standard runs.

- **Hold accuracy:** HoldMAE of 3.17° is oscillation-dominated. ENC bias of 0.01° confirms the oscillation is symmetric around 0°. Encoder range of 15.1° means the lever swung over a ±7.5° window during hold. ENC Max AE of 8.79° is the largest single-sample excursion.

- **Oscillation:** 1.70 Hz (ENC-based) is a pure hold-phase measurement with no rise dynamics. The angle P-term in subplot 3 oscillates at the same frequency, confirming the outer loop as the source. No windup events on either loop.

- **Sensor health:** IMU-ENC MAE of 0.24° and bias of −0.02° are excellent. Pearson r=0.9963. IMU trailing 100% is an artefact of how the trailing metric is computed for a pure hold with no large unidirectional motion — there is no sustained angular velocity for the IMU to trail behind; the metric is not meaningful here.