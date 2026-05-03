# Flight Analysis: 2026-05-02_12-28-31

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_12-28-31 |
| Duration | 14.9 s |
| Samples | 284 |
| Start angle | 52.1° |
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
2026-05-02_12-28-31          52.1   ok       NO          -         -       0.0s    14.9s
----------------------------------------------------------------------------------------
```

### profile_flight.py

```
(skipped — Reached = NO)
```

### Plot

`test_runs/flights/2026-05-02_12-28-31/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached horizontal | NO |
| T→0 (s) | — |
| HoldMAE (°), post-reach | — |
| T@0 (s) | 0.0 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | — |
| IMU-ENC MAE (°) | — |
| IMU-ENC bias (°) | — |
| Pearson r | — |
| IMU trails motion (%) | — |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | — | |
| Whole-run ENC MAE (°) | ~52° | Lever held near start position throughout |
| Angle windup events | — | |
| Rate windup events | — | |

## Observations

- **Did not reach horizontal.** Encoder held at ~52° for the entire 14.9s run — no meaningful progress toward 0°. The single dynamic event at t≈2s (brief gyro spike, rate D-term excursion, motor flutter visible in subplot 5) is the only departure from steady state; the lever returned to ~52° immediately after.

- **Control loop:** Angle PID P-term saturated at the output limit (~163 deg/s, kp=3.5 × 46° error = 161) for virtually the entire run, correctly commanding maximum rate setpoint. Rate PID P-term held steady at ~65 throttle units (rate_kp=0.5 × rate_sp≈130), producing M1≈537, M2≈660 — a differential of ~123 throttle units in the correct direction (M2 higher to push M2-end down and raise the encoder angle toward 0°). The loop is commanding correctly; the lever is not responding.

- **Duration termination:** Actual duration 14.9s against configured 15s (99.3%) — clean exit, no early termination anomaly.

- **Pipeline note:** This is the first fully autonomous flight (reset → deploy → run → fetch → analyse). The pipeline executed end-to-end without manual intervention.