# Flight Analysis: 2026-05-02_14-29-33

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_14-29-33 |
| Duration | 119.9 s |
| Samples | 2275 |
| Start angle | 52.9° |
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
2026-05-02_14-29-33          52.9   ok       NO          -         -       0.0s   119.9s
----------------------------------------------------------------------------------------

WARNING: POWER CUT — flat encoder (std=0.1° < 3.0°) with active motor differential (mean |M2-M1|=130). Pico was alive and commanding; ESC power was lost.
```

### profile_flight.py

```
(skipped — Reached = NO)
```

### Plot

`test_runs/flights/2026-05-02_14-29-33/plot.png` — open and review alongside this report.

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
| Whole-run ENC MAE (°) | ~53° | Lever held near start position throughout |
| Angle windup events | — | |
| Rate windup events | — | |

## Observations

- **Power cut.** `score_flight.py` flagged POWER CUT: encoder std=0.1° (lever stationary) with mean |M2-M1|=130. Subplot 1 shows encoder and IMU both flat at ~+50° for the full 119.9s; angle PID P-term saturated at ~163 deg/s (subplot 3); M1~540, M2~660 sustained throughout (subplot 5) — Pico and DShot pipeline alive, ESC motor power absent. Telemetry begins after the pre-spin ramp completes, so the brief spike at t≈0:00 in subplots 4 and 5 is the rate PID responding to the initial angle error at control loop start, not the ramp itself. ESC power was lost at or immediately after that moment.