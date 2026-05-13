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
Run                          Start  OK  Reached  T->SP (s)  HoldMAE_s (°)  Dur (s)
----------------------------------------------------------------------------------
2026-05-03_21-22-30          51.9°  ok      YES       2.3s          3.93°   119.9s
----------------------------------------------------------------------------------

  Rise 10-90%: 2.9s      Overshoot: 28.0%     T_s (settling): 103.7s

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
  Feedforward: lead_ms=15

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                    2.3s
  Rise time 10-90% (s)                         2.9s
  Overshoot (% of step)                       28.0%
  Settling time T_s (s)                      103.7s
  HoldMAE_s (°), post-settle                  3.93°

  --- Sample Rate ---
  Samples                                      2249
  Duration (s)                                119.9
  Achieved Hz                                  18.7
  Mean dt (ms)                                 53.3
  Median dt (ms)                               51.0
  dt_p99 (ms)                                  96.0
  dt_max (ms)                                 115.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                0.52
  MAE (fast motion)                            0.72
  MAE (slow motion)                            0.44
  Max AE                                       4.18
  RMS error                                    0.74
  Bias (IMU-ENC)                              -0.27
  IMU trails motion (%)                        39.1
  Encoder range (°)                            66.4
  IMU range (°)                                62.1

  --- Hold-Window Tracking (ENC vs +0°, post-reach) ---
  Whole-run ENC MAE (°)                        4.50
  (includes rise — not comparable to HoldMAE_s)
  Hold bias (°, signed)                       -2.63
  Hold std (°)                                 4.50
  Hold P95 |error| (°)                         9.91
  Hold max |error| (°)                        14.50
  Pearson r (hold window)                    0.9896
  FFT dominant freq (Hz)                      0.008
    (freq resolution Hz)                      0.008

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   600.0
  RMS throttle                                600.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      96.1
  RMS dM2/dt (throttle/s)                      96.1

  --- Inner Loop (hold window) ---
  Rate tracking RMS (°/s)                     19.35

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-03_21-22-30/plot.png` — open alongside this report if needed.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | Yes |
| T→SP (s) | 2.3 |
| Rise time 10-90% (s) | 2.9 |
| Overshoot (% of step) | 28.0% |
| Settling time T_s (s) | 103.7 |
| HoldMAE_s (°), post-settle | 3.93 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 18.7 |
| Mean dt (ms) | 53.3 |
| Median dt (ms) | 51.0 |
| dt_p99 (ms) | 96.0 |
| dt_max (ms) | 115.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 0.52 |
| MAE fast motion (°) | 0.72 |
| MAE slow motion (°) | 0.44 |
| Bias IMU-ENC (°) | -0.27 |
| IMU trails motion (%) | 39.1 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | -2.63 |
| Hold std (°) | 4.50 |
| Hold P95 \|error\| (°) | 9.91 |
| Hold max \|error\| (°) | 14.50 |
| Pearson r (hold window) | 0.9896 |
| FFT dominant freq (Hz) | 0.008 |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 600.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 96.1 |
| RMS dM2/dt (throttle/s) | 96.1 |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 19.35 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Hold quality**: Post-reach hold std (4.50°) substantially exceeds bias (−2.63°), indicating oscillation-dominated error rather than a steady-state offset. Settling was not confirmed until 103.7s — meaning the 101s between first reach (2.3s) and confirmed settling was spent in persistent oscillation that repeatedly left and re-entered the ±10° band without sustaining the 5s criterion. HoldMAE_s of 3.93° covers only the final 16.2s of the run.

- **FFT dominant frequency**: 0.008 Hz equals the frequency resolution of the FFT window — this cannot be interpreted as a confirmed oscillation period. It indicates that the lowest non-DC bin carries the most energy, consistent with a very slow within-band drift rather than a resolved frequency.

- **Transient response**: 28% overshoot on the 51.9° step corresponds to ~14.5°, sending the lever to roughly −14.5°. Rise time of 2.9s is moderate. The combination of significant overshoot and very late settling (103.7s) shows the ring-down after the overshoot is the limiting factor for this run — the hold window between 2.3s and 103.7s is dominated by recovery.

- **Control effort**: Mean throttle is exactly 600 (base_throttle) and dM1/dt = dM2/dt = 96.1 throttle/s — perfectly symmetric, no saturation at either limit. The high symmetric dM rate reflects the ongoing oscillation, not chattering against a boundary. No steady disturbance is present.

- **Inner loop**: Rate tracking RMS of 19.35 °/s during the hold window is high for a hold phase, consistent with the oscillating hold — the inner loop is servicing large angular rate demands rather than tracking a small residual around setpoint.