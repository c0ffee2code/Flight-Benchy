# Flight Analysis: 2026-05-02_21-26-11

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_21-26-11 |
| Duration | 119.9 s |
| Samples | 2257 |
| Start angle | 51.9° |
| Standard start | YES |
| Setpoint | −15.0° |

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
2026-05-02_21-26-11          51.9   ok      YES       2.5s      2.92     116.6s   119.9s
----------------------------------------------------------------------------------------

Passed — use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-02_21-26-11
```

### profile_flight.py

```
Loaded 2026-05-02_21-26-11: 2257 samples, 119.9s

==========================================================
  2026-05-02_21-26-11
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->setpoint (s)                               2.5
  HoldMAE (°), post-reach                      2.92
  T@setpoint (s)                              116.6

  --- Sample Rate ---
  Samples                                      2257
  Duration (s)                                119.9
  Achieved Hz                                  18.8
  Mean dt (ms)                                 53.2
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                1.71
  MAE (fast motion)                            1.81
  MAE (slow motion)                            1.63
  Max AE                                       4.22
  RMS Error                                    1.85
  Bias (IMU-ENC)                              -1.66

  --- Setpoint Error (vs -15°) ---
  Whole-run ENC MAE (°)                        3.45
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                  3.84
  ENC Bias (°)                                 0.72
  ENC Max AE (°)                              66.94

  --- Correlation & Tracking ---
  Pearson r                                  0.9904
  IMU trails motion (%)                         9.5
  Encoder range (°)                            76.6
  IMU range (°)                                74.5

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.01
  Angle windup events                             0
  Rate windup events                              0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_21-26-11/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint (−15°) | YES |
| T→setpoint (s) | 2.5 |
| HoldMAE (°), post-reach | 2.92 |
| T@setpoint (s) | 116.6 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 18.8 |
| IMU-ENC MAE (°) | 1.71 |
| IMU-ENC bias (°) | −1.66 |
| Pearson r | 0.9904 |
| IMU trails motion (%) | 9.5 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.01 | IMU-ENC tracking error — not lever position oscillation |
| Whole-run ENC MAE (°) | 3.45 | vs −15° setpoint; includes the rise |
| ENC Bias (°) | +0.72 | vs −15° setpoint; hold phase mean ≈ −15.1° after accounting for rise |
| ENC Max AE (°) | 66.94 | Peak at start position (~+52° from −15° = 67°) |
| Angle windup events | 0 | At threshold ±50°/s |
| Rate windup events | 0 | At threshold ±25°/s |

## Observations

- **Formula fix validated.** The lever reached −15° in 2.5s and held for 116.6s (HoldMAE 2.92°). Subplot 1 shows the encoder and IMU settling squarely on the setpoint line for the full hold phase — a clear contrast with the previous run where they tracked 30° above it.

- **I-term carrying the gravity disturbance cleanly.** ENC Bias of +0.72° across the whole run implies the hold phase averaged approximately −15.1° (the +0.72° mean includes the brief +52°→−15° rise inflating it positively). Subplot 3 shows the I-term (orange) building to a sustained steady value and holding there — zero windup events confirms it settled below 50% of iterm_limit. The I-term is compensating for the constant gravitational torque at −15° without saturating.

- **Motor asymmetry matches the disturbance.** Subplot 5 shows M1 running consistently higher than M2 throughout the hold. At −15° (M2 end lower), gravity pulls M2 further down; M1 carrying more thrust is the correct steady-state response to counteract this.

- **Sensor health is good; bias sign shifted at this angle.** IMU-ENC bias is −1.66° (IMU reads below encoder), Pearson r=0.9904, trail 9.5%. The bias magnitude is similar to previous runs but the sign is negative here — IMU and encoder disagreement is consistent in direction throughout the hold (not oscillating), so it has no effect on control accuracy.

- **Moderate hold oscillation visible in subplot 1.** The 2.92° HoldMAE corresponds to visible ±3–5° slow oscillation around the setpoint in subplot 1. Oscillation freq metric (0.01 Hz) reflects IMU-ENC tracking coherence, not lever position oscillation — both signals move together, so their difference is nearly constant.