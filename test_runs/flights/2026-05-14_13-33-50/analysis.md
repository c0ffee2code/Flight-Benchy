# Flight Analysis: 2026-05-14_13-33-50

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_13-33-50 |
| Duration | 119.9 s |
| Samples | 2292 |
| Start angle | 51.9° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
| motor | base=500.0, min=100.0, max=900.0 |
| feedforward_lead_ms | 15.0 |
| angle_report | game_rotation_vector @ 50 Hz |
| rate_report | calibrated_gyroscope @ 200 Hz |

---

## Raw Tool Output

### score_flight.py

```
Run                           Start  OK  Reached  T->SP (s)  HoldMAE_s (deg)  Dur (s)
-------------------------------------------------------------------------------------
2026-05-14_13-33-50        51.9deg  ok      YES      20.4s          2.28deg   119.9s
-------------------------------------------------------------------------------------

  Rise 10-90%: 20.8s     Overshoot: 12.2%     T_s (settling): 20.4s
  Damping ratio zeta: 0.557

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_13-33-50
```

### profile_flight.py

```
Loaded 2026-05-14_13-33-50: 2292 samples, 119.9s

==========================================================
  2026-05-14_13-33-50
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   20.4s
  Rise time 10-90% (s)                        20.8s
  Overshoot (% of step)                       12.2%
  Damping ratio zeta                          0.557
  Settling time T_s (s)                       20.4s
  HoldMAE_s (deg), post-settle              2.28deg

  --- Sample Rate ---
  Samples                                      2292
  Duration (s)                                119.9
  Achieved Hz                                  19.1
  Mean dt (ms)                                 52.3
  Median dt (ms)                               50.0
  dt_p99 (ms)                                  70.0
  dt_max (ms)                                  76.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                1.02
  MAE (fast motion)                            1.10
  MAE (slow motion)                            0.98
  Max AE                                       4.73
  RMS error                                    1.17
  Bias (IMU-ENC)                              -1.02
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          58.3
  IMU range (deg)                              54.1

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      4.86
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                     -0.73
  Hold std (deg)                               2.62
  Hold P95 |error| (deg)                       4.82
  Hold max |error| (deg)                       9.58
  Pearson r (hold window)                    0.9992
  FFT dominant freq (Hz)                      0.080

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      40.8
  RMS dM2/dt (throttle/s)                      40.8
  ANG_I mean (hold, deg/s)                     2.00
  M2-M1 mean (hold, throttle)                  -2.6
  I-term sign vs dM                      FLIP - sign error in control chain

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                    9.35

  --- Windup (whole-run)---
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_13-33-50/01_timeseries.png` — full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-05-14_13-33-50/02_step_response.png` — full run milestones + transient zoom
- `test_runs/flights/2026-05-14_13-33-50/03_spectrum.png` — PSD of hold-window error
- `test_runs/flights/2026-05-14_13-33-50/04_hold_error_distribution.png` — hold-error histogram
- `test_runs/flights/2026-05-14_13-33-50/05_phase_portrait.png` — phase portrait, time-coloured trajectory

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 20.4 |
| Rise time 10-90% (s) | 20.8 |
| Overshoot (% of step) | 12.2% |
| Damping ratio ζ | 0.557 |
| Settling time T_s (s) | 20.4 |
| HoldMAE_s (°), post-settle | 2.28 |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.3 |
| Median dt (ms) | 50.0 |
| dt_p99 (ms) | 70.0 |
| dt_max (ms) | 76.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 1.02 |
| MAE fast motion (°) | 1.10 |
| MAE slow motion (°) | 0.98 |
| Bias IMU-ENC (°) | −1.02 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | −0.73 |
| Hold std (°) | 2.62 |
| Hold P95 \|error\| (°) | 4.82 |
| Hold max \|error\| (°) | 9.58 |
| Pearson r (hold window) | 0.9992 |
| FFT dominant freq (Hz) | 0.080 |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 40.8 |
| RMS dM2/dt (throttle/s) | 40.8 |
| ANG_I mean (hold, deg/s) | 2.00 |
| M2−M1 mean (hold, throttle) | −2.6 |
| I-term sign vs ΔM | FLIP - sign error in control chain |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 9.35 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Hold quality**: HoldMAE_s of 2.28° is dominated by oscillation rather than steady-state offset — hold std (2.62°) is 3.6× larger than the signed bias (−0.73°). The FFT picks up a single dominant frequency at 0.080 Hz (≈12.5s period), putting this squarely in a slow low-frequency wobble regime. P95 of 4.82° means the lever drifts as far as ±4.82° on most excursions.

- **Transient response**: Rise time (20.8s) and settling time (20.4s) are nearly identical, indicating the system crept up slowly without meaningful overshoot phase — the 12.2% overshoot resolved immediately on crossing setpoint rather than producing a distinct ring-down. Damping ratio of 0.557 (underdamped) is consistent with the slow residual wobble persisting into the hold window.

- **Control effort**: No saturation on either bound throughout the run; throttle mean sits exactly at base (500). RMS dM/dt is symmetric at 40.8 throttle/s for both motors. The I-term sign diagnostic flags a FLIP — ANG_I mean of +2.00 deg/s is positive while M2−M1 mean is −2.6 (M1 running slightly higher). This sign inconsistency between the integral accumulation and the resulting motor differential warrants investigation in the control chain.

- **Inner loop**: Rate tracking RMS of 9.35°/s during hold is above the ~5°/s threshold for a well-settled hold, directly corresponding to the 0.080 Hz oscillation visible in the hold window.

- **Sensor health**: IMU-ENC bias of −1.02° (IMU reads consistently 1° lower than encoder) is stable across fast and slow motion (MAE 1.10° vs 0.98°). No trail detected (0.0%). Pearson r of 0.9992 confirms tight IMU-encoder coherence throughout the hold window.