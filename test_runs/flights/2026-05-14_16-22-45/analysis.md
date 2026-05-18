# Flight Analysis: 2026-05-14_16-22-45

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-14_16-22-45 |
| Duration | 119.9 s |
| Samples | 2289 |
| Start angle | 51.9° |
| Standard start | YES |

## Config Snapshot

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=3.0, ki=0.05, kd=0.5, iterm_limit=5.0 |
| rate_pid | kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0 |
| motor | base=500, min=100, max=900 |
| feedforward_lead_ms | 15.0 |
| angle_report | game_rotation_vector @ 50 Hz |
| rate_report | calibrated_gyroscope @ 200 Hz |

---

## Raw Tool Output

### score_flight.py

```
Run                           Start  OK  Reached  T->SP (s)  HoldMAE_s (deg)  Dur (s)
-------------------------------------------------------------------------------------
2026-05-14_16-22-45        51.9deg  ok      YES      26.1s          3.05deg   119.9s
-------------------------------------------------------------------------------------

  Acceptance levels (tolerance: +/-10 deg):
    hold_mae_deg             3.05  ->  good
    time_to_sp_s            26.06  ->  below_pass
    settling_time_s         26.16  ->  pass
    hold_duration_s         93.73  ->  good
    overshoot_pct           13.03  ->  good

  Rise 10-90%: 32.9s     Overshoot: 13.0%     T_s (settling): 26.2s
  Damping ratio zeta: 0.544

Passed - use profile_flight.py for deep dive:
  python .claude/skills/analyse-flight/scripts/profile_flight.py test_runs\flights\2026-05-14_16-22-45
```

### profile_flight.py

```
Loaded 2026-05-14_16-22-45: 2289 samples, 119.9s

==========================================================
  2026-05-14_16-22-45
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.5, iterm_limit=5.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=500.0, min=100.0, max=900.0
  Feedforward: lead_ms=15.0

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                              YES
  T->SP (s)                                   26.1s
  Rise time 10-90% (s)                        32.9s
  Overshoot (% of step)                       13.0%
  Damping ratio zeta                          0.544
  Settling time T_s (s)                       26.2s
  HoldMAE_s (deg), post-settle              3.05deg

  --- Sample Rate ---
  Samples                                      2289
  Duration (s)                                119.9
  Achieved Hz                                  19.1
  Mean dt (ms)                                 52.4
  Median dt (ms)                               51.0
  dt_p99 (ms)                                  70.1
  dt_max (ms)                                  77.0

  --- Sensor Health (IMU vs ENC, whole-run) ---
  MAE (overall)                                1.14
  MAE (fast motion)                            1.28
  MAE (slow motion)                            1.09
  Max AE                                       4.78
  RMS error                                    1.30
  Bias (IMU-ENC)                              -1.14
  IMU trails motion (%)                         0.0
  Encoder range (deg)                          58.7
  IMU range (deg)                              54.4

  --- Hold-Window Tracking (ENC vs +0deg, post-reach) ---
  Whole-run ENC MAE (deg)                      6.17
  (includes rise - not comparable to HoldMAE_s)
  Hold bias (deg, signed)                     -0.02
  Hold std (deg)                               3.78
  Hold P95 |error| (deg)                       9.40
  Hold max |error| (deg)                      10.02
  Pearson r (hold window)                    0.9996
  FFT dominant freq (Hz)                      0.053
    (freq resolution Hz)                      0.011

  --- Control Effort (hold window) ---
  Mean throttle (avg M1+M2)                   500.0
  RMS throttle                                500.0
  Saturation upper % (>= max)                   0.0
  Saturation lower % (<= min)                   0.0
  RMS dM1/dt (throttle/s)                      41.5
  RMS dM2/dt (throttle/s)                      41.5
  ANG_I mean (hold, deg/s)                    -0.08
  M2-M1 mean (hold, throttle)                  -2.5
  I-term sign vs dM                      N/A (P-term dominant)

  --- Inner Loop (hold window) ---
  Rate tracking RMS (deg/s)                   11.52

  --- Windup (whole-run) ---
  Angle windup events                             0
  Angle windup threshold                        2.5
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plots

- `test_runs/flights/2026-05-14_16-22-45/01_timeseries.png` — full time-series (angle, rate, PID terms, motors)
- `test_runs/flights/2026-05-14_16-22-45/02_step_response.png` — full run milestones + transient zoom
- `test_runs/flights/2026-05-14_16-22-45/03_spectrum.png` — PSD of hold-window error
- `test_runs/flights/2026-05-14_16-22-45/04_hold_error_distribution.png` — hold-error histogram
- `test_runs/flights/2026-05-14_16-22-45/05_phase_portrait.png` — phase portrait, time-coloured trajectory

---

## KPI Scorecard

| Metric | Value | Level |
|--------|-------|-------|
| Reached setpoint | YES | — |
| T->SP (s) | 26.1 | below_pass |
| Rise time 10-90% (s) | 32.9 | — |
| Overshoot (% of step) | 13.0% | good |
| Damping ratio zeta | 0.544 | — |
| Settling time T_s (s) | 26.2 | pass |
| Hold Duration (s) | 93.7 | good |
| HoldMAE_s (°), post-settle | 3.05 | good |

## Sample Rate

| Metric | Value |
|--------|-------|
| Achieved Hz | 19.1 |
| Mean dt (ms) | 52.4 |
| Median dt (ms) | 51.0 |
| dt_p99 (ms) | 70.1 |
| dt_max (ms) | 77.0 |

## Sensor Health (IMU vs ENC, whole-run)

| Metric | Value |
|--------|-------|
| MAE overall (°) | 1.14 |
| MAE fast motion (°) | 1.28 |
| MAE slow motion (°) | 1.09 |
| Bias IMU-ENC (°) | -1.14 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | -0.02 |
| Hold std (°) | 3.78 |
| Hold P95 \|error\| (°) | 9.40 |
| Hold max \|error\| (°) | 10.02 |
| Pearson r (hold window) | 0.9996 |
| FFT dominant freq (Hz) | 0.053 |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % (>= throttle_max) | 0.0 |
| Saturation lower % (<= throttle_min) | 0.0 |
| RMS dM1/dt (throttle/s) | 41.5 |
| RMS dM2/dt (throttle/s) | 41.5 |
| ANG_I mean (hold, deg/s) | -0.08 |
| M2-M1 mean (hold, throttle) | -2.5 |
| I-term sign vs dM | N/A (P-term dominant) |

## Inner Loop (hold window)

| Metric | Value |
|--------|-------|
| Rate tracking RMS (°/s) | 11.52 |

## Windup (whole-run)

| Metric | Value |
|--------|-------|
| Angle windup events | 0 |
| Rate windup events | 0 |

---

## Observations

- **Transient response:** T->SP of 26.1 s is below the 10 s pass threshold, the slowest approach in this session. Rise time 10-90% of 32.9 s confirms a very gradual climb. T_s of 26.2 s is 0.1 s after T->SP, indicating the lever crossed the band boundary and immediately confirmed settled hold with no prior bounce. Overshoot of 13.0% is good.

- **Hold quality:** HoldMAE_s 3.05° with bias -0.02° and std 3.78° — the hold is almost perfectly centered (bias negligible) but oscillation-dominated. P95 of 9.40° and max of 10.02° show the lever is right at the edge of the band on peak excursions, just touching the boundary. FFT dominant frequency of 0.053 Hz (one cycle ~19 s) is a clearly resolved slow oscillation, well above the 3x resolution threshold of 0.033 Hz.

- **Control effort:** Mean throttle at base (500.0), zero saturation, and perfectly symmetric motors (RMS dM1/dt = RMS dM2/dt = 41.5 throttle/s). ANG_I mean of -0.08 deg/s and M2-M1 mean of -2.5 throttle units indicate negligible steady-state load. The controller is operating entirely in P-term dominant mode throughout the hold.

- **Sensor health:** IMU-ENC bias of -1.14° matches 2026-05-14_16-16-07 exactly. The slightly higher fast-motion MAE (1.28° vs slow 1.09°) is consistent with a small lag component on top of the fixed offset. IMU trails at 0.0% and Pearson r 0.9996 confirm good sensor agreement.
