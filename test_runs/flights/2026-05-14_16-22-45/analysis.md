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
| motor | base=500.0, min=100.0, max=900.0, expo=0.0 |
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
  Overshoot (% of step)                       26.2s
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

- `test_runs/flights/2026-05-14_16-22-45/01_timeseries.png`
- `test_runs/flights/2026-05-14_16-22-45/02_step_response.png`
- `test_runs/flights/2026-05-14_16-22-45/03_spectrum.png`
- `test_runs/flights/2026-05-14_16-22-45/04_hold_error_distribution.png`
- `test_runs/flights/2026-05-14_16-22-45/05_phase_portrait.png`

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint | YES |
| T→SP (s) | 26.1 |
| Rise time 10-90% (s) | 32.9 |
| Overshoot (% of step) | 13.0% |
| Damping ratio ζ | 0.544 |
| Settling time T_s (s) | 26.2 |
| HoldMAE_s (°), post-settle | 3.05 |

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
| Bias IMU-ENC (°) | −1.14 |
| IMU trails motion (%) | 0.0 |

## Hold-Window Tracking (post-reach)

| Metric | Value |
|--------|-------|
| Hold bias (°, signed) | −0.02 |
| Hold std (°) | 3.78 |
| Hold P95 \|error\| (°) | 9.40 |
| Hold max \|error\| (°) | 10.02 |
| Pearson r (hold window) | 0.9996 |
| FFT dominant freq (Hz) | 0.053 |

## Control Effort (hold window)

| Metric | Value |
|--------|-------|
| Mean throttle avg M1+M2 | 500.0 |
| Saturation upper % | 0.0 |
| Saturation lower % | 0.0 |
| RMS dM1/dt (throttle/s) | 41.5 |
| RMS dM2/dt (throttle/s) | 41.5 |
| ANG_I mean (hold, deg/s) | −0.08 |
| M2−M1 mean (hold, throttle) | −2.5 |
| I-term sign vs ΔM | N/A (P-term dominant) |

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

- **Overshoot and settling**: 13.0% overshoot with T_s = T→SP = 26.2s — no ring-down, immediate settle. Damping ratio ζ = 0.544. HoldMAE_s = 3.05° is the best fast-settle hold accuracy in this session. ANG_I mean = −0.08 deg/s confirms the integrator stays near-zero throughout; hold tracking is P-term-driven with zero integrator contribution.

- **Resolved hold oscillation**: FFT dominant freq = 0.053 Hz (period ≈ 19s), above the 3× resolution threshold (0.033 Hz) — this is a resolved, not advisory, frequency. Hold std = 3.78° and P95 = 9.40° are consistent with a low-amplitude 0.05 Hz wobble over the ~93s hold window. Hold max of 10.02° touches the ±10° settling band edge. HoldMAE_s = 3.05° reflects the mean level; the FFT shows a slow periodic component that the headline MAE underrepresents.

- **Timing clean**: dt_p99 = 70.1ms, dt_max = 77.0ms — best timing of any run at iterm_limit=5. Rate tracking RMS = 11.52°/s, consistent with iteration 5 (11.88°/s). No saturation events, symmetric motor differential.

- **Near-zero hold bias**: −0.02° — the lever is centred almost exactly on setpoint over the hold window. The M2−M1 mean of −2.5 throttle is the P-term correcting a small steady angular error rather than integrator accumulation.
