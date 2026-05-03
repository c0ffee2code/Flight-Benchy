# Flight Analysis: 2026-05-02_19-52-27

## Run Identity

| Field | Value |
|-------|-------|
| Flight ID | 2026-05-02_19-52-27 |
| Duration | 119.9 s |
| Samples | 2284 |
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
2026-05-02_19-52-27          51.9   ok       NO          -         -       0.0s   119.9s
----------------------------------------------------------------------------------------
```

### profile_flight.py

```
Loaded 2026-05-02_19-52-27: 2284 samples, 119.9s

==========================================================
  2026-05-02_19-52-27
----------------------------------------------------------
  Angle PID: kp=3.0, ki=0.05, kd=0.3, iterm_limit=100.0
  Rate PID:  kp=0.5, ki=0.0, kd=0.009, iterm_limit=50.0
  IMU: angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz
  Motor: base=600, min=100, max=800

  --- Canonical KPIs (from score_flight) ---
  Reached setpoint                               NO
  T->setpoint (s)                                 -
  HoldMAE (°), post-reach                         -
  T@setpoint (s)                                0.0

  --- Sample Rate ---
  Samples                                      2284
  Duration (s)                                119.9
  Achieved Hz                                  19.0
  Mean dt (ms)                                 52.5
  Median dt (ms)                               51.0

  --- Sensor Health (IMU vs ENC) ---
  MAE (overall)                                1.25
  MAE (fast motion)                            1.36
  MAE (slow motion)                            1.20
  Max AE                                       5.91
  RMS Error                                    1.64
  Bias (IMU-ENC)                               1.16

  --- Setpoint Error (vs -15°) ---
  Whole-run ENC MAE (°)                       43.18
  (includes the rise — not comparable to HoldMAE)
  IMU MAE (°)                                 44.34
  ENC Bias (°)                                43.18
  ENC Max AE (°)                              66.86

  --- Correlation & Tracking ---
  Pearson r                                  0.9741
  IMU trails motion (%)                         0.0
  Encoder range (°)                            36.4
  IMU range (°)                                30.6

  --- Oscillation & Windup ---
  Oscillation freq (Hz)                        0.00
  Angle windup events                             0
  Angle windup threshold                       50.0
  Rate windup events                              0
  Rate windup threshold                        25.0
==========================================================
```

### Plot

`test_runs/flights/2026-05-02_19-52-27/plot.png` — open and review alongside this report.

---

## KPI Scorecard

| Metric | Value |
|--------|-------|
| Reached setpoint (−15°) | NO |
| T→setpoint (s) | — |
| HoldMAE (°), post-reach | — |
| T@setpoint (s) | 0.0 |

## Sensor Health

| Metric | Value |
|--------|-------|
| Achieved sample rate (Hz) | 19.0 |
| IMU-ENC MAE (°) | 1.25 |
| IMU-ENC bias (°) | 1.16 |
| Pearson r | 0.9741 |
| IMU trails motion (%) | 0.0 |

## Control Loop Health

| Metric | Value | Notes |
|--------|-------|-------|
| Oscillation frequency (Hz) | 0.00 | Computed from IMU-ENC error sign changes — not lever position oscillation |
| Whole-run ENC MAE (°) | 43.18 | vs −15° setpoint; includes the rise |
| ENC Bias (°) | 43.18 | vs −15° setpoint; always positive → encoder never crossed −15° |
| Encoder range (°) | 36.4 | Start ~+58°, minimum ~+21° — lever stopped ~36° short of setpoint |
| Angle windup events | 0 | At threshold ±50°/s (50% of iterm_limit=100) |
| Rate windup events | 0 | At threshold ±25°/s (50% of iterm_limit=50) |

## Observations

- **Setpoint not reached.** The encoder never entered the ±10° band around −15°. `ENC Bias = 43.18°` with `Whole-run ENC MAE = 43.18°` — the two being identical proves the encoder was always above −15° (no sample crossed the setpoint from above). With an encoder range of 36.4° and a start of ~+58°, the lever dropped to approximately +21° minimum and oscillated there for the full run. The target required 73° of travel from the restrictor; the lever achieved roughly half of that.

- **Control loop: P-term saturation blocking I-term accumulation.** Zero windup events despite the encoder sitting ~40° above the setpoint for over 100 seconds. At 40° error, `P = kp × 40 = 3.0 × 40 = 120°/s`, which nearly saturates the `angle_pid output_limit = 130`. Anti-windup suppresses I-term growth whenever the output is clamped — so the I-term never built up enough to appear in the windup counter, and the loop spent the entire run P-term-saturated at the equilibrium it found near +21°. The absence of windup events is a signature of anti-windup clamping, not small error.

- **Motor outputs.** Subplot 5 shows sustained asymmetry and oscillation between M1 and M2, consistent with the loop actively fighting the +20–30° equilibrium it settled into. The differential is not sufficient to drive the lever further toward −15°.

- **Sensor health is nominal.** IMU-ENC MAE 1.25°, Pearson r = 0.97, bias 1.16° — sensors are tracking correctly and are not a factor in the shortfall.

- **`osc_freq_hz = 0.00` does not mean no oscillation.** This metric counts sign changes of the IMU-ENC tracking error. Since IMU and ENC move coherently (r = 0.97, consistent 1.16° bias), the error has nearly constant sign throughout — hence zero sign changes. Subplot 1 clearly shows lever position oscillating; that oscillation is simply invisible to this metric because it is captured equally in both signals.
