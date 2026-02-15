# ADR-007: Motor Thrust Characterization Test Bench (Future)

## Status

Proposed

## Context

Predictive correction (ADR-006) compensates for IMU sensor fusion lag, but the motor/ESC response time is another significant delay in the control chain. We estimate ~20ms but have no measured value. Accurate thrust characterization — both the static thrust curve and the dynamic response time — is critical for:

1. **Tuning `LEAD_TIME_MS`** — knowing the actual motor lag lets us include it in the prediction budget.
2. **Feed-forward control** — mapping DShot command to expected thrust enables model-based compensation.
3. **Mixer linearization** — the thrust-to-throttle relationship is nonlinear (see reference data below). A linearized mixer could improve PID behavior across the operating range.

## Decision

Build a dedicated thrust measurement test bench using an I2C load cell (e.g. HX711-based or NAU7802-based) connected directly to the Pico 2. This bench measures:

- **Static thrust curve** — thrust (grams) vs DShot throttle command at steady state.
- **Step response time** — time from DShot command change to thrust reaching 90% of final value (T90).
- **Thrust asymmetry** — compare the two motors to quantify unit-to-unit variation.

### Test procedure (planned)

1. Mount motor + prop pointing down onto a rigid arm resting on the load cell.
2. Sweep throttle from idle to max in fixed steps, recording steady-state thrust at each.
3. For step response: command a throttle jump (e.g. 30% → 70%) and record thrust at high sample rate (~1kHz) until settled.
4. Log everything to SD card for offline analysis.

## Reference Data: BetaFPV Lava 1104 7200KV

From manufacturer specifications ([motor](https://betafpv.com/products/lava-series-1104-brushless-motors), [prop](https://betafpv.com/products/gemfan-2218-3-blade-propellers-1-5mm-shaft)). Test conditions: 12V, Gemfan 2.2" 3-blade prop (GF 2.2in-3B). See `specification/BetaFPV Lava 1104 motor.png` for the original data sheet.

| Throttle (%) | Current (A) | Thrust (g) | Efficiency (g/W) | Input (W) |
|:---:|:---:|:---:|:---:|:---:|
| 30 | 0.51 | 29 | 4.71 | 6.10 |
| 40 | 1.25 | 58 | 3.85 | 15.04 |
| 50 | 2.29 | 82 | 2.99 | 27.51 |
| 60 | 3.49 | 105 | 2.52 | 41.88 |
| 70 | 4.36 | 137 | 2.63 | 52.32 |
| 80 | 5.91 | 163 | 2.30 | 70.88 |
| 90 | 8.17 | 192 | 1.96 | 97.99 |
| 100 | 10.20 | 234 | 1.91 | 122.45 |

Key observations from the reference data:
- **Nonlinear thrust curve** — thrust roughly doubles from 30% to 60%, but only increases ~2.2x from 60% to 100%. The low-throttle region is more sensitive per percent.
- **Efficiency peaks at low throttle** — 4.71 g/W at 30% vs 1.91 g/W at 100%. Operating near midrange is most efficient.
- **Our operating range** — with `BASE_THROTTLE = 300` and DShot range 48–2047, our baseline sits around 15% of full DShot range, likely in the 30–50% effective throttle region. The load cell bench will map DShot commands to actual thrust precisely.

### What the reference data does NOT tell us

- **Step response time** — how fast thrust changes when the DShot command changes. This is the motor lag we need for ADR-006's prediction budget.
- **Actual thrust at our battery voltage** — the bench uses regulated 12V; our PDB voltage sags under load.
- **Prop-to-prop variation** — manufacturing tolerances in the Gemfan 2218.
- **Motor-to-motor variation** — our two motors may not produce identical thrust at the same command.

## Hardware (tentative)

- **Load cell**: I2C-connected, ≥500g range, ≥10 Hz sample rate (ideally 80 Hz for step response).
- **Mounting**: rigid frame, motor pointing down onto load cell platform.
- **Reuse**: same Pico 2, same DShot driver, same SD telemetry pipeline.

## Consequences

- **Positive:** Measured thrust curve enables mixer linearization and accurate feed-forward.
- **Positive:** Measured step response time closes the last unknown in the latency budget.
- **Positive:** Reuses existing Pico 2 infrastructure (DShot, SD telemetry, analyser).
- **Negative:** Requires additional hardware (load cell + amplifier board, mounting frame).
- **Negative:** Separate test bench — results must be transferred manually to flight bench config.
