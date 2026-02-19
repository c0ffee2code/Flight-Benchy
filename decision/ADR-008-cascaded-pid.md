# ADR-008: Cascaded PID — Angle Loop + Rate Loop (M4)

**Status:** Implemented — baseline working, tuning in progress
**Date:** 2026-02-17
**Context:** Replace single angle PID with cascaded architecture matching real flight controllers

## Context

The current controller (M1/M2) uses a single PID loop at 50 Hz operating on the BNO085 game rotation vector (report 0x08). This works — MAE 2.87° after calibration and tare — but has inherent limitations:

1. **Sensor fusion lag (~60ms)** — the game rotation vector passes through BNO085's on-chip Kalman-like filter. The PID always reacts to where the lever *was*, not where it *is*. ADR-006's predictive correction partially compensates, but it's a workaround, not a fix.
2. **Single loop bandwidth ceiling** — the outer angle loop can't be faster than the fusion filter update rate without seeing stale data. Increasing gains to improve response leads to oscillation due to the phase lag.
3. **Architecture gap** — real flight controllers (Betaflight, ArduPilot) universally use cascaded PID. A single-loop controller has no equivalent in production drones.

## Decision

### Cascaded loop architecture

Replace the single angle PID with two nested loops:

```
                ┌─────────────────────────────────────────────────┐
 setpoint ──►(+)──► ANGLE PID ──► desired rate ──►(+)──► RATE PID ──► mixer ──► motors
  (0 deg)   (-)    (outer, slow)    (deg/s)      (-)    (inner, fast)
              ▲                                    ▲
              │                                    │
         angle from                          angular velocity
         quaternion                          from gyro (raw)
```

- **Outer loop (angle)** — compares desired angle (0°) with measured angle. Outputs a desired rotation rate in deg/s. Runs at 50 Hz (every `PID_INTERVAL_MS`).
- **Inner loop (rate)** — compares desired rate with measured angular velocity. Outputs a scalar fed to the mixer. Runs at 200 Hz (every GIRV report).

Both loops run in the same cycle — no separate timer or interrupt. The main loop runs at the inner loop rate (200 Hz). An iteration counter or elapsed-time check gates the outer loop to execute every Nth inner cycle (e.g. every 4th at 200/50 Hz).

### Gyro Integrated Rotation Vector (report 0x2A)

Use BNO085's **Gyro Integrated Rotation Vector** instead of requesting separate game rotation vector + raw gyro reports. This single report, delivered on SHTP channel 5, provides both values in one I2C read:

| Field | Content | Use |
|-------|---------|-----|
| `qr, qi, qj, qk` | Rotation quaternion (Q14) | Outer loop angle input |
| `ax, ay, az` | Angular velocity in rad/s (Q10) | Inner loop rate input |

**Why this report and not separate gyro + game RV:**

- **Single I2C transaction** — both attitude and rate from one packet, halving bus traffic vs two reports
- **Synchronized data** — attitude and angular velocity from the same sensor instant, no timestamp alignment needed
- **Angular velocity is raw gyro** — bypasses the fusion filter, so latency is ~1-2ms (sensor sampling + I2C), not ~60ms
- **Up to 1000 Hz** — the driver already declares max rate 1000 Hz for this report. Actual achievable rate on MicroPython/I2C TBD

**Why not calibrated gyroscope (0x02) or raw gyroscope (0x15):**

- Calibrated gyro (0x02) is a separate report on channel 3 — would require enabling two concurrent reports and doubling I2C reads
- Raw gyro (0x15) provides uncalibrated data needing bias compensation in user code
- Gyro Integrated RV bundles everything: calibrated angular velocity + fused quaternion in one packet

### Inner loop rate

Target: **200 Hz** for the initial implementation. This is 4x faster than the current 50 Hz single loop and conservative enough for MicroPython over I2C.

Constraints at 200 Hz:
1. **I2C bus bandwidth** — at 400 kHz, a 14-byte GIRV packet + SHTP header (~24 bytes total) takes ~0.5ms wire time
2. **MicroPython overhead** — `update_sensors()` parsing, PID compute, mixer, DShot write
3. **5ms cycle budget** — must complete all inner loop work within this window

Higher rates (400+ Hz) are deferred to a future performance milestone, which may include switching BNO085 from I2C to SPI, reducing GC pressure, and other optimizations.

### Predictive correction (ADR-006) disposition

With a fast inner rate loop using near-zero-lag gyro data, the predictive correction from ADR-006 may become unnecessary. Plan:

1. Implement cascaded PID with prediction still enabled
2. Test with prediction disabled — if MAE is comparable or better, remove it
3. If prediction still helps (possible — outer loop still uses fused quaternion with lag), keep it for the outer loop only

### PID gains — tuning approach

The cascaded architecture needs separate gains for each loop with different units than the old single PID. Existing gains (kp=3.5, ki=0.4, kd=0.3) are not transferable — fresh tuning is required.

#### Unit chain analysis

Map the signal flow to understand what each gain "means" physically:

```
angle error (deg) --[angle_kp]--> rate setpoint (deg/s) --[rate_kp]--> mixer scalar (throttle offset)
```

- **Angle kp**: degrees of error → deg/s of desired correction rate.
  `kp=4.0` means 10° off → 40 deg/s setpoint — "correct at 40°/s".
- **Rate kp**: deg/s of rate error → throttle units.
  `kp=0.8` means 40 deg/s rate error → 32 throttle offset (out of ±230 max authority).
- **Combined static gain**: `angle_kp × rate_kp` should approximate the old working single-PID kp.
  Old: kp=3.5 → 10° error → 35 throttle. Target combined: ~3.0–4.0.

#### Gain sizing from old working single PID

The M2 single PID (`kp=3.5, ki=0.4, kd=0.3` at 50 Hz, MAE 2.87°) provides a baseline:

| Metric | Single PID | Cascaded equivalent |
|--------|-----------|---------------------|
| 10° error → throttle | 35 (kp alone) | angle_kp × rate_kp × 10 |
| Damping mechanism | D term on angle error derivative | Inner rate loop opposes angular velocity directly |
| Integral accumulation rate | 0.4 × error × 0.02s per cycle | angle: ki × err × 0.02s; rate: ki × err × 0.005s (4× faster!) |

Key insight: **rate ki accumulates 4× faster** than angle ki because the inner loop runs at 200 Hz vs 50 Hz. Scale it down proportionally.

#### Rate D term: why it's dangerous at 200 Hz

The rate PID D term computes `kd × d(rate_error)/dt`. At 200 Hz (dt=5ms):

- Gyro noise of ±5 deg/s between samples → derivative = 1000 deg/s²
- Direction reversals (gyro flips sign): Δrate_error can be 500 deg/s in one cycle → derivative = 100,000 deg/s²
- With kd=0.01: D output = 0.01 × 100,000 = **1000 throttle units** — far exceeding motor range

**Rule: start with rate kd=0 and add only if needed, with aggressive filtering.**

#### Integral limits at different loop rates

At 200 Hz inner loop, even ki=0.1 accumulates fast:
- Sustained 50 deg/s rate error for 1 second: integral = 50 × 1.0 = 50 → ki × 50 = 5.0 per second
- At 200 Hz that's 200 accumulation steps per second vs 50 for the outer loop

Scale inner ki to ~¼ of what you'd use at 50 Hz, or keep integral_limit tight.

### Telemetry changes

Telemetry principle: record all inputs and outputs of each algorithm stage so behaviour can be fully reconstructed offline. The CSV format gains columns for both loops:

```
T_MS,ENC_QR,...,ENC_QK,IMU_QR,...,IMU_QK,GYRO_X,ANG_ERR,ANG_P,ANG_I,ANG_D,RATE_SP,RATE_ERR,RATE_P,RATE_I,RATE_D,PID_OUT,M1,M2
```

22 columns total. New/changed columns vs the pre-cascaded schema:
- `GYRO_X` — raw gyro angular velocity (deg/s), the inner loop process variable
- `ANG_ERR` — outer loop angle error (deg), replaces `ERR`
- `ANG_P/I/D` — outer loop PID terms, replaces `P/I/D`
- `RATE_SP` — rate setpoint from outer loop (deg/s)
- `RATE_ERR` — inner loop rate error (deg/s)
- `RATE_P/I/D` — inner loop PID terms

Breaking change to the CSV schema — the desktop analyser (`tools/analyse_telemetry.py`) must be updated to match.

## Implementation Plan

### Phase 1: Validate GIRV on hardware

Before writing the full cascaded controller, confirm GIRV data is usable:

1. Enable GIRV at 200 Hz
2. Tight loop: `update_sensors()` → read quaternion + angular velocity → measure `ticks_us` per iteration
3. Verify angular velocity values are sane (sign, magnitude, units) by moving the lever manually

### Phase 2: Cascaded controller

1. Add a second `PID` instance for the rate loop (reuse existing `pid.py` — it's input-agnostic)
2. Single main loop at inner rate (~200 Hz), with an iteration counter gating the outer angle loop every Nth cycle
3. Inner loop reads angular velocity from GIRV, computes rate PID, writes to mixer
4. Outer loop reads quaternion from the same GIRV report, computes angle PID, outputs rate setpoint
5. Update telemetry format and `TelemetryRecorder.record()` signature
6. Update `config.yaml` format with inner/outer PID sections

### Phase 3: Tune and validate

1. Tune inner rate loop first (disable outer loop, command fixed rate setpoints)
2. Then tune outer angle loop with inner loop active
3. Compare against M2 baseline (2.87° MAE) using SD card telemetry
4. Evaluate whether predictive correction (ADR-006) is still needed

## Pre-baseline postmortem

Three bugs had to be fixed before the cascade produced correct behaviour. Test run data from this period has been dropped — it reflects debugging noise, not controller performance.

### Bug 1: outer loop sign inverted (positive feedback)

The outer loop called `pid.compute(predicted_roll, dt)`. `pid.compute(error, dt)` takes an **error directly** — it has no internal setpoint subtraction. With setpoint = 0°, the correct call is `pid.compute(-predicted_roll, dt)`.

With the bug: lever at +10° → `rate_setpoint = kp × +10 = positive` → inner loop drives lever **further positive** → pure positive feedback. All early runs oscillated only because friction and end-stops bounded the instability, not the controller.

Fix: `ang_err = -predicted_roll` and `angle_pid.compute(-predicted_roll, outer_dt)`.

### Bug 2: combined gain too low

After the sign fix, the first clean run used `angle_kp=1.5 × rate_kp=1.0 = 1.5` combined gain. The lever's gravitational imbalance requires a continuous motor differential to hold horizontal. At small angles near 0° the controller produced < 1 throttle unit of differential — essentially zero. Lever free-fell from horizontal to the ramp stop in < 52ms.

Fix: `rate_kp` 1.0 → 2.5, combined gain 1.5 → 3.75, matching the old working single-PID effective authority.

### Bug 3: mixer sign backwards

With corrected outer sign and adequate gain, the lever started in the down position and the bottom motor went to max thrust — lever pinned to the ramp.

`m1 = base + output, m2 = base - output` drove the wrong motor when `output` was positive. The isolated rate-loop test had confirmed gyro convergence but not physical position direction — a sign error in the mixer cancels out in pure rate control (gyro provides the feedback) but manifests in position control (angle error sustains the wrong motor high).

Fix in `mixer.py`: `m1 = base - output, m2 = base + output`. Sign chain end-to-end: positive angle error → positive `rate_setpoint` → positive `pid_output` → M2>M1 → lever lifts toward horizontal.

### Baseline gains (working state)

| Parameter | Value |
|-----------|-------|
| angle_pid | kp=1.5, ki=0.0, kd=0.2, limit=100 |
| rate_pid | kp=2.5, ki=0.0, kd=0.0, limit=50 |
| THROTTLE_MAX | 400 |
| BASE_THROTTLE | 250 |
| Combined static gain | 1.5 × 2.5 = 3.75 |

Motor authority ±150 DShot units. Inner loop stays proportional up to 60 deg/s rate error. Outer loop D term (kd=0.2 at 50 Hz) provides velocity damping without derivative noise at 200 Hz. No integrals until steady-state error is characterised.

## Consequences

### Positive

- **Dramatically reduced effective lag** — inner loop reacts to gyro in ~2ms vs ~60ms for fused quaternion
- **Real flight controller architecture** — directly transferable knowledge to Betaflight/ArduPilot tuning
- **Better disturbance rejection** — inner loop counters perturbations before they become large angle errors
- **Cleaner control structure** — angle loop sets intent (rate), rate loop executes. Separation of concerns

### Negative

- **Doubled tuning complexity** — two sets of PID gains to tune instead of one
- **GIRV report untested in flight context** — channel 5 parsing exists in the driver but hasn't been used in the control loop yet. May surface edge cases

## Dependencies

- BNO085 driver: GIRV support on channel 5 — implemented (`BNO085` submodule, commit `b12c718`)
- `pid.py` — reusable as-is for both loops
- `mixer.py` — sign corrected (Run 6): `m1 = base - output`, `m2 = base + output`
- Completed: M2 (IMU input), M2a (telemetry), M3 (mixer extraction)
