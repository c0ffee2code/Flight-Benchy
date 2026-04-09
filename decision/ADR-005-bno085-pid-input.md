# ADR-005: BNO085 IMU as Primary PID Input (M2)

**Status:** Accepted
**Date:** 2026-02-15
**Context:** Switch PID control input from AS5600 encoder to BNO085 IMU, matching real flight controller architecture

## Context

The PID controller (M1) used the AS5600 magnetic encoder as its process variable. This was convenient for initial bring-up but doesn't reflect how real flight controllers work — a drone has no encoder at its pivot point. Real flight controllers rely on an IMU for attitude estimation.

M2 switches the PID input to the BNO085 IMU's **game rotation vector** (gyro + accel fusion, no magnetometer). The AS5600 encoder becomes telemetry-only ground truth for offline analysis of IMU lag and angle error.

## Decision

### Game Rotation Vector as PID input

Use `imu.game_quaternion` (BNO report 0x08) instead of the standard rotation vector (0x05). Game rotation vector fuses gyroscope and accelerometer only — no magnetometer corrections. This avoids sudden heading jumps from magnetic interference near the motors and power distribution board.

Report rate set to 100 Hz (`IMU_REPORT_HZ`) at time of writing, which is 2x the PID rate (50 Hz), ensuring fresh IMU data is available every control cycle. ⚠️ **Superseded by ADR-010:** GRV rate was later reduced to 50 Hz (outer loop rate); calibrated gyroscope added at 200 Hz for the inner loop.

### Roll angle extraction via simplified quaternion math

The lever rotates around a single axis (roll/X). Instead of computing full Euler angles (3x `atan2` + `asin`), extract roll with a single `atan2`:

```python
imu_roll = 2.0 * atan2(qi, qr)
```

This is exact for pure X-axis rotation and a good approximation for the mechanically constrained single-axis lever. Full Euler conversion is unnecessary overhead in the 50 Hz control loop.

### Quaternion telemetry format

Telemetry stores raw quaternions from both sensors — no Euler conversion in the hot loop:

```
T_MS,ENC_QR,ENC_QI,ENC_QJ,ENC_QK,IMU_QR,IMU_QI,IMU_QJ,IMU_QK,ERR,P,I,D,PID_OUT,M1,M2
```

- **Encoder quaternion**: AS5600 single-axis angle converted to quaternion via `angle_to_quat(deg)` — rotation around X axis: `(cos(half), sin(half), 0, 0)`
- **IMU quaternion**: Raw `(qr, qi, qj, qk)` from BNO085 game rotation vector
- Quaternion values formatted to 5 decimal places (Q14 precision ~0.00006)
- All Euler analysis (lag measurement, angle error) happens offline from the stored quaternions

### Shared I2C bus

BNO085 shares I2C bus 0 with the AS5600 encoder (GPIO 0/1, 400 kHz). Both devices have distinct addresses (AS5600: 0x36, BNO085: 0x4A). The IMU uses dedicated reset (GPIO 2) and interrupt (GPIO 3) pins.

## Consequences

### Positive

- **Real flight controller architecture** — PID now uses IMU input, same as Betaflight/ArduPilot
- **Ground truth preserved** — AS5600 in telemetry enables offline IMU quality assessment (lag, drift, noise)
- **Quaternion telemetry** — no information loss from Euler conversion; offline tools can compute any representation
- **Minimal hot-loop overhead** — single `atan2` for PID scalar, two trig calls for encoder quaternion conversion

### Negative

- **PID tuning may need adjustment** — IMU has different noise characteristics and latency vs encoder. Existing kp=5.0/ki=0.5 gains may need retuning.
- **IMU initialization time** — BNO085 takes ~100ms to initialize and begin reporting. Acceptable since it happens once at power-on before arming.
- **Game rotation vector drift** — without magnetometer correction, heading can drift over long sessions. Acceptable for single-axis roll control where gravity reference is sufficient.

### Files changed

- `main.py` — added BNO085 imports, IMU initialization, `angle_to_quat()` helper, `IMU_REPORT_HZ` constant; PID input changed from encoder angle to IMU roll; telemetry call updated with quaternion arguments
- `telemetry/recorder.py` — CSV header and `record()` signature updated for quaternion columns (8 quaternion values replace 2 degree values)
- `CLAUDE.md` — updated project state (M2 done), telemetry format documentation
- `README.md` — updated M2 milestone status

### Not changed

- `pid.py` — PID controller is input-agnostic (accepts any scalar error)
- `mixer.py` — mixer is PID-output-agnostic
- `telemetry/sdcard.py` — SD card driver unchanged
- BNO085 driver (`BNO085/driver/`) — used as-is, no modifications needed

## Amendments

### Folder-per-run telemetry (2026-02-15)

Telemetry storage changed from flat files (`/sd/blackbox/log_YYYY-...csv`) to folder-per-run directories:
```
/sd/blackbox/YYYY-MM-DD_hh-mm-ss/
    config.yaml    # PID gains, motor limits, IMU rate, encoder calibration
    log.csv        # Telemetry CSV (same format as before)
```

Each run is self-contained — config and telemetry grouped together. `config.yaml` is written as plain string formatting on MicroPython and enables automated parameter sweep analysis. Desktop analyser script at `tools/analyse_telemetry.py` reads run folders, converts quaternions to roll angles, and produces diagnostic plots and statistics.

### IMU signal quality improvement path (2026-02-15)

PID tuning runs with game rotation vector revealed two issues:

1. **IMU lag** — IMU trails the encoder by ~60ms (trail percentage ~59% across all runs). High P gains on lagged input cause overshoot and growing oscillation.
2. **Reference frame offset** — IMU zero point doesn't align with encoder mechanical zero. Bias of +3–21° observed across runs, causing steady-state error that the I term must slowly correct.

Tested PID gain progression:

| Run | kp | ki | kd | integral_limit | Result |
|-----|----|----|----|----|--------|
| 14-44-05 | 5.0 | 0.5 | 0.0 | 200 | Growing oscillation, ±60° swings |
| 14-54-01 | 2.0 | 0.1 | 0.5 | 50 | Stuck at extremes, insufficient authority |
| 15-01-30 | 3.5 | 0.2 | 0.3 | 50 | Reaches target, can't hold — slow I accumulation |
| 15-11-14 | 3.5 | 0.4 | 0.3 | 50 | Best: MAE 7.1°, holds near target, bias 7° |

**Key tuning insights:**

- **Radians vs degrees**: initial M2 code fed raw `atan2` radians to a PID tuned for degrees — PID output was 57x too weak. Always match units between sensor output and PID gains.
- **IMU lag changes the stability boundary**: kp=5.0 was stable with the near-zero-lag encoder but caused growing oscillation with the ~60ms-lagged IMU. The maximum stable P gain is inversely related to sensor lag.
- **P and I are complementary, not redundant**: kp=2.0 with ki=0.1 couldn't move the lever (insufficient P authority) AND couldn't correct steady-state error (too-slow I). The solution was moderate P (3.5) for immediate response plus stronger I (0.4) for offset correction.
- **D term is essential with lagged input**: kd=0.3 provides damping that prevents the lagged P term from overshooting. Without D, the system oscillates; with too much D (0.5), it fights the already-weak P term.
- **Integral limit should be tight**: reducing from 200 to 50 prevents windup during large transients while still allowing enough I accumulation to correct the ~7° IMU bias.
- **IMU range vs encoder range**: in all runs, IMU range is 3-4x smaller than encoder range (e.g. 14.5° vs 45.4°). The BNO085's internal fusion filter smooths out peak excursions. This means the PID sees a "calmer" signal than reality — good for stability, bad for responsiveness.

**Improvement path (in order of complexity):**

1. **PID gain tuning** — DONE. Current best: kp=3.5, ki=0.4, kd=0.3, integral_limit=50.
2. **BNO085 calibration** — DONE (2026-02-16). All three MEMS sensors calibrated (accel accuracy 2, gyro 3, mag 3). DCD saved to flash, persists across power cycles. See `BNO085/decision/004-sensor-calibration.md`.
3. **BNO085 tare** — DONE (2026-02-16), then corrected (2026-02-22). Initial tare used `basis=0` (Rotation Vector, magnetometer included). In the bench environment (near motors/ESCs/metal) `mag_acc=0`, so this baked an arbitrary ~45° heading offset into the tare frame — magnetometer was usable for calibration status but not for attitude reference near ferrous interference. ⚠️ **Corrected in ADR-008 Amendment 2026-02-22:** tare must use `basis=1` (Game Rotation Vector, gyro+accel only, no magnetometer). Eliminates Phase 0 (figure-8 detach sequence). See BNO085/decision/004-sensor-calibration.md Bug 4.
4. **Gyro-integrated rotation vector** — SUPERSEDED. GIRV (0x2A) was used as the initial M4 inner loop input (ADR-008) because it bundles quaternion + angular velocity in one packet. Hardware testing on 2026-02-19 revealed ~1.5°/min drift (10.8° over 9 min) — GIRV integrates raw gyro without accelerometer correction. Replaced by GRV (0x08) for the outer loop + calibrated gyroscope (0x02) for the inner loop. See ADR-010.

### Calibration + tare results (2026-02-17)

Test run `2026-02-17_19-15-59` compared against pre-calibration run `2026-02-15_21-24-09` (identical PID gains, same hardware configuration):

| Metric | Before | After | Change |
|--------|--------|-------|--------|
| MAE (overall) | 22.11 deg | **2.87 deg** | 7.7x better |
| Max AE | 47.86 deg | **12.71 deg** | 3.8x better |
| RMS Error | 24.48 deg | **2.94 deg** | 8.3x better |
| Bias (IMU-ENC) | +6.85 deg | **-2.87 deg** | Reduced, now consistent |
| IMU range | 25.6 deg | **77.7 deg** | Tracks encoder (86 deg) |
| Oscillation freq | 0.08 Hz | **0.00 Hz** | Eliminated |

Before calibration, the IMU range was only 25.6 deg vs 102 deg encoder range — the PID was working with a severely compressed, biased signal. After calibration + tare, the IMU faithfully tracks the full lever motion (77.7 vs 86.0 deg encoder range) and the limit cycle oscillation is gone.

The remaining -2.87 deg bias is likely IMU lag manifesting as a systematic offset during active stabilization (predictive correction with `lead_time_ms=60` does not fully compensate). This is a dynamic effect — the static tare test showed only 0.08 deg bias at rest.

See `test_runs/2026-02-17_19-15-59/` for data and plot.

## Dependencies

- BNO085 driver delivering reliable game rotation vector data at 100 Hz — validated in `BNO085/tests/report_rate/`
- BNO085 physically connected: I2C on GPIO 0/1, RST on GPIO 2, INT on GPIO 3
