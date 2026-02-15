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

Report rate set to 100 Hz (`IMU_REPORT_HZ`), which is 2x the PID rate (50 Hz), ensuring fresh IMU data is available every control cycle.

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

**Planned improvements (in order of complexity):**

1. **PID gain tuning** — continue iterating kp/ki/kd with telemetry analyser feedback. Current best: kp=3.5, ki=0.4, kd=0.3, integral_limit=50.
2. **IMU tare at startup** — read IMU roll once before entering control loop, subtract as offset. Eliminates reference frame mismatch without full calibration. Requires BNO085 driver work.
3. **BNO085 calibration** — run the BNO085's built-in calibration procedure (magnetometer, accelerometer, gyroscope) and save calibration data. Improves absolute accuracy and reduces bias.
4. **Gyro-integrated rotation vector** — switch from game rotation vector (report 0x08, 344 Hz max, ~60ms lag) to gyro-integrated rotation vector (report 0x2A, 1000 Hz, 0–2ms lag). Test data in `BNO085/tests/report_rate/results/gyro_integrated_vector/` shows dramatically lower latency. Trade-off: drifts without accel correction, best combined with tare or periodic re-reference. May be ideal as the inner loop input for M4 cascaded PID.

Each step builds on the previous. Gain tuning is pure config changes; tare/calibration require BNO085 driver development; gyro-integrated RV may reshape the control architecture toward M4 cascaded PID.

## Dependencies

- BNO085 driver delivering reliable game rotation vector data at 100 Hz — validated in `BNO085/tests/report_rate/`
- BNO085 physically connected: I2C on GPIO 0/1, RST on GPIO 2, INT on GPIO 3
