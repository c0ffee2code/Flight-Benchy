# ADR-010: GRV + Calibrated Gyro Dual Report (replaces GIRV in M4)

**Status:** Implemented
**Date:** 2026-02-20
**Supersedes:** Report selection sub-decision in ADR-008 (GIRV for both loops)

## Problem

Test run 2026-02-19 confirmed ~1.5°/min drift in encoder-vs-IMU divergence while the M4 cascaded PID ran.

**Root cause:** `gyro_integrated_rotation_vector` (GIRV, 0x2A) integrates raw gyro on-chip without any accelerometer correction. Gyro bias accumulates without bound. Over a 9-minute run, encoder-to-IMU divergence grew from ~0° to 10.8°. The controller continued to hold "flat" according to the IMU, but the lever was physically 10.8° off horizontal. The integral term never engaged to correct it (ki=0 in both loops), so the drift was entirely uncontrolled.

Evidence from `test_runs/2026-02-19_22-05-59/log.csv`:
- Run duration: ~9 min
- Encoder-to-IMU divergence at end: 10.8°
- Drift rate: ~1.5°/min
- IMU MAE vs encoder: grew monotonically over the run

## Decision

Replace GIRV with two separate BNO085 reports:

| Loop | Report | ID | Rate | Purpose |
|------|--------|----|------|---------|
| Outer (angle) | Game Rotation Vector | 0x08 | 50 Hz | Drift-free roll/pitch via gyro+accel fusion |
| Inner (rate)  | Calibrated Gyroscope | 0x02 | 200 Hz | Bias-compensated angular velocity |

Both are standard BNO085 reports, concurrent enabling is supported by the driver via `enable_feature()`.

## Alternatives Considered

**Integral term (band-aid):** Adding ki to the angle loop would counteract steady-state angle error, including drift-induced offset. Rejected because it treats a symptom rather than the root cause. A non-zero integral would also accumulate error from genuine disturbances, complicating tuning.

**AS5600 encoder for the angle loop:** The encoder is drift-free and highly accurate (~0.09°). Rejected because it breaks the flight controller philosophy — a real FC has no encoder at the rotation centre; the IMU is the only attitude reference. Using the encoder for the control loop undermines the goal of validating FC-applicable sensor fusion.

**Rotation Vector (0x05):** Uses magnetometer for absolute heading. Unnecessary for a single-axis bench with no yaw requirement. GRV (no magnetometer) is cleaner.

## Driver Impact

No driver changes required. Both reports are implemented in the BNO085 driver:

- `imu.game_quaternion` — `SensorFeature4`, `__iter__` yields `(qr, qi, qj, qk)`, `.full` returns `(qr, qi, qj, qk, accuracy, ts)`
- `imu.gyro` — `SensorFeature3`, `__iter__` yields `(x, y, z)` in rad/s, `.full` returns `(x, y, z, accuracy, ts)`

Both are enabled independently via `.enable(hertz=N)`.

## Latency Notes

GRV has comparable filter group delay to GIRV for attitude — BNO085's on-chip Kalman filter runs at the same base rate for both. ADR-006 measured ~10ms group delay for game_quaternion, and the cascaded PID was designed with that in mind. `FEEDFORWARD_LEAD_MS` was set to 0 during initial commissioning and re-enabled at 10ms after baseline validation; see ADR-006 amendment for the revised rate-estimate formula (uses `gyro_x` directly instead of GRV finite difference).

Calibrated gyro latency is ~1–2ms (sensor sampling + I2C), same as the angular velocity component of GIRV was. No change to inner loop response.

## Implementation Changes

### `main.py`

- Added `GRV_REPORT_HZ = const(50)` constant (outer loop rate)
- `arm_motors()`: replaced `imu.gyro_integrated_rotation_vector.enable(hertz=IMU_REPORT_HZ)` with two separate enables
- `stabilize()` seed: `imu.game_quaternion.full` instead of `imu.gyro_integrated_rotation_vector.full`
- `stabilize()` inner loop: `gx, _gy, _gz = imu.gyro` for rate input; `gyro_x = degrees(gx)`
- `stabilize()` outer loop: `iqr, iqi, iqj, iqk = imu.game_quaternion` before computing `imu_roll`
- `init_session()`: config YAML updated to `angle_report`/`angle_report_hz`/`rate_report`/`rate_report_hz`

### `tools/analyse_telemetry.py`

- `print_config_summary()`: dual-branch logic — old `report_hz` key (GIRV) and new `angle_report_hz` key, so all run folders display cleanly

## Consequences

### Positive

- **Drift-free angle reference** — GRV accelerometer correction prevents quaternion drift over minutes/hours, matching what a real FC would provide
- **Same inner loop latency** — calibrated gyro has the same ~1–2ms latency as GIRV's angular velocity component
- **Tare compatibility** — tare was performed on `game_quaternion` (basis=1, GRV), so the persisted tare DCD carries over directly with no recalibration required
- **Cleaner separation of concerns** — attitude source (fused, drift-corrected) and rate source (raw, low-latency) are explicit

### Negative / Trade-offs

- **Two I2C reads per inner cycle** instead of one — minor bus overhead (~0.3ms extra at 400 kHz), well within the 5ms budget
- **Quaternion repeats in telemetry** — GRV updates at 50 Hz but is logged at the inner loop rate (200 Hz), so `IMU_QR/QI/QJ/QK` columns repeat every 4 rows. This is correct behaviour, not a bug
- **Calibrated gyro Q9 fixed-point** vs GIRV Q10 — both convert to float via `degrees()` identically; no code change needed

## Verification Criteria

1. Deploy `main.py` to Pico, run for 5+ minutes
2. Copy run folder to `test_runs/`, run `python tools/analyse_telemetry.py test_runs/<run>/`
3. Config summary shows `angle=game_rotation_vector @ 50 Hz, rate=calibrated_gyroscope @ 200 Hz`
4. Encoder-to-IMU divergence stays < 2° over the full run (vs 10.8° before)
5. `config.yaml` in run folder contains `angle_report: game_rotation_vector`
