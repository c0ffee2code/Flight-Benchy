# ADR-006: Predictive Correction for IMU Lag Compensation

## Status

Accepted

## Context

The 225s test run (2026-02-15_19-56-03) with BNO085 game rotation vector as PID input revealed a persistent limit cycle: MAE 19.9°, encoder range 109.8°, oscillation at 0.10 Hz.

Root cause: the BNO085 game rotation vector introduces ~60ms of latency (sensor fusion processing + I2C transport). The PID controller sees where the lever *was*, not where it *is*. By the time motor thrust changes (additional ~20ms ESC response), the lever has moved further — the controller is always chasing.

## Decision

Apply first-order lead compensation by predicting the current angle from the angular rate:

```python
angular_rate = (imu_roll - prev_imu_roll) / dt   # deg/s
predicted_roll = imu_roll + angular_rate * LEAD_TIME_S
```

- `LEAD_TIME_MS = 60` — see latency breakdown below. Tunable.
- Angular rate is computed from consecutive game rotation vector roll readings. The BNO085's internal fusion filter already smooths high-frequency noise, making this numerical derivative reasonably clean.
- The predicted roll is fed to `pid.compute()` instead of the raw IMU roll.

### Latency budget and LEAD_TIME_MS rationale

The dominant source of lag is the BNO085's on-chip sensor fusion processor (Hillcrest Labs ARVR stabilization filter). It combines accelerometer, gyroscope, and magnetometer through a Kalman-like filter before outputting the fused game rotation vector quaternion. This introduces significant group delay.

| Source | Estimate |
|--------|----------|
| BNO085 fusion filter (game RV) | ~50–60ms |
| I2C transport + MicroPython overhead | ~2ms |
| **Total IMU lag** | **~60ms** |

Motor/ESC response (~20ms) is a separate, additive delay not included in the initial value. It can be added later by increasing `LEAD_TIME_MS` to ~80 if testing shows benefit.

This is also why M4's raw gyroscope inner loop should help — raw gyro readings bypass the fusion filter entirely, reducing latency to just sensor sampling + I2C (~1–2ms).

## Telemetry Interpretation

The CSV format is unchanged. Column semantics:

| Column | With prediction | Without prediction |
|--------|----------------|--------------------|
| IMU_Q* | Raw IMU quaternion (lagged) | Raw IMU quaternion |
| ERR | Predicted roll (what PID sees) | Raw IMU roll |

Raw IMU roll is recoverable from the IMU quaternion columns: `degrees(2 * atan2(IMU_QI, IMU_QR))`. The analyser plots both raw IMU and predicted roll when prediction is active.

Config gains a `prediction.lead_time_ms` field.

## Relationship to PID D-term

Prediction and the D-term are complementary:
- **Prediction** shifts the operating point forward in time — the PID "sees" a corrected angle closer to ground truth.
- **D-term** dampens oscillation around that corrected position.

## Relationship to M4 Cascaded PID

M4 adds an inner rate loop using raw gyroscope data (not fused quaternion). The inner loop's gyro feedback is nearly instantaneous, which inherently reduces the lag problem. When M4 is implemented, predictive correction may become unnecessary or need re-tuning. This approach is a simpler intervention to test before the full cascaded architecture.

## Consequences

- **Positive:** Reduced phase lag should shrink the limit cycle amplitude and improve tracking.
- **Positive:** Zero architecture change — three lines in the hot loop, fully reversible.
- **Negative:** Numerical differentiation amplifies noise. The game rotation vector's built-in smoothing mitigates this, but aggressive lead times may cause jitter.
- **Negative:** `LEAD_TIME_MS` is a tuning parameter that depends on the specific IMU configuration and may need adjustment if `IMU_REPORT_HZ` changes.
