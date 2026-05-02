# DR-014: Setpoint Representation and Config Structure

**Status:** Accepted
**Date:** 2026-05-02

## Context

The target attitude for the lever is currently hardcoded as 0° (horizontal) inside `flight.py`. Making the setpoint configurable is useful for testing control behaviour at off-horizontal targets and is the natural first step toward multi-axis control where each axis needs an independently configurable setpoint.

Two representation choices exist: degrees or quaternions.

## Decision

Setpoints are stored in **degrees** in `config.json`.

Quaternions are an internal computation representation, not a user-facing command representation. All real flight control systems (ArduPilot, Betaflight, PX4) accept attitude setpoints in degrees — from the pilot stick, from an autopilot waypoint, or from a ground station command. The controller converts internally as needed.

For this bench the angle loop already operates fully in degrees (`imu_roll` is in degrees, PID gains are tuned for degree-scale errors, rate setpoint is in deg/s). Expressing the setpoint in degrees is therefore zero-cost — no conversion is required and the gains retain their physical meaning.

Quaternion setpoints would only be warranted if: (a) commanding full 3D attitudes simultaneously where gimbal lock in the setpoint itself is a risk, and (b) the controller computes quaternion error internally. Neither applies here, and even in cases where (b) is true, the quaternion is derived from degree inputs rather than configured directly.

## Structure

Setpoints live inside `bench.session` — alongside `duration_s` — not inside `vehicle`. The reasoning: on a real aircraft the setpoint is commanded per-flight by the pilot or autopilot, not baked into the vehicle definition. The bench operator sets it before each run, which is the exact analog. `vehicle` is reserved for control algorithm parameters (gains, IMU reports, motor limits, feedforward) that stay fixed across sessions.

```json
"bench": {
  "session": {
    "duration_s": 120,
    "setpoint": {
      "roll_deg": 0.0,
      "pitch_deg": 0.0,
      "yaw_deg": 0.0
    }
  }
}
```

Only `roll_deg` is used by the current single-axis controller. `pitch_deg` and `yaw_deg` are present as reserved keys — they have no effect until multi-axis control is implemented, but their presence in the schema makes the intent explicit and avoids a breaking config change later.

The `vehicle` / `bench` top-level split is described in the config restructure that accompanies this DR.

## Consequences

- Setpoints are human-readable and directly interpretable without tooling.
- PID gains retain their physical meaning (deg, deg/s units).
- Off-horizontal hold tests require only a config change — no code change.
- Multi-axis extension adds keys to the existing `setpoint` object — no structural change.
- `flight.py` computes angle error as `setpoint_roll_deg - predicted_angle` rather than the current implicit `-(imu_roll + gyro_x * lead_s)`.