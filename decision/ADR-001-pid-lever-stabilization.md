# ADR-001: PI(D) Controller for Single-Axis Lever Stabilization

**Status:** Validated on hardware
**Date:** 2026-02-08
**Context:** Flight control test bench — first closed-loop control milestone

## Context

The test bench has a swinging lever pivoting around a central axis. Two drone motors are mounted on opposite ends of the lever, both producing downward thrust (inverted for safety — prevents the bench from lifting off the desk). An AS5600 magnetic encoder at the pivot provides ground-truth angle measurement with ~0.088° resolution and ~486 μs latency.

The lever's mechanical range is approximately ±50° from horizontal. At rest (motors off), the lever settles at one of its extreme positions depending on weight distribution.

### Goal

Hold the lever at 0° (horizontal) within ±3° using differential motor thrust, controlled by a PI(D) feedback loop reading the AS5600 encoder.

### Why AS5600 Only (No IMU)

The BNO085 IMU driver needs further work before it can be used in a control loop. The AS5600 encoder is the higher-precision sensor anyway (~0.088° vs IMU's ~1-2° with lag), making it the right choice for initial PID tuning. The IMU will be added later for comparison and sensor fusion experiments.

## Decision

### Control Architecture

```
┌──────────┐     ┌──────────┐     ┌───────────────────┐     ┌──────────┐
│  AS5600   │────▶│   PID    │────▶│ MotorThrottleGroup│────▶│  Motors  │
│  Encoder  │     │ (Core 0) │     │    (Core 1)       │     │  M1, M2  │
└──────────┘     └──────────┘     └───────────────────┘     └──────────┘
     ▲                                                            │
     └────────────────────────────────────────────────────────────┘
                          Physical feedback
```

- **Core 0:** Reads encoder, computes PID, sets throttle values
- **Core 1:** Sends DShot commands at 1 kHz continuously (handled by MotorThrottleGroup from DShot submodule)

### PID Output Mapping

Both motors push their side of the lever **down**. Differential thrust creates torque:

```
error = target_angle - current_angle    (target = 0°)
output = Kp × error + Ki × ∫error·dt + Kd × d(error)/dt

M1_throttle = BASE_THROTTLE + output
M2_throttle = BASE_THROTTLE - output

Both clamped to [THROTTLE_MIN(70) .. THROTTLE_MAX(600)]
```

If the lever diverges instead of converging, the sign convention is wrong — flip the sign of Kp.

### Initial Tuning Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Kp | 5.0 | 5 throttle units per degree — conservative start |
| Ki | 0.5 | Slow integral to eliminate steady-state error |
| Kd | 0.0 | Disabled initially — add if oscillation needs damping |
| BASE_THROTTLE | 150 | Below movement threshold (~250), leaves headroom for PID |
| INTEGRAL_LIMIT | ±200 | Anti-windup clamp prevents integral runaway |
| Loop rate | 50 Hz | 20 ms period — well within encoder latency budget |

These values are starting points. Tuning will happen on hardware iteratively.

### Anti-Windup

The integral term is clamped to ±INTEGRAL_LIMIT. Without this, when the lever is far from target (e.g., at ±50° during initial approach), the integral accumulates a large value that causes massive overshoot once the lever reaches 0°.

### State Machine

The program operates as a 4-state machine for safe operation:

1. **DISARMED** — Display invite, wait for B+Y button combo
2. **ARMING** — Start Core 1 loop, arm ESCs, idle at minimum throttle
3. **READY CHECK** — Safety banner showing current angle, user presses A to confirm readiness
4. **STABILIZING** — PID loop active, display angle and throttle values, B+Y to disarm

The ready check state exists because PID engagement moves the lever immediately — the operator must confirm they are prepared for physical movement.

## Consequences

### Positive

- Simple, well-understood controller suitable for single-axis stabilization
- Differential thrust mapping is straightforward with two opposing motors
- AS5600 provides fast, accurate feedback with no processing overhead
- MotorThrottleGroup handles timing-critical DShot on Core 1, isolating PID from motor communication concerns
- State machine ensures safe arming/disarming workflow

### Risks

- **Sign convention uncertainty:** Motor-to-angle mapping depends on physical wiring. First test showed divergence (positive feedback) — resolved by flipping error sign from `-current_angle` to `+current_angle`.
- **BASE_THROTTLE too low/high:** If 150 doesn't provide enough control authority at 0°, or causes unwanted movement during READY CHECK, it needs adjustment.
- **No D term initially:** System may oscillate near setpoint. Adding Kd is straightforward if needed.
- **Gravity asymmetry:** The lever may need different thrust at different angles. A simple PID may struggle if the gravitational torque curve is highly nonlinear. For ±3° target this should be acceptable.

## Test Bench vs Real Drone: Simplifications Made

This section documents what a real flight controller does differently and why the test bench deliberately simplifies each aspect. Understanding these gaps defines the learning path toward a full flight controller.

### 1. Single axis vs three axes

A real quadcopter controls roll, pitch, and yaw simultaneously. Each axis has its own PID controller, and all three outputs are combined by a **mixer** to produce individual motor commands:

```
IMU → PID (roll)  → roll_output  ──┐
IMU → PID (pitch) → pitch_output ──┼──▶ MIXER ──▶ M1, M2, M3, M4
IMU → PID (yaw)   → yaw_output  ──┤
pilot/althold ──▶ base_throttle  ──┘
```

The mixer is pure addition/subtraction based on motor geometry. For a typical X-frame quadcopter:

```
M1 (front-right) = base + pitch - roll - yaw
M2 (rear-right)  = base - pitch - roll + yaw
M3 (rear-left)   = base - pitch + roll - yaw
M4 (front-left)  = base + pitch + roll + yaw
```

Signs depend on motor positions and propeller rotation directions.

**Test bench simplification:** One axis (pitch), two motors, trivial mixer (`base ± output`). The algorithm structure is identical — just fewer axes and motors. The `base + output` / `base - output` pattern in the test bench is already a 1-axis mixer.

### 2. Single PID loop vs cascaded PIDs

Real flight controllers use **two nested loops per axis**:

```
                    Outer loop                    Inner loop
                    (angle → rate)                (rate → thrust)
                    ~100-500 Hz                   ~1-8 kHz
                    ┌──────────┐                  ┌──────────┐
desired angle ────▶│ Angle PID │──▶ desired rate ▶│ Rate PID │──▶ mixer
                    └────┬─────┘                  └────┬─────┘
                         │                              │
                    angle from IMU              gyro rate (raw)
```

The inner **rate loop** uses the gyroscope directly (near-zero lag, very fast updates). It controls *how fast* the drone is rotating. The outer **angle loop** controls *where* the drone is pointing and feeds a desired rotation rate to the inner loop.

This cascade matters because:
- Gyro rate has much less lag than integrated angle
- The inner loop can run very fast (kHz), giving crisp response
- The outer loop can be slower and still feel responsive

**Test bench simplification:** Single angle PID at 50 Hz. This works because the encoder has very low lag (~486 μs) and the lever's moment of inertia is small. A real drone with IMU lag and higher inertia needs the cascaded approach.

### 3. Encoder vs IMU as primary sensor

The test bench uses an AS5600 encoder with ~0.088° precision and ~486 μs total latency. A real drone has no such luxury — it relies on an IMU (gyro + accelerometer + magnetometer) with sensor fusion that introduces latency and drift.

**Test bench simplification:** The encoder provides near-perfect ground truth. Switching to the BNO085 IMU as the primary control input (future milestone) will expose the real challenges of IMU-based control: sensor lag, noise, drift, and the need for lag compensation.

### 4. Fixed pivot vs free flight

The mechanical pivot constrains all motion to one axis and removes the altitude problem. A real drone in free flight must also maintain altitude (a 4th PID for throttle/vertical velocity) and handle cross-axis coupling (roll affects yaw, etc.).

**Test bench simplification:** Gravity creates a restoring force through the pivot. On a real drone, gravity is purely adversarial — without active thrust, it falls. The `BASE_THROTTLE` on the bench just provides control authority; on a drone it fights gravity directly.

## Future Extensions

- Switch PID input from AS5600 to BNO085 IMU (primary and only control input). AS5600 becomes telemetry-only for logging angle error, lag analysis, and algorithm improvements
- Implement telemetry logging (angle, throttle, PID terms over time) for post-flight analysis
- Add D term and tune for tighter hold (±1° target)
- Implement gain scheduling or feedforward for gravity compensation if needed
