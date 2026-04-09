# ADR-012: Differential Expo in LeverMixer

**Status:** Proposed
**Date:** 2026-04-09
**Context:** Post-rebuild baseline (2026-04-07) has 6.90° HoldMAE and 0.05 Hz oscillation near setpoint; PID gains are tuned for recovery authority from −58° start, which makes them too aggressive near horizontal.

## Context

The cascaded PID baseline (ADR-008 Amendment 2026-04-07) exposes a fundamental tension in gain selection:

- **Recovery requires high authority**: at −58° start, wire tension + bearing friction (~18g) must be overcome. `angle_kp=3.5` saturates `ANGLE_RATE_LIMIT` (130 deg/s) at the start position, producing sufficient differential thrust to reach horizontal in 1.3s.
- **Near setpoint, the same gains cause oscillation**: at small angle errors (±10°), those same gains produce small but still forceful differential commands, leading to 0.05 Hz hold oscillation and 6.90° HoldMAE.

There is no mechanism in the current linear mixer to reduce near-setpoint sensitivity without proportionally reducing recovery authority. Something must be made nonlinear.

## Two distinct problems — two distinct solutions

**Actuator nonlinearity** and **gain tension** are different problems requiring different treatments and different pipeline positions.

### Problem A: Actuator nonlinearity (motor + propeller physics)

Motor thrust follows approximately `T ≈ k × throttle^n`. The **actuator** is the source of this nonlinearity — not the mixer. The mixer is linear by design; it correctly converts PID output into per-motor commands using arithmetic. The problem is that the motors' physical response to those commands is nonlinear.

Betaflight's `thrust_linear` applies a heuristic correction **after the mixer, per motor, independently on each absolute throttle value**:

```
PID → Mixer → [per-motor correction on u_i] → ESC
```

Importantly, a single `thrust_linear` coefficient is shared across all motors, but it is **applied independently per motor** — not once to a global scalar. This is architecturally different from applying a correction to the pre-split differential.

The correction direction is also significant: it *boosts* small throttle commands to compensate for the disproportionately low thrust they produce. This is the opposite of our gain scheduling function, which *reduces* near-setpoint differential commands.

**What Betaflight actually knows:** The flight controller sends a DShot/PWM signal but does not measure thrust. Even with bidirectional DShot (which provides RPM telemetry), Betaflight only knows `input → RPM` — not `RPM → thrust`. Thrust depends on propeller geometry, air density, battery sag, and frame interactions, none of which are observed in flight. Betaflight's `thrust_linear` is therefore a **heuristic based on a physics prior** (`thrust ∝ input²` approximately), not an inversion of a measured thrust curve. It is a "good enough generic correction" that works because the prior is usually in the right direction for prop-driven systems.

**Status for this bench:** deferred. The actual `T(throttle)` curve for this motor + prop combination at operating voltage has not been measured. ADR-007 specifies a dedicated thrust bench (load cell, bidirectional DShot or external RPM, controlled voltage) to characterise each motor individually. Once that data exists, a true empirical inverse lookup replaces the heuristic. This is more rigorous than Betaflight's approach — BF uses a prior; we will use measurement.

### Problem B: Gain tension (gain scheduling)

The near-setpoint oscillation is primarily a **gain scheduling** problem. The gains required for recovery produce too-large corrections when the lever is near horizontal at small angle errors. The operating condition changes significantly between recovery and hold, but the gains do not.

This is an asymmetric problem: large commands should retain full authority; small commands should be disproportionately reduced.

## Decision

Implement **differential expo** — a cubic shaping function applied to the PID scalar before the per-motor split. This explicitly solves Problem B (gain scheduling) without claiming to solve Problem A (actuator physics).

### Why scalar (pre-split), not per-motor (post-split)

The two pipeline positions are genuinely different:

| | Scalar expo (this ADR) | Per-motor correction (BF-style, ADR-007) |
|---|---|---|
| Operates on | Differential scalar Δu | Absolute throttle per motor u_i |
| Solves | Gain tension | Actuator nonlinearity |
| Direction | Reduces small commands | Boosts small commands |
| Parameter scope | One coefficient, applied once | One coefficient, applied independently per motor |
| Requires | No thrust data | Measured T(throttle) curve |

For gain scheduling (Problem B), the differential signal is the natural domain: the oscillation is caused by the differential being too aggressive, not by each motor's absolute command being on a nonlinear part of its curve. Shaping the differential directly addresses the problem at the right level.

Actuator compensation (Problem A) belongs post-split and must operate on absolute throttle values, because that is where the motor's physics lives. Per-motor application is also necessary to handle unit-to-unit asymmetry.

### Why differential-domain shaping is a first-order approximation

The physics operates on absolute thrust per motor:

```
Torque ∝ T(m2) − T(m1)   where   T_i = f(u_i)
```

Correct per-motor compensation applies the inverse of `f` to each motor command individually. Differential-domain shaping applies a function to Δu before splitting:

```
Δu → g(Δu)   then   m1 = base − g(Δu),   m2 = base + g(Δu)
```

These are not equivalent: `g(Δu) ≠ f⁻¹(base + Δu) − base` in general.

**Validity conditions for the scalar approximation:**
- System is symmetric (both motors matched, identical thrust curves)
- Base throttle is stable (operating point does not vary widely)
- Focus is on differential thrust behaviour, not absolute thrust accuracy

When these conditions hold, the differential-domain result is a good first-order approximation of per-motor correction. The bench currently satisfies all three. If motors diverge (wear, different props), or if base throttle changes significantly, this approximation degrades. Per-motor ADR-007 curves handle all cases exactly.

### Choice of shaping function

The cubic blend `(1 − expo) × u + expo × u³` is a conventional gain-scheduling function — the same family used for RC expo in flight controllers. It is **not** derived from motor physics, and should not be interpreted as an approximation of the actuator inverse curve.

Properties:
- Passes through (0, 0) and (±1, ±1): endpoints are preserved
- Reduces midrange output proportionally to `expo`
- Smooth and monotonic across the full range

The cubic exponent (u³) produces more aggressive near-zero reduction than the motor thrust physics would suggest (`T ∝ u^1.5–2`). This is intentional: the function is chosen for scheduling properties, not physical fidelity. Physical fidelity is ADR-007's goal.

### Pipeline position and saturation

```
rate_pid.compute()
      ↓  pid_output (scalar Δu)
[expo: normalise to ±1, apply cubic blend, denormalise]
  — internal normalisation clamp limits shaped output to ±expo_limit
      ↓  shaped_output
m1 = base − shaped_output
m2 = base + shaped_output
[downstream motor clamp to [throttle_min, throttle_max]]
      ↓
ESC
```

Two distinct clamps are in play:

- **Internal normalisation clamp** (inside expo): limits `shaped_output` to `±expo_limit` where `expo_limit = throttle_max − base`. This handles the upper end symmetrically.
- **Downstream motor clamp**: handles the lower end (where `base − throttle_min` may exceed `expo_limit`) and provides a hard safety floor/ceiling.

The normalisation range (`throttle_max − base`) is not symmetric around base — in the current configuration `throttle_max − base = 200` while `base − throttle_min = 500`. Commands that would push a motor below `throttle_min` are handled by the downstream clamp, not by the expo normalisation. The downstream clamp must remain in place and post-expo. This ordering is correct.

### Effect on PID structure

The expo is applied after the PID, so the integral accumulates based on the unmodified error signal. This preserves the **structural integrity** of the PID (integral is not corrupted by the shaping function). However, since the expo reduces effective plant gain near setpoint, the closed-loop system is not strictly linear: effective gain varies with operating point. Near setpoint a given integrator value produces less corrective force than it would in the linear case. This is the inherent characteristic of gain scheduling — intentional and expected. Monitor for integrator behaviour differences near setpoint vs during recovery.

### Parameters

| Parameter | Initial value | Notes |
|-----------|--------------|-------|
| `expo` | `0.0` (off) | First test at `0.3`; tune in 0.1 steps |
| normalisation range | `throttle_max − base` | Natural full-authority differential; derived, not separately tunable |

`expo=0.0` is an exact no-op — full regression safety.

## Validation plan

1. Run at `expo=0.0` — confirm identical KPIs to 2026-04-07 baseline.
2. Enable `expo=0.3` — compare HoldMAE, oscillation frequency, time-to-reach.
3. If HoldMAE improves without significant time-to-reach degradation, accept as new baseline.
4. If time-to-reach degrades, compensate by raising `ANGLE_RATE_LIMIT` (restores recovery authority without affecting near-setpoint behaviour).

## Migration path to per-motor compensation (Phase 2)

The current interface:

```
Δu → expo(Δu) → split into m1/m2 → clamp
```

The target interface once ADR-007 thrust curves are available:

```
Δu → split into m1_raw/m2_raw → per-motor f⁻¹(u_i) → clamp
```

Migration inside `LeverMixer`:

1. The per-motor split stays in place (`m1 = base − Δu`, `m2 = base + Δu`).
2. The scalar expo is removed or kept as an independent gain-scheduling layer.
3. Each raw motor command is passed through a per-motor inverse lookup table derived from ADR-007 measurements.
4. The downstream clamp remains unchanged.

The current `LeverMixer` class structure supports this transition: the split and clamp logic are already separate from the expo shaping. No interface change in `main.py` is required — the migration is internal to the mixer.

Both corrections (scalar gain scheduling + per-motor actuator compensation) can coexist and are independently tunable.

## Consequences

**Positive:**
- Directly addresses gain tension (Problem B) without conflating it with actuator physics (Problem A).
- PID structural integrity preserved — integral accumulates from error, not shaped output.
- `expo=0.0` is a no-op — zero regression risk.
- Clear separation of concerns: this ADR solves gain scheduling; ADR-007 will provide the empirical basis for actuator compensation that is more rigorous than Betaflight's heuristic approach.
- Current `LeverMixer` structure supports clean Phase 2 migration with no interface changes required.

**Negative:**
- Does not compensate for motor nonlinearity — deferred to ADR-007.
- Scalar (pre-split) shaping is a first-order approximation of per-motor correction; valid under current symmetry assumptions, degrades if motors diverge.
- Cubic curve is a scheduling heuristic, not derived from motor physics.
- Effective closed-loop gain varies with operating point; I-term behaviour near setpoint differs from during recovery.
- Normalisation range is asymmetric (`throttle_max − base ≠ base − throttle_min`); downstream clamp is load-bearing and must remain.