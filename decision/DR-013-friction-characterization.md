# DR-013: Friction and Resistance Characterization

**Status:** Superseded (2026-04-11) — 2026-04-10 sweep results invalidated by IMU wire interference; re-measurement required
**Date:** 2026-04-10

## Context

The force budget in DR-008 Amendment 2026-04-07 estimated ~18g of resistance (wire tension + bearing friction) at the −57° start position, inferred from the minimum PID output required to initiate recovery. This was not a direct measurement — it was derived from observing that the system barely achieved recovery at `ANGLE_RATE_LIMIT=130`, `rate_kp=0.5`, producing a differential of ~130 throttle units → ~19g. The assumption was that 19g was just sufficient to overcome 18g of resistance.

DR-012 testing revealed the estimate matters: expo=0.1 reduced the differential below the assumed floor and initially prevented recovery. Before adding any friction-compensating mechanisms (deadband compensator, revised integrator tuning), the actual resistance profile needed to be measured directly.

**Lever arm geometry:**

| Frame | Motor-to-pivot distance | Relative torque per gram of thrust |
|-------|------------------------|-------------------------------------|
| Old (Al + PLA, retired 2026-04-03) | 9.25 cm | 2.64× |
| **New (CF + PETG, current)** | **3.5 cm** | **1.0×** |

The new frame has a 2.64× shorter moment arm. The same motor thrust produces 2.64× less corrective torque; the same resistive torque at the pivot requires 2.64× more motor force to overcome. This is a major contributor to why post-rebuild gains needed aggressive retuning — the motors are fighting the same bearing friction and cable tension but with much less mechanical advantage.

**Sources of resistance on this bench:**
- Bearing friction at the pivot (relatively constant with angle)
- Cable tension: 6 × ESC-to-motor wires hang down from static frame to rotating motor mounts; 6 × I2C hub wires similarly routed. Cable routing is not symmetric — ESC wires and I2C wires hang on different sides, so resistance may differ by direction.

## Experiment

**Tool:** `tools/test_friction_sweep.py` — open-loop single-motor logger. One motor physically disconnected; the active motor holds a fixed throttle for 10s while encoder position is logged at 20 Hz.

**Procedure:** Start lever at the restrictor (active motor at its lowest position). Apply fixed throttle. Observe where the lever settles — equilibrium angle is where motor thrust equals kinetic friction at that position. Sweep throttle values to build a profile. Data stored in `test_runs/free_sweep/`.

## Results

### M2 pushing — M2 at −57°, lever rises toward +58° (2026-04-10)

| Throttle | Equilibrium | Travel | Outcome |
|----------|-------------|--------|---------|
| 200 | −56.5° | +1.1° | barely lifts off restrictor |
| 210 | −43.5° | +14.1° | balances at −43° |
| 220 | −22.7° | +35.2° | balances at −23° |
| 230 | +58.7° | +116° | **full sweep to other restrictor** |
| 240 | +58.6° | +116° | full sweep |
| 245 | +58.7° | +116° | full sweep |
| 250 | +58.7° | +116° | full sweep |

Tipping point: **between throttle 220 and 230**. The progressive equilibrium positions (−43°, −23°, full sweep) reveal that resistance builds continuously as the lever moves away from the natural rest position — cable tension accumulates through the stroke. At throttle 230, the motor overcomes this peak everywhere.

### M1 pushing — M1 at +58°, lever descends toward −57° (2026-04-10)

| Throttle | Equilibrium | Travel | Outcome |
|----------|-------------|--------|---------|
| 200 | +54.0° | +5.0° | barely lifts off restrictor |
| 210 | −57.5° | +116° | **full sweep to other restrictor** |
| 220 | −57.4° | +116° | full sweep |
| 230 | −57.7° | +116° | full sweep |
| 240 | −57.6° | +116° | full sweep |
| 250 | −57.6° | +116° | full sweep |

Tipping point: **between throttle 200 and 210**. M1 direction has significantly lower resistance than M2.

*Note on throttle 245:* started from +53° instead of the +58° restrictor; lever stopped at −45.7° after 10s (98.8° of travel). This is a time artifact — the 10s window ended while the lever was still moving. Not used for friction estimation.

## Analysis

### Friction force estimates (power-law, pending scale confirmation)

Using measured thrust at BASE=600 (~45g/motor) and typical exponent n≈1.8:

| Direction | Tipping throttle | Estimated peak friction | vs 18g assumption |
|-----------|-----------------|------------------------|-------------------|
| M1 push (normal recovery) | ~205 | **≈ 6.5g** | 2.8× lower |
| M2 push (opposite) | ~225 | **≈ 7.7g** | 2.3× lower |

These are power-law estimates. Kitchen scale measurements will replace them with exact values.

### Asymmetry between directions

M1-push tipping point (~205) is significantly lower than M2-push (~225). The resistance in the normal recovery direction (M1 from top, descending) is roughly 20% less than the reverse.

Likely cause: cable routing. The ESC-to-motor wires and I2C hub wires hang differently on each side of the pivot. As M1 descends from +58°, the cables on the M1 side relax (motor approaches their natural hang point) while M2 cables stretch — a geometry that partially assists M1 descent. The M2-push direction works against this: both M2 cable stretch and M1 cable stretch oppose the motion as the lever travels further from the −57° rest position.

The progressive M2 equilibrium positions (−43°, −23°, then full sweep at 230) confirm that resistance builds continuously through the stroke, not just at one peak angle.

### Implications for differential control

In the two-motor differential setup, net force = `2 × pid_output × 0.147 g/unit`. To overcome the friction peaks:

| Direction | Peak friction | Minimum pid_output needed | Corresponding angle error |
|-----------|--------------|--------------------------|--------------------------|
| M1 push (recovery) | ~6.5g | ~22 units | ~13° |
| M2 push | ~7.7g | ~26 units | ~15° |

For angle errors smaller than ~13–15°, the P-term alone cannot overcome friction; the integrator must carry the correction. This explains the ~5° residual HoldMAE: small corrections near horizontal are below the P-term friction floor, and the integrator oscillates slowly as it accumulates and releases authority.

### What the 18g estimate actually measured

The original 18g estimate was not simply wrong — it was capturing **static friction at the bottom of the stroke** (−57°), where stiction must first be broken before any motion occurs. That is a worst-case one-time event at the start of each run. The sweep measurements here capture the kinetic friction profile across the full arc, which is consistently lower. The 18g static stiction at the start position is consistent with the motors barely breaking away at throttle 200 (M1) and 200 (M2).

## Pending measurements

### Finer resolution around tipping points (optional)

M1: one data point between 200 (barely moves) and 210 (full sweep) would pin down the tipping throttle more precisely. M2: one point between 220 (stops at −23°) and 230 (full sweep). For control system design the current resolution is likely sufficient; the kitchen scale measurements will give the exact force values regardless.

### Direct thrust measurement (kitchen scale)

The current 10g peak friction estimate uses a power-law approximation for motor thrust at throttle 260. A kitchen scale measurement eliminates this uncertainty:

1. Fix motor horizontally, propeller pointing down onto scale
2. Record thrust at throttle: 200, 240, 250, 260, 280, 300, 400, 500, 600
3. Repeat for both motors (unit-to-unit variation)
4. Thrust at throttle 260 = exact friction peak value, no estimation

This also produces the thrust curve data required by DR-007 for the current motor + prop combination, which is foundational for any future per-motor actuator compensation.

## Consequences

**Positive:**
- The 18g floor in force budget calculations should be revised: peak kinetic friction is ~6.5g (M1 direction) and ~7.7g (M2 direction), both well below the assumed 18g.
- Recovery authority at `pid_output=65` → ~19g differential has ~3× margin over the kinetic friction peak in the normal recovery direction, not the ~5% margin the 18g estimate implied.
- The residual ~5° HoldMAE is explained: P-term alone is below the friction floor for angle errors smaller than ~13°; integrator dependency near horizontal is structural, not a tuning failure.
- The asymmetry between directions is confirmed and quantified — any future deadband compensator needs separate parameters for the two directions.

**Implications for future work:**
- Increasing `angle_ki` modestly (0.05 → 0.07–0.08) may reduce residual hold offset by letting the integrator overcome near-horizontal friction faster.
- A deadband/friction compensator is feasible and now has a measured target: peak friction ~6.5g in the recovery direction. Compensator parameter `b` in throttle units ≈ 6.5 / 0.147 ≈ 44 units (subject to scale measurement confirmation).
- All force budget calculations in DR-008 and DR-012 that reference 18g should be read as **static stiction at −57°**, not kinetic friction across the arc. The operational resistance is 2–3× lower.

## Retrospective

This characterisation should have been done before any control tuning on the new frame. Instead, the friction level was inferred from the minimum PID output that achieved recovery — an indirect and fragile estimate that conflated static friction at one position with the full resistance profile. That 18g number then propagated into force budget analysis (DR-008), expo authority calculations (DR-012), and several tuning decisions.

A 30-minute open-loop sweep with one motor disconnected replaced weeks of assumption with direct measurement. The lesson: **characterise the physical plant before tuning the controller**. For this bench specifically:

1. **Thrust curve first** (DR-007, still pending) — measure T(throttle) per motor before reasoning about force budgets.
2. **Friction profile second** (this DR) — one-time sweep, directly bounds what the controller needs to overcome.
3. **Lever geometry** — the 2.64× shorter moment arm on the new frame was noted at rebuild time but its implication for the force budget was not carried through to the tuning decisions that followed.

These are properties of the physical plant that do not change with software. Knowing them upfront would have avoided the circular tuning-and-testing loop that characterised DR-012.

## Amendment 2026-04-11 — Results invalidated by IMU wire interference

The 2026-04-10 sweeps were performed with the full wiring harness attached — including the 6 IMU wires (I2C SDA/SCL, VCC, GND, RST, INT) routed from the static frame to the rotating motor mount. These wires hang outside the rotation axis with no slip ring. As the lever rotates away from the natural rest position, they flex and build tension, contributing a continuous mechanical torque bias.

Repeat sweeps on 2026-04-11 with IMU wires removed — ESC wires only (3 wires per motor, 6 total) — reveal that the IMU wire bundle was the dominant source of the directional asymmetry and the progressive equilibria observed on 2026-04-10.

### Clean sweep results (ESC wires only, 2026-04-11)

**M1 pushing from +58.9° toward −57° (runs `2026-04-11_14-17-05`):**

| Throttle | Equilibrium | Outcome |
|----------|-------------|---------|
| 200–280  | +58.9°      | no movement |
| 290      | −57.3°      | full sweep to other restrictor |

**M2 pushing from −57.3° toward +59° (run `2026-04-11_14-19-54`):**

| Throttle | Equilibrium | Outcome |
|----------|-------------|---------|
| 200–280  | −57.3°      | no movement |
| 290      | +59.0°      | full sweep to other restrictor |

### What the clean data shows

**Symmetric thresholds.** Both motors flip at throttle ~290. The 2026-04-10 asymmetry (M1: ~205, M2: ~225) was entirely wire-induced: the IMU wires biased the lever toward M1-end-down, artificially assisting M1 and opposing M2.

**No progressive equilibria.** Without the distance-dependent IMU wire tension, neither motor produces intermediate settling positions. The lever either stays put or does a full sweep. The gradual progression (−43°, −23°, full sweep) observed for M2 on 2026-04-10 was driven by IMU wire tension increasing through the stroke, not by bearing friction.

**IMU wire torque magnitude.** The wire tension was equivalent to roughly 85–90 DShot units of motor force in the M1-down direction — sufficient to shift M1's flip threshold from the true ~290 down to ≤200, and to enable continuous M2 equilibria across a 30-unit throttle range.

### Invalidated conclusions

All quantitative results from the 2026-04-10 sweeps are invalidated:

- The tipping points (M1: ~205, M2: ~225) are not properties of bearing friction; they are the points at which motor force overcame the combined resistance of bearing friction *and* IMU wire tension.
- The friction force estimates (6.5g M1, 7.7g M2) were derived from those tipping points and are therefore incorrect.
- The 20% directional asymmetry attributed to cable routing geometry was largely IMU wire interference, not bearing friction asymmetry.
- The implication that "increasing `angle_ki` to 0.07–0.08 may reduce hold offset" and the deadband compensator target parameter (~44 throttle units) were derived from these estimates and should not be used without re-measurement.

### What remains valid from the original characterisation

- The general principle that resistance builds through the stroke due to cable tension is confirmed — it now applies to ESC wires only, with much smaller magnitude.
- The observation that static stiction at −57° must be broken before motion occurs remains valid; the 18g estimate in DR-008 was inferred from flight behavior with all wires attached and remains accurate for that configuration.
- The lever geometry analysis (2.64× shorter moment arm than old frame) is unaffected.
- The retrospective lesson — characterise the plant before tuning the controller — stands, with the added corollary: **isolate mechanical interference sources before characterising**. Wire routing is part of the plant; an uncontrolled harness contaminates any open-loop measurement.

### Re-measurement required

Friction characterization must be repeated with ESC wires only. The clean sweep data establishes a new reference point: single-motor flip threshold at throttle ~290 against bearing friction alone. Exact force values require direct scale measurement (DR-007, still pending).
