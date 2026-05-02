# DR-012: Differential Expo in LeverMixer

**Status:** Rejected (second attempt, 2026-05-02) — expo=0.3 worsened HoldMAE and produced elevated bias across two runs; rate_kd raised to 0.009 as the next lever (see Amendment 2026-05-01)
**Date:** 2026-04-09
**Context:** Post-rebuild baseline (2026-04-07) has 6.90° HoldMAE and 0.05 Hz oscillation near setpoint; PID gains are tuned for recovery authority from −58° start, which makes them too aggressive near horizontal.

## Context

The cascaded PID baseline (DR-008 Amendment 2026-04-07) exposes a fundamental tension in gain selection:

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

**Status for this bench:** deferred. The actual `T(throttle)` curve for this motor + prop combination at operating voltage has not been measured. DR-007 specifies a dedicated thrust bench (load cell, bidirectional DShot or external RPM, controlled voltage) to characterise each motor individually. Once that data exists, a true empirical inverse lookup replaces the heuristic. This is more rigorous than Betaflight's approach — BF uses a prior; we will use measurement.

### Problem B: Gain tension (gain scheduling)

The near-setpoint oscillation is primarily a **gain scheduling** problem. The gains required for recovery produce too-large corrections when the lever is near horizontal at small angle errors. The operating condition changes significantly between recovery and hold, but the gains do not.

This is an asymmetric problem: large commands should retain full authority; small commands should be disproportionately reduced.

## Decision

Implement **differential expo** — a cubic shaping function applied to the PID scalar before the per-motor split. This explicitly solves Problem B (gain scheduling) without claiming to solve Problem A (actuator physics).

### Why scalar (pre-split), not per-motor (post-split)

The two pipeline positions are genuinely different:

| | Scalar expo (this DR) | Per-motor correction (BF-style, DR-007) |
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

When these conditions hold, the differential-domain result is a good first-order approximation of per-motor correction. The bench currently satisfies all three. If motors diverge (wear, different props), or if base throttle changes significantly, this approximation degrades. Per-motor DR-007 curves handle all cases exactly.

### Choice of shaping function

The cubic blend `(1 − expo) × u + expo × u³` is a conventional gain-scheduling function — the same family used for RC expo in flight controllers. It is **not** derived from motor physics, and should not be interpreted as an approximation of the actuator inverse curve.

Properties:
- Passes through (0, 0) and (±1, ±1): endpoints are preserved
- Reduces midrange output proportionally to `expo`
- Smooth and monotonic across the full range

The cubic exponent (u³) produces more aggressive near-zero reduction than the motor thrust physics would suggest (`T ∝ u^1.5–2`). This is intentional: the function is chosen for scheduling properties, not physical fidelity. Physical fidelity is DR-007's goal.

### Pipeline position and saturation

```
rate_pid.compute()
      ↓  pid_output (scalar Δu)
[expo: normalise to ±1 per direction, apply cubic blend, denormalise]
  — positive Δu normalised by authority_up   (throttle_max − throttle_base)
  — negative Δu normalised by authority_down (throttle_base − throttle_min)
      ↓  shaped_output
m1 = throttle_base − shaped_output
m2 = throttle_base + shaped_output
[downstream motor clamp to [throttle_min, throttle_max]]
      ↓
ESC
```

Two distinct clamps are in play:

- **Internal normalisation clamp** (inside expo): clips the normalised scalar to [−1, +1] before the cubic blend is applied. Positive `pid_output` is normalised by `authority_up = throttle_max − throttle_base`; negative by `authority_down = throttle_base − throttle_min`. This correctly uses the full available authority in each direction. In the current configuration `authority_up = 200`, `authority_down = 500`.
- **Downstream motor clamp**: hard safety floor/ceiling on each motor command. Must remain in place post-expo.

**Clamp ordering (clamp-then-shape vs shape-then-clamp):** The normalisation clamp fires before the cubic blend is applied. An alternative would be to apply the cubic blend first and clamp the output afterwards. For this specific shaping function both orderings are equivalent: the cubic blend `(1 − expo) × t + expo × t³` maps `t = ±1` to exactly `±1` (substituting: `(1 − expo) × 1 + expo × 1 = 1`). For any `|t| ≤ 1` the internal clamp never fires and the orderings are identical. For `|t| > 1` the function with positive expo produces an output `> 1`, which the downstream clamp reduces to `1` regardless — again the same result. The ordering is therefore immaterial for this function and the current implementation (clamp-then-shape) is correct.

### Asymmetric normalisation — zero-crossing continuity and large-signal asymmetry

The two authorities are numerically different (`authority_up = 200`, `authority_down = 500`), so the normalisation step has a different slope on each side of zero. This raises the question of whether there is a gain discontinuity at zero crossing.

**There is no discontinuity.** The authority divides in and multiplies back out, so it cancels in the linear term:

```
output = f(t) × authority   where t = pid_output / authority

d(output)/d(pid_output) at t=0  =  f′(0) × authority × (1/authority)  =  f′(0)  =  (1 − expo)
```

The slope at zero is `(1 − expo)` on both sides — identical, regardless of authority. The function is smooth and continuous at the zero crossing.

**Asymmetry is present but confined to large deflections.** The two authority values determine how quickly each side reaches the nonlinear regime. A smaller `authority_up` means the positive side reaches high `t` (and therefore higher curvature) at lower absolute `pid_output` values than the negative side.

Effective gain `d(output)/d(pid_output) = (1 − expo) + 3 × expo × t²` at `expo=0.1`:

| `|pid_output|` | `t` positive (auth=200) | `t` negative (auth=500) | gain⁺ | gain⁻ | Δ |
|---|---|---|---|---|---|
| 20 | 0.100 | 0.040 | 0.903 | 0.900 | <0.5% |
| 65 | 0.325 | 0.130 | 0.932 | 0.905 | ~3% |
| 200 | 1.000 | 0.400 | 1.000 | 0.948 | ~5% |

**Near setpoint (small `pid_output`), the response is effectively symmetric** — the asymmetry is negligible where expo is intended to act. The asymmetry grows with deflection magnitude and is most significant at full recovery authority, where the positive side (smaller `authority_up`) is shaped more aggressively. This is the mechanism behind the recovery authority loss observed on 2026-04-10: positive corrections are expo-reduced at lower absolute magnitudes than negative ones. Raising `ANGLE_RATE_LIMIT` compensates directly.

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

The target interface once DR-007 thrust curves are available:

```
Δu → split into m1_raw/m2_raw → per-motor f⁻¹(u_i) → clamp
```

Migration inside `LeverMixer`:

1. The per-motor split stays in place (`m1 = base − Δu`, `m2 = base + Δu`).
2. The scalar expo is removed or kept as an independent gain-scheduling layer.
3. Each raw motor command is passed through a per-motor inverse lookup table derived from DR-007 measurements.
4. The downstream clamp remains unchanged.

The current `LeverMixer` class structure supports this transition: the split and clamp logic are already separate from the expo shaping. No interface change in `main.py` is required — the migration is internal to the mixer.

Both corrections (scalar gain scheduling + per-motor actuator compensation) can coexist and are independently tunable.

## Outcome

### Initial test runs (2026-04-09) — inconclusive

Three expo values were tested against the expo=0.0 post-tare baseline (run `2026-04-09_18-54-22`: 6.02° HoldMAE, 0.05 Hz oscillation):

| expo | HoldMAE | Oscillation | T→0 |
|------|---------|-------------|-----|
| 0.0 (baseline) | 6.02° | 0.05 Hz | 1.6s |
| 0.1 | 7.43° / 8.29° | 0.09 / 0.05 Hz | 14.8s / 3.2s |
| 0.2 | 6.74° | 0.27 Hz | 4.6s |
| 0.3 | 7.34° | 0.38 Hz | 1.0s |

All tested values showed worse HoldMAE than baseline. However, these results were obtained with a **normalisation bug**: `LeverMixer` used `throttle_max − throttle_base` (200) as the normalisation limit in both directions, instead of the correct per-direction authorities (`authority_up = 200`, `authority_down = 500`). With the symmetric-but-wrong limit, negative `pid_output` was being normalised against a range less than half its actual authority — the expo curve was applied over a distorted domain in the negative direction, producing asymmetric shaping that was not intended.

The bug was fixed: `LeverMixer` now normalises positive `pid_output` by `authority_up` and negative by `authority_down`. All prior expo test results should be considered invalid.

**Re-test required** with the corrected implementation before any conclusion about expo efficacy can be drawn.

### Re-test at expo=0.1, ANGLE_RATE_LIMIT=130 (2026-04-10) — recovery authority insufficient

First re-test with the corrected normalisation. Lever reached −12.8° at best — just outside the ±10° threshold.

Root cause: at `ANGLE_RATE_LIMIT=130`, the rate PID produces at most `0.5 × 130 = 65` throttle units. After expo=0.1 shaping (`t = 65/200 = 0.325` → shaped ≈ 59 units), the differential is ~118 throttle units → ~17.3g — below the ~18g resistance floor. `ANGLE_RATE_LIMIT` raised to 145: raw output = 72.5 → after expo ≈ 66 → ~19.4g, restoring margin.

### Re-test at expo=0.1, ANGLE_RATE_LIMIT=145 (2026-04-10) — hold performance worse

| Run | expo | ANGLE_RATE_LIMIT | T→0 | HoldMAE | T@0 | Oscillation |
|-----|------|-----------------|-----|---------|-----|-------------|
| 2026-04-09_18-54-22 (baseline) | 0.0 | 130 | 1.6s | 6.02° | — | 0.05 Hz |
| 2026-04-10_10-05-15 | 0.1 | 130 | DNF | — | — | — |
| 2026-04-10_11-41-09 | 0.1 | 145 | 5.6s | **12.01°** | 34.6s/103.6s | 0.01 Hz |

Recovery works. Hold is dramatically worse: HoldMAE doubled (6.02° → 12.01°), the lever spent two-thirds of the run outside ±10°, and the ENC bias of −12.31° shows it was parked at ~−12° as a steady-state error rather than oscillating around 0°. Oscillation frequency dropped from 0.05 Hz to 0.01 Hz — slower but much larger amplitude.

### Why positive expo makes hold worse

Positive expo reduces near-setpoint P gain by `(1 − expo)`. This was intended to reduce hunting. Instead it creates a new equilibrium below horizontal: the expo-reduced corrective force at intermediate angles (~−12° operating point, t≈0.2) is small enough that friction and cable tension balance it before the lever reaches 0°. The integrator accumulates but cannot overcome this because the expo reduces the force delivered for any given integrator value. The result is a slow large-amplitude drift rather than the original tight fast oscillation — worse on every hold metric.

### Conclusion: wrong diagnosis

The doubled HoldMAE, slow large-amplitude drift, and steady-state offset at ~−12° are the **classic symptoms of too little DC gain** — insufficient authority at small errors. This is the opposite symptom pattern from too much gain (which produces fast, tight oscillations around the setpoint). Positive expo pushed the system in the wrong direction: it reduced DC gain when the system already had just enough.

The 0.05 Hz near-setpoint oscillation is a **damping problem**, not a gain magnitude problem. Reducing P gain (expo) does not add damping — it only makes corrections smaller. The correct lever is `rate_kd`: derivative action damps oscillations by opposing the rate of change, without reducing the DC corrective force that holds the lever at horizontal.

This also confirms the Problem A / Problem B distinction holds: Betaflight's `thrust_linear` boosts small per-motor commands to compensate for actuator nonlinearity (correct for Problem A). Our expo reduces small differential commands (intended for Problem B). They are opposite in direction and operate at different pipeline positions. This DR's framing of these as separate problems requiring separate treatments is validated.

Next step: `rate_kd` raised from 0.003 to 0.006, `expo=0.0` restored, `ANGLE_RATE_LIMIT` reverted to 130. The `linearise_thrust` method and asymmetric authority infrastructure remain in `LeverMixer` — they are correct and may be useful if a future approach (anti-expo, or per-motor compensation per DR-007) requires them.

## Amendment 2026-04-11 — Wire interference note

The friction sweep data that informed force budget reasoning in this DR (via DR-013) was collected on 2026-04-10 with IMU wires attached. Those measurements are invalidated — see DR-013 Amendment 2026-04-11 for details.

The DR-012 conclusions are unaffected: the expo rejection was driven by the doubled HoldMAE and steady-state offset in flight runs, not by force budget estimates. The ~18g figure used in the recovery authority analysis was derived from flight-run behavior (with all wires attached) and remained accurate for that configuration. Future characterization and force budget analysis should use the clean ESC-wires-only sweep data from 2026-04-11.

## Consequences

**Positive:**
- Directly addresses gain tension (Problem B) without conflating it with actuator physics (Problem A).
- PID structural integrity preserved — integral accumulates from error, not shaped output.
- `expo=0.0` is a no-op — zero regression risk.
- Clear separation of concerns: this DR solves gain scheduling; DR-007 will provide the empirical basis for actuator compensation that is more rigorous than Betaflight's heuristic approach.
- Current `LeverMixer` structure supports clean Phase 2 migration with no interface changes required.

**Negative:**
- Does not compensate for motor nonlinearity — deferred to DR-007.
- Scalar (pre-split) shaping is a first-order approximation of per-motor correction; valid under current symmetry assumptions, degrades if motors diverge.
- Cubic curve is a scheduling heuristic, not derived from motor physics.
- Effective closed-loop gain varies with operating point; I-term behaviour near setpoint differs from during recovery.
- Normalisation is asymmetric per direction (`authority_up ≠ authority_down`); downstream clamp is load-bearing and must remain.

## Amendment 2026-05-01 — Second expo attempt

### Why the previous rejection does not preclude re-testing

The April 2026 expo attempt failed for a specific and well-understood reason: expo reduced near-setpoint corrective force below the friction threshold, producing a steady-state offset of ~−12° and doubled HoldMAE. That failure was a DC-gain problem, not a general indictment of expo.

The system state has changed materially since then:

| Metric | April 9-10 baseline | 2026-05-01 best run |
|--------|--------------------|--------------------|
| HoldMAE | 6.02° | **2.46°** |
| Encoder bias | Not reported | **0.02°** (negligible) |
| Oscillation frequency | 0.05 Hz (slow, large amplitude) | **1.38 Hz** (tight, fast) |
| rate_kd | 0.003 | **0.006** (doubled per DR-012 recommendation) |

The April failure's signature was slow large-amplitude drift with a large steady-state offset — the DC-gain-below-friction pattern. The current signature is symmetric tight wobble at 1.38 Hz with negligible bias. These are different physical regimes.

**1.38 Hz tight oscillation is the near-setpoint gain-excess signature** — the pattern expo was originally designed to address. The April failure occurred because the system had too little DC gain margin; the current run has substantial margin (0.02° bias, 2.46° MAE, strong hold).

### Why rate_kd is not the only option

DR-012's original conclusion ("correct lever is rate_kd") was right for the April state. Rate_kd was raised from 0.003 to 0.006 and the improvement was dramatic (6.02° → 2.46°). However, a further kd increase is not guaranteed to continue reducing the oscillation:

- rate_kd=0.006 with the current system produces 1.38 Hz oscillation.
- Further kd increase risks exciting higher-frequency modes (sensor noise amplification in the derivative term).
- Expo and rate_kd address different aspects: kd damps the rate of change; expo reduces the amplitude of corrective commands near zero. Both can reduce oscillation amplitude but via different mechanisms.

Expo is the lower-risk lever to try next: it leaves rate_kd unchanged, is easily reversed (set to 0.0), and directly targets the near-setpoint gain excess.

### Test plan (second attempt)

Starting conditions: `angle_pid.kp=3.5, rate_pid.kd=0.006, expo=0.0` (run `2026-05-01_19-19-05` baseline).

| Step | expo | Stop if |
|------|------|---------|
| 1 | 0.3 | HoldMAE increases OR steady-state bias > 1° |
| 2 | 0.5 | HoldMAE increases OR steady-state bias > 1° |
| 3 | 0.7 | HoldMAE increases OR steady-state bias > 1° |

The steady-state bias watch is the critical gate: if bias rises above ~1°, the system is approaching the friction-floor condition that caused the April failure. Revert to expo=0.0 immediately.

Success target: HoldMAE < 2.0°, T→0 ≤ 3.5s, bias < 0.5°.

If expo stalls at ~2.0° with no bias regression: consider rate_kd 0.006 → 0.009 as the next lever.

### Outcome (2026-05-02)

Two runs at expo=0.3 (expo=0.0 baseline: 2.46° HoldMAE, 0.02° bias, 1.38 Hz):

| Run | HoldMAE | Bias | T→0 | Stop condition |
|-----|---------|------|------|----------------|
| 2026-05-01_21-57-19 | 3.56° | 2.37° | 15.7s | Bias > 1° AND HoldMAE increased |
| 2026-05-02_08-23-41 | 3.49° | 0.39° | 9.0s | HoldMAE increased |

Both stop conditions from the test plan were met. expo=0.3 consistently worsened HoldMAE relative to baseline and produced large T→0 degradation. The session-to-session bias variation (2.37° vs 0.39° at identical config) indicates the system is near the friction-floor boundary — small session-to-session differences in initial conditions tip it between acceptable and unacceptable hold.

Expo re-test is closed. Next lever: rate_kd 0.006 → 0.009 (expo=0.0 restored). First results at rate_kd=0.009 in runs 2026-05-02_09-17-54 (2.60° HoldMAE, 0.74 Hz) and 2026-05-02_10-57-21 (3.63° HoldMAE, 1.08° bias).