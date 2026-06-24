---
title: Cascaded Control Loop Implementation Review
date: 2026-06-10let'
reviewer: Claude Fable 5 (Anthropic) — via Claude Code
review_type: deep control-engineering review ("would I fly with this?")
scope: src/flight.py, src/pid.py, src/mixer.py, src/ui.py, src/telemetry/recorder.py,
  driver contract boundaries (BNO085, DShot MotorThrottleGroup)
evidence_run: test_runs/flights/2026-06-07_13-48-31
verification_tool: tmp/check_gyro_sign.py (to be promoted per QA-I)
status: adopted as backlog
---

# REVIEW-2026-06-10 — Cascaded Control Loop Implementation Review

**Scope:** `src/flight.py` (entry: `stabilize()` and `run()`), `src/pid.py`, `src/mixer.py`,
`src/ui.py`, `src/telemetry/recorder.py`, and the contract boundaries with
`dependencies/BNO085/driver/{bno08x,i2c}.py` and `dependencies/DShot/driver/motor_throttle_group.py`.

**Method:** Static review of the full control path (sensor -> loops -> mixer -> motors ->
shutdown), plus empirical verification of every sign-related claim against the telemetry of
run `2026-06-07_13-48-31` (1545 records, 119.9 s). The verification script is preserved at
`tmp/check_gyro_sign.py` and can be re-pointed at any run folder.

**Perspective:** "Would I fly with this?" — i.e. every finding is judged not by whether the
bench currently works (it does), but by what happens in the corner cases: sensor death,
saturation, config drift, mechanical change, and operator interaction at the worst moment.

**Verdict in one paragraph:** The cascade is stable and the tuning is genuinely good. But it is
stable *despite* two latent defects (an inverted feedforward term and a 10% dt arithmetic
error) that have been silently absorbed into the tuned gains, and it is safe only as long as
nothing fails: there is no abort path, no sensor-staleness watchdog, no envelope protection,
and the shutdown sequence leaves motors at flight throttle for seconds — and can skip disarm
entirely if the SD card is what failed. The findings below are ordered by the order I would
fix them in.

---

## Part 0 — The sign-convention primer (read this first)

Several findings (F1, F2, F9) hang off one structural fact about this codebase, so it is
worth deriving once, carefully. This is the "inversion mess" — here is its exact shape.

### 0.1 Definitions

Let `phi` be the lever angle in **encoder convention**: positive = M1 end lower. This is the
convention of `ENC_ROLL` in telemetry, of `bench.start_angle_deg`, and of the setpoint in
config. Let `phi_dot` = d(phi)/dt be the angular rate in the same convention.

### 0.2 The plant sign

Thrust pushes a motor end DOWN (confirmed by single-motor test 2026-04-06). The mixer is:

    m1 = base - output
    m2 = base + output

So positive `output` -> more thrust on M2 -> M2 end pushed down -> M1 end rises -> `phi`
decreases. The plant gain from `output` to angular acceleration is **negative**:

    phi_ddot = -c * output        (c > 0, lumping inertia and thrust gain)

### 0.3 What the rate loop requires for stability

The inner loop computes:

    rate_error = rate_setpoint - gyro_x
    output     = kp_r * rate_error + (D term)

Substituting into the plant:

    phi_ddot = -c * kp_r * (rate_setpoint - gyro_x)

**Case A — suppose `gyro_x = +phi_dot`** (gyro in encoder convention):

    phi_ddot = -c*kp_r*rate_setpoint + c*kp_r*phi_dot

The coefficient on `phi_dot` is **positive**: any rate perturbation grows exponentially.
Closing the outer loop (`rate_setpoint = kp_a*(phi - sp)`) gives

    phi_ddot - c*kp_r*phi_dot + c*kp_r*kp_a*phi = const

— a second-order system with **negative damping**. Unstable oscillator. This configuration
cannot hold the lever.

**Case B — suppose `gyro_x = -phi_dot`** (gyro inverted relative to encoder convention):

    phi_ddot = -c*kp_r*rate_setpoint - c*kp_r*phi_dot

Positive damping. With the outer loop closed:

    phi_ddot + c*kp_r*phi_dot + c*kp_r*kp_a*phi = c*kp_r*kp_a*sp

— a standard stable second-order system converging to `phi = sp`. This is the textbook
cascaded-PID closed loop, just written in a mirror.

Since the bench demonstrably holds (HoldMAE ~2 deg), **reality is Case B**: in this codebase,
`gyro_x` is the *negative* of the encoder-convention angular rate. The code produces this at
`flight.py:155` via `gyro_x = -imu_sign * degrees(gx)`.

### 0.4 Empirical confirmation

From run `2026-06-07_13-48-31`:

| Check | Result | Meaning |
|---|---|---|
| corr(IMU_roll_from_quat, ENC_ROLL) | **+0.9955** | `imu_roll` is in encoder convention: `imu_roll ~ phi` |
| corr(GYRO_X, d(ENC_ROLL)/dt) | **-0.61** | `gyro_x ~ -phi_dot` (Case B confirmed) |
| Row 1 spot check | ENC falling at ~-82 deg/s, GYRO_X = +56 | same conclusion, raw data |

(The -0.61 rather than -0.99 is an artifact of differentiating decimated 76 ms encoder
samples against an instantaneous gyro reading; the *sign* is unambiguous, and the spot check
confirms the magnitude relationship too.)

### 0.5 The hidden contract

So the cascade runs entirely in an **inverted convention** relative to textbook PID:

| Quantity | This codebase | Textbook |
|---|---|---|
| angle error | `ang_err = measurement - setpoint` | `setpoint - measurement` |
| rate measurement | `gyro_x = -phi_dot` | `+phi_dot` |
| `rate_setpoint` semantics | positive = "phi should decrease" | positive = "phi should increase" |

Two inversions cancel; the loop is net negative feedback; everything works. **But this is a
landmine, not a style choice**, because:

1. Every intermediate signal in telemetry (`ANG_ERR`, `RATE_SP`, `RATE_ERR`) has the
   opposite sign of what anyone (including future-you) will assume when reading a CSV.
2. Any new term added to the loop must independently honor the inversion — and exactly one
   such term already got it wrong (F1, the feedforward).
3. The in-code comment at `flight.py:150-154` that justifies the negation is itself confused:
   it argues from GRV-vs-gyro reporting conventions, but GRV and gyro come from the *same
   physical sensor in the same body frame* — if `imu_roll = phi` then `degrees(gx) = +phi_dot`
   by construction. The negation is required by **loop stability given the mixer sign**
   (Section 0.3), not by anything about the sensor. A comment that justifies the right code
   with the wrong reason will eventually license the wrong code.
4. Porting these gains or this structure to any standard flight controller convention
   (error = setpoint - gyro) will flip behavior.

This is F9 in the backlog: either rewrite the loop in textbook convention (one deliberate,
well-tested session: flip `ang_err`, flip `gyro_x`, flip mixer application — net behavior
identical), or keep the inverted convention but document it as a first-class contract in
flight.py and fix the misleading comment.

---

## Part 1 — Confirmed bugs

### F1. Feedforward lead term is sign-inverted — it adds GRV lag instead of canceling it

**Severity: HIGH (control correctness) | Location: `flight.py:169` | Effort: 1-line fix + A/B session**

```python
feedforward_roll = (imu_roll + gyro_x * feedforward_lead_s) - setpoint_roll_deg
```

**Intent (DR-006):** GRV is a fused, filtered estimate; it lags physical truth by some
latency tau. To act on the *current* angle, predict forward: `phi(t) ~ grv(t) + phi_dot * tau`.

**What the code computes:** Per Part 0, `gyro_x = -phi_dot`. Therefore:

    imu_roll + gyro_x * lead  =  phi - phi_dot * lead

That is the angle `lead` seconds **in the past** — a retrodiction. Instead of canceling the
GRV filter lag, the term *doubles down on it*: the outer loop acts on an angle estimate that
is (GRV internal lag + 12 ms) stale.

**Empirical evidence** (vs encoder, which is near-zero-latency ground truth):

| Angle estimate | RMS error vs ENC_ROLL |
|---|---|
| `imu_roll` raw | 0.824 deg |
| `imu_roll + gyro_x*lead` (current code) | 0.838 deg (worse than doing nothing) |
| `imu_roll - gyro_x*lead` (sign flipped) | 0.818 deg (better than doing nothing) |

The deltas are small at lead = 12 ms, which is exactly why the system tolerates the bug.

**Why nobody noticed — and the corroborating clue:** the magnitude of the error injected into
the loop is `kp_a * lead * phi_dot` through the P path = `3.5 * 0.012 = 4.2%` of the rate
damping, with the wrong sign (it *erodes* damping instead of adding lead). Small enough to be
absorbed by tuning. But note the tuning history: **reducing lead_ms 15 -> 12 improved hold
(2026-05-31)**. If the term were doing its job, increasing lead toward the true GRV latency
should help; instead *less* helped. "Less of a harmful term is better" is exactly the
signature of an inverted term. The historical record was telling you this; it just needed
the sign analysis to decode.

**Secondary effect on the D term:** the angle PID differentiates `ang_err`, which contains
`gyro_x * lead`. Its derivative contributes `-kd_a * lead * phi_ddot` — a small spurious
angular-*acceleration* feedback term (~0.0078 * phi_ddot) that wouldn't exist if the
feedforward were correct or absent. Another reason ANG_D telemetry is hard to interpret.

**Recommended fix and verification:**
1. Change to `imu_roll - gyro_x * feedforward_lead_s` (correct given the current `gyro_x`
   convention), or fold it into the F9 convention cleanup so the formula reads naturally.
2. Run a 3-arm A/B/C: flipped sign @ 12 ms, lead = 0, and current code @ 12 ms (control).
   Compare T_s, overshoot, HoldMAE over >= 3 runs each (wire-harness variability means single
   runs are not conclusive — see the 2026-05-18 session lesson).
3. With the correct sign, it may be worth re-sweeping lead upward (15-25 ms): if GRV lag is
   real, the optimum should now sit near the true latency rather than near zero.
4. Caveat to keep honest: at 76 ms telemetry resolution the cross-correlation could not
   resolve the actual GRV lag (best fit was lag 0). It is possible GRV lag at 100 Hz is
   small enough that the right answer is "no feedforward at all." The A/B/C decides; do not
   assume.

---

### F2. `outer_dt` is wrong by 10% — integer-division artifact from the 300 Hz migration

**Severity: MEDIUM-HIGH (gain calibration) | Location: `flight.py:113-115` | Effort: 1 line + retune awareness**

```python
inner_ms    = MS_PER_S // rate_loop_hz          # 1000 // 300 = 3   (true value: 3.333)
outer_ticks = rate_loop_hz // angle_loop_hz     # 300 // 100  = 3   (exact)
outer_dt    = (inner_ms * outer_ticks) / MS_PER_S   # 0.009 s — but the real period is 0.010 s
```

The outer loop fires every 3rd gyro delivery; gyro deliveries come at 1/300 s; the true outer
period is 3/300 = 10 ms. The code computes 9 ms because `1000 // 300` floors to 3 ms.

**Consequences:** the angle PID receives `dt = 0.009` while reality is 0.010:

- I term: `integral += error * dt` accumulates 10% slow -> effective `ki` = 0.090 when config
  says 0.10.
- D term: `(error - prev) / dt` divides by a too-small dt -> effective `kd` = 0.722 when
  config says 0.65 (11% hot).

**Why it matters even though the loop is tuned:** every gain you have recorded since the
300 Hz migration (the entire kd sweep of 2026-05-31, the current baseline) is calibrated
against the *wrong dt*. The loop behaves fine — the error is static and tuning absorbed it —
but: (a) your documented gains do not mean what they say, which corrupts cross-session
reasoning and any future model-based work; (b) the bug is rate-dependent: at 200/50 Hz it
divided exactly (5 ms * 4 = 20 ms, correct), so changing loop rates again will silently
re-scale your angle gains by a different factor. That is precisely the kind of "backfires
when you least expect it" coupling you asked about.

**Fix:** compute the outer period directly and in float:

    outer_dt = outer_ticks / rate_loop_hz

**Retune note:** fixing this multiplies effective ki by 1.111 and divides effective kd by
1.111 relative to today's behavior. To preserve current closed-loop behavior exactly while
making the books honest, adjust config at the same time: `ki 0.10 -> 0.09`, `kd 0.65 -> 0.722`
(or just accept the small shift and re-confirm the baseline; either way, do it consciously
and note it in the tuning log).

**Related observation (no action needed yet):** the same `inner_ms` truncation means any
duration math built on `inner_ms` is approximate, but currently `inner_ms` is only used for
`outer_dt`, so the single-line fix covers it.

---

### F3. Shutdown sequence: motors hot through SD finalize; disarm can be skipped entirely

**Severity: CRITICAL (safety) | Location: `flight.py:258-271`, `recorder.py:141-156` | Effort: small, but test the failure paths**

```python
        stabilize(...)
        set_led(b=1)                      # (a) blue = "idle" — but motors still spinning

    except Exception as e:
        set_led(r=1)
        if telemetry is not None:
            telemetry.write_crash_log(e)  # (b) SD write while motors hold last throttle
        raise

    finally:
        if telemetry is not None:
            telemetry.end_session()       # (c) seconds of SD I/O; can itself raise
        if motors is not None:
            motors.disarm()               # (d) never reached if (c) raised
```

Three distinct problems, worst last:

**(1) Every clean run ends with seconds of uncontrolled thrust.** When `stabilize()` returns,
the throttles are frozen at the last commanded values (~600 base +/- last output) and Core 1
keeps refreshing them at 1 kHz. `end_session()` then copies `log.tmp` -> `log.bin` in
512-byte read+write chunks (~124 KB for a 120 s run at sample_every=20) and unmounts —
seconds of wall time during which the lever is held by open-loop thrust at whatever the last
differential was. Control has stopped; thrust has not.

**(2) The LED lies during that window.** `set_led(b=1)` runs *before* the finally block.
Blue means "idle / ready to arm" per the project's own convention. An operator who reaches
for the bench at blue — to catch the lever, re-seat a wire, swap the battery — is reaching
into spinning props at flight throttle. This is the single most operator-dangerous behavior
in the codebase precisely because it occurs on *every successful run*, training you to trust
it.

**(3) A failing SD card cancels the disarm.** Trace the most likely real failure: the SD
card glitches mid-run -> `SdSink.write_bytes` raises OSError inside `stabilize()` -> except
branch -> `write_crash_log` (tries the same dead card, swallows its own errors — fine) ->
`raise` -> finally -> `end_session()` -> `self._f.flush()` on the dead card **raises a new
OSError** -> the finally block aborts -> **`motors.disarm()` never executes**. The new
exception also *replaces* the original one, so the crash report points at the close path
instead of the root cause. Result: red LED, stack trace on console, and motors holding
last throttle indefinitely (Core 1 is a dumb 1 kHz repeater; it will do this until power
cut). The exact component whose failure triggered the crash is the one trusted to run
cleanly during cleanup.

Note `recorder.py`'s `end_session()` docstring even says "Call when leaving STABILIZING
state (before disarm)" — the wrong priority is documented as the contract. Telemetry
durability must never outrank motor shutdown.

**Recommended structure:**

1. First line of `finally`: `motors.emergencyStop()` (lock-free, atomic, picked up by
   Core 1 within 1 ms — it exists for exactly this).
2. Then telemetry close wrapped in its own try/except so SD failures cannot abort cleanup.
3. Then full `motors.disarm()` / `motors.stop()`.
4. Move `set_led(b=1)` to *after* motors are confirmed stopped; consider a distinct LED
   state (e.g. green blink or white) for "finalizing log, props may still be spinning" if
   you want the SD copy to happen before spin-down for any reason.
5. While there: `write_crash_log` runs before any motor stop too. Either stop motors at the
   top of the except branch, or rely on the reordered finally and accept the (small) crash-log
   delay.

**Test plan:** simulate the SD death (pull card mid-run, or monkeypatch `write_bytes` to
raise after N records) and verify motors stop. This failure path has demonstrably never been
exercised.

---

### F4. `stabilize()` has no abort path — the docstring promises one that does not exist

**Severity: CRITICAL (safety) | Location: `flight.py:103-110` (docstring) vs loop body `134-198` | Effort: small**

The docstring says: *"Run cascaded PID control loop until B+Y disarm combo or duration_ms
elapses."* There is no button check anywhere in the loop. `ui.buttons_by_held()` exists,
is tested (it gates arming via `wait_for_go()` in main.py), and is never called in flight.

Consequences, in increasing order of badness:

1. **No way to stop a bad run.** A diverging tune, a thrown prop, a smoking ESC — your only
   options are "wait out duration_s" or cut power to a system actively driving motors.
2. **`duration_s: null` is an unstoppable machine.** The docstring explicitly blesses
   `duration_ms=None` as "no time constraint — manual stop only," but the manual stop was
   never implemented. Config currently says 120, so this is latent — until the day you run
   an open-ended soak test.
3. Combined with F5 (no staleness watchdog), even the duration check can become unreachable
   in one specific case — see F5.

**Fix:** poll `buttons_by_held()` once per outer tick (100 Hz polling is plenty for a human
combo; polling at inner rate is also fine — two GPIO reads, sub-microsecond). On trigger:
break, with motors stopped by the (fixed, F3) shutdown path. The pin reads are passive and
cannot disturb the loop.

**Also fix the docstring** in the same commit — a safety doc that overpromises is worse
than no doc.

---

### F5. No sensor-staleness watchdog: a dead IMU freezes the throttles forever

**Severity: CRITICAL (safety) | Location: `flight.py:139-141` | Effort: small**

```python
if imu.update_sensors() == 0:
    utime.sleep_us(500)
    continue
```

If the BNO085 stops delivering packets — I2C bus wedge (clock-stretch deadlock is a known
BNO08x behavior), brownout reset (SHTP devices lose all feature configs on reset and go
silent until re-enabled), INT line disturbed by the wire harness — then `update_sensors()`
returns 0 forever and the loop spins in this `continue` branch **with the motors holding the
last commanded differential at full base throttle**, refreshed dutifully at 1 kHz by Core 1.

Two sub-cases:

- **Silent sensor (returns 0):** the duration check at the top of the loop *is* still
  reached each spin, so with `duration_s = 120` the run eventually ends — after up to two
  minutes of open-loop thrust. With `duration_s: null` (F4) it never ends.
- **Wedged I2C (call blocks or raises):** an OSError propagates to `run()` and lands in the
  F3 shutdown path — which, pre-fix, may skip disarm. The two findings compound.

A flight controller treats "gyro data stopped" as a within-milliseconds disarm condition.
The bench equivalent costs a few lines: track `last_packet_ms`; if
`ticks_diff(now, last_packet_ms) > N` (e.g. 100 ms = 30 missed gyro periods — far beyond
any GC pause or SD stall you have measured), kill motors and raise a descriptive error.
The crash log then tells you the IMU died, instead of you learning it from the smell of
hot ESCs.

**Related hardening in the same area (cheap, do together):**

- **Envelope protection:** you already read the trusted ground-truth encoder *every inner
  cycle* (`flight.py:182`) — purely for telemetry. One comparison turns it into a crash
  barrier: `if abs(enc_angle) > LIMIT: emergencyStop()`. The restrictor bounds the physical
  travel anyway (start is ~51 deg), so `LIMIT ~ 70 deg` can only trip on genuine loss of
  control or encoder failure, both of which warrant a stop. This is the cheapest
  highest-value safety feature available to this codebase.
- **`motors.isHealthy()` is never called after arming.** Core 1 is a `while self._running`
  loop with no exception handler; if it dies, the PIO FIFOs starve and the ESCs see signal
  loss (behavior then depends on ESC firmware — typically motor stop after a timeout, but
  you have not verified yours). A heartbeat-delta check once a second from the control loop
  (cache the counter, compare next second — do *not* call `isHealthy()` itself in the loop,
  it sleeps 5 ms) makes the failure explicit. Low probability, near-zero cost.
- **BNO accuracy status is available and discarded.** The driver delivers per-report
  accuracy (0-3) in `val[4]` / the `.full` tuple. The base-650 resonance incident
  (2026-05-31) is the proof case: GRV fusion degraded mid-run (sensor_health correlation
  0.68-0.73) and nothing in the loop noticed. Logging accuracy into telemetry (spare uint16
  exists in the record? if not, 1 byte is cheap at the next format rev) and optionally
  flagging accuracy == 0 ("Unreliable") gives you the in-run signal that today you only
  reconstruct forensically.

---

## Part 2 — Control-theory weaknesses (working today, fragile tomorrow)

### F6. Anti-windup: state clamping only, with misleading semantics, and no saturation awareness

**Severity: MEDIUM | Location: `pid.py:26-30`, interaction with `output_limit` and the mixer | Effort: medium (touches tuning)**

Current implementation:

```python
self._integral += error * dt
# clamp self._integral to +/- iterm_limit
...
self.last_i = self.ki * self._integral
```

**Issue (a) — semantics.** `iterm_limit` clamps the integral *state* (units: error-seconds),
not the I *contribution* (units: output). The actual ceiling on the I term is
`ki * iterm_limit` — for the angle loop, `0.10 * 5 = 0.5 deg/s`. Two consequences:

- Your tuning logs discuss `iterm_limit` as if it capped the term. It caps the term only
  at the current ki. **Change ki and the windup ceiling you tuned silently rescales.**
  Example: trying `ki = 0.2` in a future session doubles not just integral authority but
  also the windup ceiling, conflating two effects in one knob.
- The conventional formulation clamps `ki * integral` (the contribution) so the limit is
  in output units and independent of gain. Worth converting; it is a 2-line change plus a
  config-value translation (`iterm_limit_new = ki * iterm_limit_old`).

**Issue (b) — no conditional integration.** The integrator charges whenever error exists,
including while the output is pinned at `output_limit`. During the startup approach the
angle error is ~51 deg and the output is saturated at 130 deg/s for over a second; the
integrator rails at its clamp regardless of the fact that more I cannot do anything. This
is the *exact* mechanism behind the 2026-05-14 bimodal settling (residual I at setpoint
crossing -> overshoot -> slow ring-down). The fix applied then — crushing `iterm_limit`
100 -> 5 — works, but it is symptom suppression: it limits integral authority *everywhere*
(including in steady-state hold, where I is the term fighting the wire-harness torque) in
order to limit it *during saturation*.

The standard remedies, in increasing sophistication:

1. **Conditional integration (integrator clamping on saturation):** skip
   `integral += error*dt` when the output is saturated *and* the error has the same sign
   as the saturation direction. Cheap, robust, no new tuning parameter. The integrator
   then charges only when it can actually influence the plant.
2. **Back-calculation:** after clamping the output, bleed the integral toward the value
   that would have produced the clamped output: `integral += kaw * (out_clamped - out_raw) * dt`.
   One new gain (`kaw`, typically ~1/ki time-constant), smoother than (1).

Either would likely let you raise `iterm_limit` (and possibly ki) for stiffer steady-state
hold against the wire-harness torque — the thing the abandoned +/-9 deg campaign needed —
without resurrecting the overshoot mode. Note `compute()` would need to know saturation;
since `output_limit` already lives inside `PID`, option (1) is fully self-contained.

**Issue (c) — minor:** the rate PID integrates with `ki = 0.0`: `_integral` accumulates and
rails at +/-50 permanently, computing `0.0 * 50` forever. Harmless dead state today; becomes
a surprise only in exotic cases (e.g. introspecting `_integral`). Guard with `if self.ki:`
or just know it is there.

### F7. Mixer saturation is invisible to the PIDs — and only config luck prevents it today

**Severity: MEDIUM | Location: `mixer.py:15-19`, config `motor` + `rate.pid.output_limit` | Effort: small (assert) to medium (desaturation)**

```python
m1 = self.throttle_base - int(pid_output)
m2 = self.throttle_base + int(pid_output)
m1 = max(self.throttle_min, min(self.throttle_max, m1))
m2 = max(self.throttle_min, min(self.throttle_max, m2))
```

With today's numbers — base 600, rate `output_limit` 300, clamps [90, 900] — the full PID
output range maps to exactly [300, 900]: **inside the clamps with zero margin, by numeric
coincidence**. Nothing enforces this relationship.

What happens when it breaks (e.g. `base_throttle = 650`, which you have already tried for
other reasons): a positive output of 300 wants `m2 = 950`, clamps to 900, while `m1 = 350`
is untouched. Three simultaneous distortions, none visible to the controller:

1. **Differential delivered != 2*output.** The plant gain from `output` to torque drops —
   only on one side, only beyond the clip point. The rate loop's effective kp becomes
   asymmetric and amplitude-dependent.
2. **Collective shifts.** `(m1+m2)/2` is no longer `base`; clipping one side changes total
   thrust, which on the bench changes wire-tension/friction loading and on a vehicle changes
   altitude. The loops have no channel to know this happened.
3. **The PID's anti-windup reasoning is invalidated.** `output_limit` exists to bound
   authority; after the mixer clips, the *real* authority bound is tighter than the PID
   believes, so I continues charging against authority that does not exist (compounds F6b).

The general problem is mixer desaturation — on multirotors solved by shifting the collective
to preserve the differential (give up altitude to keep attitude authority). For the bench you
do not need the full machinery; you need to **make the invariant explicit**:

- Minimum (do this now): startup assertion in `run()` or `LeverMixer.__init__`:
  `base + output_limit <= throttle_max` and `base - output_limit >= throttle_min`, with a
  clear error message. Turns a silent plant-gain change into a config error at arm time.
- Better (when you next touch the mixer): on clip, shift both motors to preserve the
  differential (`if m2 > max: m1 -= (m2 - max); m2 = max`, then re-clamp m1 against min) —
  i.e. sacrifice collective for differential, which is the right priority for an
  attitude-control bench. Log a saturation flag into telemetry so clipped cycles are
  identifiable in analysis.

**Footgun in the same file:** the mixer returns `(m1, m2, m1, m2)` for four motors with two
installed. The day M3/M4 are mounted, plant torque gain doubles instantly under gains tuned
for half the authority — expect a violently hot loop on first arm. Make motor count (or a
per-pair enable) explicit in config *before* that hardware day arrives, and treat first
4-motor arm as a fresh tuning session with reduced gains.

### F8. dt is measured in integer milliseconds — +/-15-20% multiplicative noise on the rate loop's I and D scaling

**Severity: MEDIUM | Location: `flight.py:143-147`, consumed in `pid.py` | Effort: small**

At 300 Hz the true inner period is 3.333 ms; `ticks_ms()` arithmetic yields `dt_ms` in
{3, 4}. Every cycle, the rate PID's:

- I accumulation uses dt that is 10% low or 20% high;
- **D term divides a measurement delta by that quantized dt** — a hard +/-15-20%
  multiplicative noise on `RATE_D` every single cycle, uncorrelated with the signal.

The D noise is the part that matters: derivative terms are noise amplifiers by nature, and
this adds artificial jitter *on top of* gyro quantization (gyro LSB ~0.11 deg/s at Q9). At
kd = 0.009 the absolute throttle impact is small today, which is why it has not bitten; but
it puts a floor under any future kd increase and pollutes RATE_D telemetry you use for
tuning decisions.

Edge case in the same code: when two packets land in the same millisecond, `dt_ms = 0`;
`pid.py:36` then forces derivative = 0 **but still advances `_prev_measurement`**, so that
sample's slope is not deferred — it is silently dropped (the next derivative spans only the
newest interval). Rare, minor, but it is data loss in the D path.

**Fix:** measure with `ticks_us()` / `ticks_diff` and convert to float seconds
(`dt = dt_us / 1_000_000`). Telemetry can keep reporting integer ms (or switch DT to
tenths-of-ms in a uint16 at the next format rev for better loop-jitter forensics). This
makes both the I scaling and the D division honest at zero runtime cost.

### F9. The inverted sign convention as a standing liability

**Severity: MEDIUM (latent) | Location: pervasive (`flight.py:155, 164-172`) | Effort: medium, one focused session**

Fully derived in Part 0. Summary of the decision to make:

- **Option A — convert to textbook convention.** `ang_err = setpoint - measurement`,
  `gyro_x = +phi_dot` (i.e. `+imu_sign * degrees(gx)`), `rate_error = rate_setpoint - gyro_x`
  unchanged, and flip the mixer application (`m1 = base + output, m2 = base - output`).
  Net closed-loop behavior is mathematically identical; every telemetry sign becomes
  standard; the feedforward reads naturally as `imu_roll + gyro_x*lead` *and is then
  correct as written*. Cost: one careful session, every analysis script that assumes
  current signs must be checked (gate.py Pearson sign, plot conventions), and one
  confirmation flight against the baseline.
- **Option B — keep it, but make it a contract.** A single authoritative comment block in
  flight.py stating: "This loop runs in inverted convention: ang_err = meas - sp,
  gyro_x = -phi_dot, positive rate_setpoint = M1 must descend. Both inversions are
  required jointly; never change one without the other." Plus fixing the incorrect
  rationale in the current `flight.py:150-154` comment (it should cite mixer-sign /
  stability, not GRV-vs-gyro reporting).

A is the right long-term answer (M5 multi-axis will multiply the number of places the
convention must be honored by 3); B is acceptable if A has to wait. Doing F1's sign fix
*inside* Option A is the cleanest sequencing.

### F10. No filtering anywhere in the D paths or on the gyro

**Severity: MEDIUM (latent — will bite with 4 motors) | Location: `pid.py` D terms, `flight.py` gyro path | Effort: small-medium**

Current state: the rate D differentiates raw quantized gyro against quantized dt (F8); the
angle D differentiates `ang_err`, which embeds the gyro lead term, so it carries angular-
acceleration noise scaled by `kd_a * lead`. There is no low-pass anywhere between sensor
and motor command.

Today this is survivable because: the frame is rigid and small, kd_r is tiny (0.009), and
the BNO's internal fusion already filters GRV. But you have *already had* the warning shot:
at base 650, frame resonance corrupted the accelerometer enough to destabilize GRV fusion
(2026-05-31). That coupling path — vibration -> sensor -> D term -> motor command ->
vibration — is the classic D-term oscillation mechanism, and adding two more motors will
move the excitation profile in untested directions.

Standard practice on flight controllers: first-order LPF on the D term (cutoff well above
the control bandwidth, well below motor/frame noise — for your ~0.04 Hz-to-few-Hz dynamics,
a 10-20 Hz D-term cutoff would be conservative and nearly phase-free), optionally an LPF on
the gyro feeding P as well. Implementation is a one-liner in `PID.compute`:
`d_filt += alpha * (d_raw - d_filt)` with `alpha = dt / (dt + 1/(2*pi*fc))`.

Sequencing note: do F8 (microsecond dt) *first* — much of the current D noise is
self-inflicted dt quantization, and you want to see the true noise floor before choosing a
filter cutoff.

### F11. Loop scheduling gates on "any SHTP packet," not on the specific report being consumed

**Severity: LOW-MEDIUM | Location: `flight.py:139` (inner), `158-163` (outer) | Effort: small**

`update_sensors() > 0` counts *any* processed packet — a control-channel response, a
GRV-only bundle, anything. Two consequences:

- **Inner loop on stale gyro:** a packet that contained no gyro report still runs the rate
  loop. The D term sees an unchanged measurement (derivative = 0 — benign), but `rate_error`
  is recomputed against a stale rate and acted upon with fresh dt. Mostly harmless jitter at
  current rates, but it is acting-on-no-information by construction.
- **Outer loop by counting, not freshness:** `outer_counter >= outer_ticks` fires whether or
  not a new GRV arrived. If a GRV report is dropped or delayed (bus contention, packet
  bundling under load), the outer tick computes from the *previous* quaternion with the
  *fixed nominal* `outer_dt`: the D term sees zero change this tick and a doubled change next
  tick — a manufactured D spike (then doubled by F2's hot kd, filtered by nothing per F10).

The driver already maintains exactly the right primitive: per-feature unread counters
exposed as `imu.gyro.updated` and `imu.game_quaternion.updated`. Gating the inner cycle on
`gyro.updated` and the outer computation on `game_quaternion.updated` (fall back to skipping
the angle PID update and holding `rate_setpoint` if GRV is stale — ZOH is what already
happens between ticks anyway) makes the "data-driven loop" design actually data-driven at
the per-sensor level. Note: reading a feature clears its unread counter, so check `.updated`
*before* the reads, and be aware the driver overwrites bundled same-type reports (only the
newest survives a multi-report packet) — your dt-from-host-time handles the integration
correctly across such drops, which is a point in the current design's favor.

### F12. Quaternion-to-roll extraction is single-axis-only, with no wrap or double-cover handling

**Severity: LOW (bench) / HIGH (the moment M5 starts) | Location: `flight.py:164` | Effort: small**

```python
imu_roll = imu_sign * degrees(2.0 * atan2(iqi, iqr))
```

This equals the roll angle only when the quaternion is a pure x-rotation (qj = qk = 0).
The general roll extraction is:

    roll = atan2( 2*(qr*qi + qj*qk), 1 - 2*(qi^2 + qj^2) )

On the bench the difference is a small bias/scale error proportional to qj/qk contamination
— tare residual, mount misalignment, frame flex (run data shows |qj| up to ~0.12, which is
not negligible). Some of the "irreducible ~1.4 deg IMU-ENC floor" you have characterized may
in fact live here, in the formula rather than the sensor — cheap to test offline: recompute
roll from the logged quaternions with the full formula and compare IMU-ENC bias across
existing runs. That is a zero-hardware experiment that directly feeds the IDEA-003 question
(mechanical vs sensor cause).

Also absent, fine on the bench, fatal on a vehicle: quaternion double-cover handling (q and
-q encode the same attitude; a sign flip between reports would step `2*atan2` by 2*pi) and
angle wrapping at +/-180 deg in the error computation. File under "before this logic ever
controls something that can rotate freely."

---

## Part 3 — Smaller items and notes

### F13. `ui.set_led` comment contradicts the code

`ui.py:10` says "active HIGH, common cathode — value 1 = LED on" but the implementation
writes `led.value(not r)` — i.e. the hardware is active LOW (Display Pack LED is common
anode). Behavior is correct; the comment will mislead the next pin-level debugging session.
One-line doc fix.

### F14. `_read_packet(wait=True)` latency tail

`i2c.py:126-129`: if the interrupt flag was set but INT has deasserted by read time, the
driver spins in `_wait_for_int` up to 50 ms. In practice the next 300 Hz report bounds the
wait to ~3 ms, so this is a theoretical tail, not a problem — noted so future "mystery 50 ms
dt spike" forensics have somewhere to look. Your existing MAX_DT_MS telemetry would catch it.

### F15. Arm/prespin period runs blind

`arm_motors` + `prespin_motors` is ~11 s (5 s settle + ~5.2 s ramp + 1 s dwell) during which
nobody calls `update_sensors()` (BNO INT stays asserted, reports queue/overwrite in the
sensor — recovered by the seed read, fine) and **no abort is possible** (F4 applies here
too: a prop strike during ramp-up has no kill switch). When F4's button check lands, give
the prespin ramp the same check between steps — it is already a 100 ms-granularity loop, the
natural place for it.

### F16. Telemetry record has no saturation/health flags

You log every PID term (excellent) but not: output-clamped flag, mixer-clipped flag, GRV
accuracy, gyro accuracy. All are 1-bit/2-bit signals that turn post-flight "why did it
wobble at t=43s" sessions from inference into lookup. Next time `_RECORD_FMT` changes
anyway, pack a flags uint16. (Do not change the format just for this.)

---

## Part 4 — Quality assurance: safety nets against sign and orientation errors

This part exists because of a confession and a pattern. The confession: the current sign
mess began when the encoder was physically flipped during the mechanical overhaul, the
symptom was misattributed to an inverted IMU, and the system was then re-tuned into working
around the misdiagnosis. The pattern: every sign bug found in this review (F1, the
historical inverted-tracking incident, the encoder flip) shares the same life cycle —
**introduced during a hardware or convention change, invisible because something else
compensates, discovered only by accident or by a die-hard reviewer with a correlation
script.** The goal of this part is to make that life cycle impossible: every contract that
today lives in someone's head gets a check that runs automatically and points at the exact
contract violated.

### 4.1 Why this bug class evades you specifically (the taxonomy)

Understand the enemy before building the defenses. Sign/orientation errors survive for
three structural reasons:

**Reason 1 — Double inversions cancel.** A single flipped sign in a feedback loop is
usually loud (the loop diverges on first arm, see Part 0 Case A). But *two* flipped signs
are silent: the loop is stable, KPIs look fine, and the only residue is that every
intermediate signal means the opposite of what you think. The system *selects for* paired
errors: when you flip the encoder and the loop diverges, your natural move is to flip
something else until it converges — and if you flip the *wrong* something (IMU instead of
encoder), you have now manufactured a stable double inversion and buried the original
mistake under a working system. That is precisely the 2026-04 sequence. The defense is
never "look harder"; it is checks that test each sign **individually against physical
truth**, so a compensating pair cannot pass.

**Reason 2 — Agreement is not correctness.** `corr(IMU_roll, ENC_ROLL) = +0.99` — the
check the analysis gate already performs — proves the two sensors *agree with each other*.
It cannot distinguish (encoder correct, IMU correct) from (encoder flipped, IMU flipped).
Pairwise consistency checks have a global sign ambiguity; you need at least one **absolute
anchor to physical reality** to break the symmetry. There are exactly three anchors
available on this bench, all cheap:

- *Gravity*: the IMU's gravity/accel vector knows which way is down, independent of any
  convention you chose. With the lever held at a known nonzero angle, the sign of the
  gravity component along the relevant axis is ground truth.
- *Known manual motion*: "I am pressing the M1 end down right now" is ground truth. Any
  sensor that disagrees with your hand is wrong, full stop.
- *Known actuation*: "M2 was just commanded 30 counts above M1" has exactly one legal
  encoder response direction (M1 end rises, phi decreases). The motor-to-encoder path is
  ground truth for the mixer + motor wiring + encoder chain jointly.

Anchored checks localize the fault: if motion says encoder is right and encoder disagrees
with IMU, *the IMU path is wrong* — no more misattribution.

**Reason 3 — The rate path had no check at all.** The position contract (IMU vs encoder)
was gated; the rate contract (`gyro_x` vs `d(enc)/dt`) was not — and the wreck hid exactly
in the ungated path. The general lesson: **every signal that enters a PID needs its own
contract check**, because the loop will happily stabilize around a wrong-signed signal if a
second wrong sign compensates (Reason 1). For this bench the full contract set is small and
enumerable; it is written out in 4.2.

### 4.2 Layer 1 — Telemetry invariants gate (offline, free, every run)

Promote `tmp/check_gyro_sign.py` into a permanent pipeline stage:
`pipelines/flight-analyser/scripts/invariants.py`, run by `run.py` right after (or inside)
`gate.py`, before any KPI scoring. It costs nothing per run and converts every flight you
were going to do anyway into a regression test of the entire sign architecture.

The invariants, each encoding one contract, each with a "what it catches" — thresholds are
deliberately loose because these are *sign and sanity* checks, not performance checks
(performance is scoring's job):

| # | Invariant | Expect | Catches |
|---|---|---|---|
| I1 | corr(IMU_roll, ENC_ROLL) | > +0.95 | IMU/encoder orientation disagreement (existing gate check — keep) |
| I2 | corr(GYRO_X, d(ENC_ROLL)/dt) | < -0.4 | rate-convention violation — **the check that would have caught F1 and the encoder flip** |
| I3 | corr(d(IMU_roll)/dt, d(ENC_ROLL)/dt) | > +0.5 | quaternion path vs encoder rate agreement (independent of gyro) |
| I4 | sign check: ANG_ERR vs (ENC_ROLL - setpoint) | corr > +0.9 | outer-loop error convention drift, setpoint sign regressions (the 2026-05-02 bug class) |
| I5 | corr((M2-M1), d2(ENC_ROLL)/dt2) over transient | < 0 | actuation direction: mixer sign, motor wiring swap, prop reversal |
| I6 | median(DT_MS) approx 1000/rate_hz; max within budget | within 20% / < 25 ms | loop-rate regressions, scheduling bugs (F2-class arithmetic shows up here) |
| I7 | fraction of samples with PID_OUT at output_limit; M1/M2 at clamp | report, warn > 30% hold-phase | silent saturation (F7), authority loss |
| I8 | RMS(IMU_roll - ENC_ROLL) during hold | < threshold (start ~1.5 deg) | tare drift, mount shift, the IMU-ENC bias variable IDEA-003 tracks — now measured every run automatically |
| I9 | sign(ENC_ROLL[0]) and magnitude vs bench.start_angle_deg | matches | encoder flip / axis_center drift at run start (existing gate has a form of this — keep, tighten) |

Implementation notes:

- I2 is the crown jewel. Note its expected sign is *negative* under the current convention
  (Part 0); if/when F9 Option A lands, the expe cted sign flips to positive — which is itself
  a perfect illustration of why the invariants file must read the expected contract from one
  authoritative place (see 4.6) rather than hardcoding it twice.
- I5 needs care: correlate only over the transient (first reach), use a smoothed second
  difference, and treat it as a sign check, not a model fit. Even corr = -0.1 with p-value
  fine is a pass; the failure mode you are catching is corr emphatically positive.
- Output: an `invariants.json` in the run's `analysis/` folder plus a hard PASS/FAIL line in
  the report. **A failed invariant should block KPI scoring** — a run with a violated sign
  contract has meaningless KPIs, and scoring it anyway is how a misdiagnosis gets re-tuned
  into the gains (Reason 1).
- Each failure message should name the contract in physical terms, e.g.: "I2 FAIL:
  GYRO_X correlates +0.61 with encoder rate; contract requires negative (gyro_x = -phi_dot).
  Likely causes: imu_invert wrong, encoder_invert wrong, gyro sign change in flight.py.
  Run the signcheck tool to localize." The error message that tells you the next diagnostic
  step is the one that prevents the misattribution.

### 4.3 Layer 2 — The signcheck tool (on-bench, after any mechanical change)

`coord_check.py` was retired when sensor orientation moved into config booleans. The
encoder-flip incident is the argument for its successor: the config booleans made the
*representation* clean but removed the *measurement procedure* that sets them. Bring back a
purpose-built interactive tool — `tools/signcheck.py`, run via mpremote like `tare.py` —
implementing the anchored checks from 4.1:

**Phase A — static gravity anchor (no hands).** With the lever resting on the restrictor
(known: M1 end down, enc approx +51 deg): read encoder sign, read IMU roll sign, read the
gravity vector. All three must be consistent with "M1 down". This catches a flipped
encoder *and* a flipped IMU independently, because gravity does not care about your config.

**Phase B — manual motion anchor.** Prompt: "slowly lift the M1 end to horizontal, then
lower it back. Press enter when done." While moving, sample enc, IMU roll, and gyro at
~50 Hz. Verdicts:
- d(enc)/dt and d(imu_roll)/dt same sign throughout -> position paths agree;
- raw gyro x sign vs d(enc)/dt -> measures the *physical* gyro orientation, from which the
  tool computes what `imu_invert` must be and what the loop's `gyro_x` will be;
- print the verdict as **the exact config values**: "measured: encoder_invert=true,
  imu_invert=false. config.json currently says: encoder_invert=true, imu_invert=false. MATCH"
  — or MISMATCH with the required edit.

**Phase C — actuation anchor (optional, motors on, props on, lever restrained by the
restrictor).** At minimum safe throttle, apply a small differential (M2 = min + 30,
M1 = min) for ~300 ms and check the encoder moves negative (M1 rises / unloads). This is
the only phase that tests the *mixer + ESC wiring + motor direction* chain. It is also the
phase that catches swapped motor connectors after a rebuild — a failure mode nothing else
in the system can see until first flight.

The tool ends with a single summary block you can paste into the session log. **Procedure
rule to adopt: signcheck is mandatory after any of:** mechanical reassembly, encoder
repositioning, IMU re-mounting or re-taring, motor/ESC rewiring, or any edit to
`bench.sensor_orientation`. Codify that list in the tool's docstring and in CLAUDE.md so
the rule survives you forgetting it.

### 4.4 Layer 3 — Runtime fuses (on the Pico, every armed run)

Layers 1-2 catch errors before and after; these catch the one that slips through, in the
first second of a run, before it becomes a slammed lever or a smoked ESC.

**Fuse 1 — control-direction fuse (runaway detector).** The signature of a sign error that
survived to arming is: output saturates AND the error grows. Both signals are already in
hand in the inner loop. Implementation sketch: maintain a small counter; each outer tick,
if `|rate_setpoint|` is at the angle output_limit AND `|ang_err|` increased vs the previous
tick, increment; any improving tick resets it. Trip threshold ~50 consecutive outer ticks
(0.5 s at 100 Hz) -> emergencyStop + descriptive exception ("control direction fuse:
error diverging under saturated output — check sign contracts / run signcheck"). The
startup transient is naturally immune (error *decreases* once motion starts; the brief
initial saturated-and-growing phase before the lever unsticks is why the threshold is 0.5 s
— validate the constant against your existing logs' startup profiles, where typical
unstick time is visible in the first records). This fuse is the difference between a sign
error costing you a 0.5 s twitch versus a full-power lever slam.

**Fuse 2 — cross-sensor coherence fuse.** Once per outer tick, accumulate
`s += gyro_x * (enc_now - enc_prev)`. Under the contract (`gyro_x = -phi_dot`) this sum
trends negative whenever the lever moves. If after the first 2 s of significant motion the
sum is emphatically positive -> trip. This is invariant I2 evaluated live, with the encoder
serving as the independent witness it was always meant to be. Cost: one multiply-add per
outer tick.

(Fuses 1-2 complement, not replace, the F5 staleness watchdog and F5b envelope protection —
those catch *dead* inputs, these catch *wrong-signed* ones. Four fuses, each a few lines,
each covering a failure family the others cannot see.)

### 4.5 Layer 4 — Software-in-the-loop simulation (desktop, every code change)

This is the highest-leverage item in Part 4, and the only one requiring real refactoring.
The control mathematics in `stabilize()` is currently welded to hardware imports (machine,
utime, the IMU driver), so the *only* way to execute it is on the bench with spinning
motors. That is why sign errors are discovered by flight test: flight test is the only
test that exists.

**The refactor:** extract the per-cycle control math into a pure function (or small class)
with no hardware imports — conceptually:

    rate_setpoint, output, m = control_cycle(state, imu_roll, gyro_x, dt, is_outer_tick, cfg)

`stabilize()` becomes a thin shell: acquire sensors, call `control_cycle`, dispatch motors,
record telemetry. `pid.py` and `mixer.py` are already pure (a quiet strength of the
existing architecture — this refactor is smaller than it sounds).

**The payoff — three test families that run on the PC in milliseconds:**

1. **Convergence test.** A 20-line lever plant model — `phi_ddot = -c*u + g_imbalance`,
   plus the known thrust-down geometry, integrated at 300 Hz — closed against the real
   `control_cycle` with the real `config.json`. Assert: from start_angle 51 deg, phi
   converges into the tolerance band and holds. This single test executes the *entire sign
   architecture end to end*. The plant constant c does not need to be accurate (you have
   not measured the thrust curve — DR-007); sign correctness is binary and any plausible c
   demonstrates it.
2. **Sign-mutation tests.** The test you really want, given the history: for each sign in
   the chain (gyro_x negation, error direction, mixer orientation, feedforward sign), flip
   it *in the test harness* and assert the convergence test now FAILS (diverges or
   oscillates). This proves your test can actually see each sign — a convergence test that
   still passes with the feedforward inverted (as the real bench effectively did) is
   measurably too weak, and the mutation suite tells you so. Note from F1: the feedforward
   mutation will likely *not* fail the basic convergence test (the term is only a 4%
   effect) — the mutation test for it must assert on the sharper property instead, e.g.
   that the reconstructed angle leads rather than lags the plant truth under simulated GRV
   delay. The mutation suite thus also documents *which contracts are load-bearing for
   stability* (gyro sign, mixer sign) *versus performance* (feedforward) — knowledge that
   today exists only in Part 0 of this review.
3. **Unit tests for the pure parts** (plain pytest, PC):
   - PID: integral clamps at +/-iterm_limit; derivative-on-measurement produces no kick on
     setpoint step; dt = 0 guard; first-call D = 0; output clamp; reset() clears state.
   - Mixer: differential symmetry; clamp boundaries; the `base +/- output_limit` invariant
     (F7) as a constructor test; int truncation behavior at the deadband.
   - control_cycle: outer tick scheduling (fires every Nth cycle), outer_dt arithmetic
     (a test that computes 300/100 Hz and asserts outer_dt == 0.010 **catches F2 forever**),
     rate_setpoint ZOH between ticks.

Run these in CI-of-one fashion: a `pytest` invocation before every deploy (the deploy skill
can refuse to upload on red — one line in `deploy.py`). For the first time, a sign error
becomes something a test fails *at your desk* instead of something the bench demonstrates
at 600 throttle.

### 4.6 Layer 5 — One authoritative home for the conventions

Reason 1 errors thrive when the convention is restated in many places (flight.py comments,
analysis scripts, tuning logs, your memory). Create a single source of truth — a
`conventions` section in `specification.json` (it already travels with every run!) or a
dedicated `conventions.json` deployed alongside config:

    "conventions": {
      "encoder_positive": "M1_end_down",
      "gyro_x_sign_vs_encoder_rate": -1,
      "angle_error": "measurement_minus_setpoint",
      "positive_rate_setpoint_means": "phi_decreasing"
    }

Then: `invariants.py` reads its expected signs from here (4.2); the signcheck tool prints
its verdicts against it (4.3); the SIL plant model is built from it (4.5); and when F9
Option A flips the convention, you change *this file* and every layer's expectation flips
in lockstep — the QA system cannot drift out of sync with the firmware because they share
one declaration. The flight code itself can stay as-is (it reads sensor_orientation as
today); the conventions block is the *contract record* the QA layers verify against.

### 4.7 Layer 6 — Process: the post-reassembly checklist as an executable artifact

The encoder flip was not a coding error; it was a *procedure* gap — hardware changed,
verification did not. The fix is a checklist that is annoying not to run because it is
executable. A `.claude/commands/recalibrate` skill (or just a markdown checklist in
`tools/`) sequencing:

1. `tools/signcheck.py` Phase A+B (sensors vs gravity and manual motion) -> updates/confirms
   `bench.sensor_orientation`.
2. Axis-center recalibration with the precision jig -> `bench.encoder.axis_center`.
3. Re-tare per the established GRV-basis procedure; jig residual check (~0.10 deg).
4. signcheck Phase C (actuation direction at min throttle).
5. One short low-stakes flight (duration_s 30, stock gains) -> pull -> **invariants gate
   must be all-green before any tuning or KPI judgment resumes.**

Steps 1-4 are ordered so each one anchors the next; step 5 closes the loop with the same
invariants that guard every future run. Total cost: ~15 minutes. Cost of the encoder-flip
incident it would have prevented: a misattributed inversion, a full retune against a
misdiagnosis, and several sessions of confounded results.

### 4.8 What this buys you, concretely

Map the defenses back onto the actual incidents:

| Incident (real) | Would have been caught by |
|---|---|
| Encoder flipped in overhaul, misattributed to IMU | signcheck Phase A/B (gravity + motion anchors localize the flip); checklist 4.7 forces the run |
| F1 feedforward sign inversion | invariant I2 (offline, first run after introduction); SIL mutation test (at the desk, before deploy) |
| F2 outer_dt arithmetic | control_cycle unit test on outer_dt; invariant I6 trends |
| 2026-04 inverted tracking (r = -0.87) | invariant I1 (existed in spirit — formalizing keeps it), plus I2 distinguishing which path |
| 2026-05-02 setpoint sign bug | invariant I4 |
| Hypothetical: motor connectors swapped during quad upgrade | signcheck Phase C; runtime Fuse 1 limits the damage to 0.5 s |
| Hypothetical: future contributor "simplifies" the gyro negation | SIL convergence test fails at desk; Fuse 1 catches it on bench; I2 catches it in analysis |

Defense in depth, but cheap depth: Layers 1, 2, 6 are scripts and JSON; Layer 3 is ~20
lines on the Pico; only Layer 4 asks for a refactor, and it is the refactor that also pays
dividends for M5 (a hardware-free control core is the thing you can carry to a real
vehicle and simulate properly).

---

## Suggested backlog ordering

**Phase 1 — Safety (do before the next tuning campaign; no retune required):**
- [ ] F3: reorder shutdown — `emergencyStop()` first in `finally`, telemetry close in its own
      try/except, LED truthful. Test the SD-death path deliberately.
- [ ] F4: B+Y abort in `stabilize()` (and F15: in the prespin ramp); fix the docstring.
- [ ] F5: sensor-staleness watchdog (>100 ms without a gyro packet -> stop + raise).
- [ ] F5b: encoder envelope protection (|enc| > ~70 deg -> stop).
- [ ] F7-minimum: startup assertion `base +/- output_limit` within `[throttle_min, throttle_max]`.
- [ ] QA-Fuse: control-direction fuse + cross-sensor coherence fuse in stabilize() (4.4) —
      same family as F5/F5b, install together.

**Phase 1.5 — QA scaffolding (before Phase 2, so the correctness fixes land under guard):**
- [ ] QA-I: promote `tmp/check_gyro_sign.py` -> `pipelines/flight-analyser/scripts/invariants.py`
      with the I1-I9 contract set (4.2); wire into run.py before scoring; failed invariant
      blocks KPI scoring.
- [ ] QA-C: `conventions` block in specification.json as the single declared contract (4.6);
      invariants.py reads expectations from it.
- [ ] QA-S: `tools/signcheck.py` — gravity / manual-motion / actuation anchored checks
      (4.3); adopt the "mandatory after mechanical change" rule in CLAUDE.md.
- [ ] QA-U: pytest suite for pid.py + mixer.py (4.5 family 3); deploy.py refuses upload on red.

**Phase 2 — Correctness (one session; ends with a baseline re-confirmation):**
- [ ] F2: `outer_dt = outer_ticks / rate_loop_hz`; translate ki/kd in config to preserve
      behavior; note in tuning log.
- [ ] F8: dt from `ticks_us`.
- [ ] F1: feedforward sign — A/B/C (flipped @ 12 ms vs lead 0 vs current), >= 3 runs/arm;
      then optionally re-sweep lead upward with the correct sign.
- [ ] F9: decide Option A (textbook convention rewrite) vs Option B (documented contract);
      if A, fold F1's fix into it and re-check analysis-script sign assumptions.

**Phase 2.5 — SIL foundation (pairs naturally with F9 Option A, since both touch the same lines):**
- [ ] QA-R: extract `control_cycle()` as a hardware-free function (4.5 refactor).
- [ ] QA-SIM: lever plant model + convergence test with real config.
- [ ] QA-MUT: sign-mutation suite — every load-bearing sign provably visible to a test.
- [ ] QA-F2: outer_dt unit test (locks in the F2 fix permanently).

**Phase 3 — Robustness / tuning headroom:**
- [ ] F6: conditional integration (or back-calculation) + iterm_limit -> output-unit
      semantics; then revisit whether ki can rise for stiffer hold (the +/-9 deg objective).
- [ ] F11: gate inner/outer on `gyro.updated` / `game_quaternion.updated`.
- [ ] F10: D-term LPF (after F8, so the cutoff is chosen against the true noise floor).
- [ ] F12-cheap: offline experiment — full roll formula vs `2*atan2(qi,qr)` on existing
      logs; feeds IDEA-003.

**Phase 4 — Before hardware changes:**
- [ ] F7-full: differential-preserving mixer desaturation + telemetry flag.
- [ ] F7-footgun: motor count / pair-enable in config before M3/M4 are mounted; plan a
      reduced-gain first arm.
- [ ] F12-full + F16: general roll extraction, wrap handling, telemetry flags — prerequisites
      for M5.

**Cosmetic, anytime:** F13 (LED comment), F6c (`ki=0` integrator), F14 (note only).

---

## Closing assessment — the project as a whole

A step back from line numbers. This section judges the project against what it is *for* —
a vehicle for refreshing control engineering and learning flight dynamics — not against
product standards.

### Strong sides

**1. The methodology is the standout — genuinely better than most professional teams.**
Decision records with amendments, structured tuning sessions with iteration tracking, KPI
specifications with pass/good/excellent tiers, config + spec snapshots frozen into every
run folder, black-box telemetry with every PID term logged. The project produces
*evidence*, not vibes. The clearest proofs are the uncomfortable moments: DR-013's friction
data was invalidated by its own author when wire interference was discovered, and the
+/-9 deg session was **abandoned** after 9 failures with a documented root cause instead of
cherry-picking the 3 passes. That is scientific integrity most people do not apply to their
day jobs, let alone pet projects.

**2. The encoder-as-ground-truth decision is the single best architectural call.** Most
hobbyists tune flight controllers blind against the IMU itself — and you cannot see IMU
errors with the IMU. The independent reference is what made the inverted-tracking diagnosis
possible, made the IMU-ENC bias floor measurable, and made this entire review verifiable
instead of speculative. Every empirical claim in this document rests on that one sensor.

**3. Honest scoping.** Single axis, constrained travel, a restrictor, staged milestones
M1 -> M4. Degrees of freedom were reduced until each effect was individually observable —
textbook pedagogy, self-administered. The systems instincts show too: dual-core motor
command loop, binary telemetry with preallocation *after* diagnosing FAT latency, the
data-driven inner-loop redesign. Correct diagnoses followed by proportionate fixes.

### Weak sides

**1. Empiricism without theory anchoring — for the stated learning goal, the big one.**
The project tunes brilliantly but models nothing. No plant model, no measured thrust curve
(DR-007 deferred), no system identification, no frequency-domain design. Every gain was
found by sweep. That exercises the experimental muscle hard and the analytical muscle
barely at all — and it is exactly why F1 and F2 survived: **with no model predicting what
the response should look like, there is no anomaly when it looks wrong.** A working loop
became the only oracle, and a working loop tolerates 10% dt errors and an inverted
feedforward indefinitely.

The highest-value learning move available: a system-ID session — step responses at several
throttles, fit a second-order model from telemetry that is *already being collected* —
then **compute** gains via pole placement and compare against the empirically tuned ones.
Where they agree, the plant is understood; where they disagree is where the real learning
lives. The entrance ticket already exists: the 0.038 Hz oscillation found by FFT
(2026-05-31) was the project's first frequency-domain observation. Injecting a small
sine/chirp on the setpoint and measuring the closed-loop frequency response empirically
would be a superb next experiment.

**2. The mechanical plant is the known weakest link, and it now gates everything.** The
wire harness (no slip ring) shifts equilibrium every reset, killed the +/-9 deg campaign,
and contaminated the friction characterization. The project has reached the point where
**the bench's repeatability ceiling is mechanical, not algorithmic** — a slip ring or
serious wire management buys more KPI improvement than any conceivable gain tweak.
IDEA-003 is correctly aimed at exactly this.

**3. Failure-path engineering lagged feature work.** Parts 1 and 4 cover the specifics; the
project-level point is about habit, not bench risk: in the domain being learned, **the
failure path is most of the product**. Real flight controller work is overwhelmingly
failsafes, sanity checks, and degraded-mode behavior — the control law is the easy 20%.
Treating Phase 1 of this backlog as domain learning rather than chores frames it correctly.

**4. Watch the single-axis comfort zone.** The 22 deg -> 6.9 deg -> 2.2 deg MAE journey
taught enormous amounts. The marginal learning of the next kd sweep at +/-10 deg is
approaching zero — polishing a solved problem is comfortable and the bench is good at it.
Flight *dynamics* — the stated learning target — starts where axes couple, and the lever
physically cannot teach it. Define explicit "bench is done" criteria (suggestion: +/-9 deg
demonstrated *after* the slip ring, QA layers installed, one model-based tuning pass) so
the bench stays a stage and does not become the destination.

**5. Dependency depth is shallower than the dependence.** The BNO085 driver is a large
third-party codebase whose quirks (packet bundling overwrites samples, the 50 ms wait path,
per-report accuracy that is ignored, `.updated` flags that are unused) sit directly under
the control loop's correctness. Forking it as a submodule was right; the contract-level
understanding is partial. One focused session writing down "what exactly does
`update_sensors()` guarantee" would retire a whole category of latent surprise.

### If I were steering

The arc: **Phase 1 safety + QA scaffolding -> slip ring -> system ID + model-based retune
(the learning centerpiece) -> SIL refactor -> four motors.** That sequence fixes the
foundation, removes the confounder, converts the project from "tuning practice" to
"control engineering practice," and only then scales the hardware.

The meta-observation: this project's real strength is that it is built to *find out it is
wrong* — the encoder, the logs, the DRs, this review. Its real weakness was that until now
it could only find out **after** being wrong on hardware. The backlog above closes exactly
that gap. For a pet project, that is a remarkably coherent place to be.

---

## Appendix — Verification data

Run: `test_runs/flights/2026-06-07_13-48-31` (1545 records, 119.9 s, sample_every=20 -> 76 ms
median record interval). Script: `tmp/check_gyro_sign.py`.

```
corr(IMU_roll, ENC_ROLL)        = +0.9955
corr(GYRO_X, d(ENC_ROLL)/dt)    = -0.6099
RMS(imu_roll - enc)                     = 0.8235 deg
RMS(imu_roll + gyro*lead - enc)  [code] = 0.8383 deg
RMS(imu_roll - gyro*lead - enc)  [flip] = 0.8180 deg
lag scan (76 ms resolution): best fit at lag 0 -> GRV lag below telemetry resolution;
the feedforward A/B/C (F1) is the instrument that can actually resolve whether a lead
term earns its keep.
```

Spot check, first record of the run: ENC_ROLL falling 49.7 -> 43.5 over 76 ms
(phi_dot ~ -82 deg/s) while GYRO_X = +56.2 — the gyro signal in code convention is the
negative of the physical encoder-convention rate, as required by the stability analysis in
Part 0 and as relied upon (incorrectly) by the feedforward term in F1.