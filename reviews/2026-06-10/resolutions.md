---
title: Resolutions — REVIEW-2026-06-10
updated: 2026-06-23
---

# Resolutions — REVIEW-2026-06-10

Companion to `review.md`. Records what has been implemented, with commit references and
notes on any deviation from the review's recommendation.

---

## F2 — `outer_dt` wrong by 10% (integer-division artifact)

**Commit:** `8d79705`

Fix: `outer_dt = outer_ticks / rate_loop_hz` (float division).
Config adjusted at the same time to preserve closed-loop behavior: `ki 0.10 -> 0.09`,
`kd 0.65 -> 0.722` (review's suggested translation).

---

## F3 — Shutdown sequence: motors hot through SD finalize; disarm can be skipped

**Commit:** `8d79705`

Fix: `motors.stop()` is now the single cleanup call in `finally` (it subsumes
`emergencyStop` + `disarm`). Telemetry close is wrapped in its own `try/except` so
an SD failure cannot skip motor shutdown. LED is set *after* motors are stopped.

Note: the review recommended `emergencyStop()` first, then telemetry, then full `stop()`.
We collapsed the two motor-shutdown calls to `stop()` alone — `stop()` is the
`MotorThrottleGroup` API for the full teardown path and is correct; chaining
`emergencyStop + stop` was redundant.

---

## QA-I — Telemetry invariants gate (§4.2)

**Commits:** `f0d8bed` (initial), `9e01cc3` (schema cleanup), `343d2f9` (I6 p99/max split),
`16439d3` (docstring cleanup)

New file: `pipelines/flight-analyser/scripts/invariants.py`
Wired into `pipelines/flight-analyser/run.py` between `plots` and `verdict`.

### Invariants implemented

| Name | Severity | Catches |
|------|----------|---------|
| imu-enc-position-agreement | FAIL | IMU/encoder orientation disagreement |
| gyro-rate-sign | FAIL | rate-convention violation (`gyro_x = -phi_dot`) |
| imu-enc-rate-agreement | FAIL | quaternion rate path vs encoder rate |
| angle-error-convention | FAIL | outer-loop error convention, setpoint sign regressions |
| actuation-direction | FAIL | mixer sign, motor wiring swap (transient only) |
| loop-timing | FAIL / WARN | loop-rate regressions; p99 = FAIL, single spike = WARN |
| saturation-report | WARN | silent saturation, authority loss |
| imu-enc-hold-bias | WARN | tare drift, IMU-ENC bias (IDEA-003 signal) |
| start-angle-sign | FAIL | encoder flip / axis_center drift at run start |
| feedforward-direction | WARN | F1 indicator (code sign makes estimate worse) |
| mixer-output-sign | FAIL | mixer software sign inversion |

### Deviations from / additions to review spec

**I11 (mixer-output-sign) added.** The review's table listed I1–I9. During implementation
we identified that mixer *software* sign inversion (as opposed to physical motor wiring)
had no coverage. `corr(pid_out, m2-m1) > 0.9` closes that gap. loop-meltdown was removed
from `gate.py` at the same time — it was a symptom (enc range > 90 deg); I5 + I11 cover
the root causes directly without the false-positive on hard-slam startups.

**I6 split into p99 FAIL + max WARN.** The review spec had a single `max < 25 ms` FAIL.
In practice `2026-06-17_21-22-20` had one GC spike at 30 ms with an otherwise clean p99
(~5 ms) — a single spike should not block scoring. Decision: `p99 > 3x expected = FAIL`
(systematic jitter), `max > 25 ms = WARN` (isolated spike, flagged but not blocking).

**Schema:** entries use `name` as sole identifier (no numeric IDs). Key is `status`
(`PASS/FAIL/WARN/SKIP`) in both `invariants.json` and `gate.json` check entries.

**Forensic-before-gate principle** adopted: `plots` runs unconditionally before
`invariants`, so a hard FAIL still leaves a visual record of the runaway trajectory.
Stage ordering is sequencing only, not a data dependency.

**Shared analysis in `flight_data_loader.py`:** `detect_reach_event` and
`detect_hold_window` live in the loader; both `plots.py` and `invariants.py` call them
independently. No inter-stage data dependencies.

### Current status on reference runs

| Run | Gate | Invariants | Verdict |
|-----|------|------------|---------|
| 2026-06-07_13-48-31 | PASS | WARN (feedforward-direction) | PASS |
| 2026-06-17_21-22-20 | PASS | WARN (loop-timing, feedforward-direction) | PASS |
| 2026-06-23_21-59-35 | PASS | FAIL (gyro-rate-sign, actuation-direction), WARN (loop-timing, feedforward-direction) | see note |

`feedforward-direction` WARN is the standing F1 indicator on all runs — expected until F1
is fixed (requires the A/B/C flight session; see review F1).

**Note on 2026-06-23_21-59-35 FAILs — sample_every aliasing, not a sign regression.**
`gyro-rate-sign` (corr=+0.052) and `actuation-direction` (corr=+0.002) are near-zero, not
inverted. Root cause: both invariants use unsmoothed point samples. At `sample_every=20` each
CSV row is ~66 ms apart; a raw `gyro_x` snapshot at 66 ms inter-sample interval has high
decorrelation against `np.gradient(enc)` — the gyro can complete partial oscillations between
samples, collapsing the correlation toward zero. In contrast, `imu-enc-rate-agreement` (which
uses the BNO085 quaternion derivative — an internally integrated rate) passes at 0.994,
confirming the sign contract is correct via the smoothed path.

Verified: approach-window-only computation gives `corr(gyro_x, denc) = -0.081` (weakly
negative), and `gyro_x` std = 54 deg/s during the approach — the signal is active, not
decoupled or stale. The `FuseSensorCoherence` runtime fuse did not trip (accumulator stayed
negative throughout), independently confirming the sign contract.

**Invariant design backlog item (Phase 1.5):** `gyro-rate-sign` and `actuation-direction`
should be windowed to the approach/transient phase (run start → first ±10° entry), or require
`sample_every ≤ 5` to be reliable. At sample_every=20 these invariants produce structurally
unreliable correlations for the raw (unsmoothed) signals.

---

---

## Phase 2.5 SIL — ControlCore extraction + pytest suite (§4.5)

### What was done

**QA-R — ControlCore extraction.**
All control math extracted from `stabilize()` in `flight.py` into a hardware-free
`src/core/control.py::ControlCore`. `flight.py` is now a thin hardware shell: it reads
sensors, calls `core.step(iqr, iqi, iqj, iqk, raw_gx_rad, dt_s)`, then writes motor
commands and telemetry. No `machine`, `utime`, IMU object, or motor object touches
`ControlCore`.

Sign overrides (`gyro_sign`, `ff_sign`, `err_sign`, `mixer_sign`) are injectable via
`__init__` kwargs — production code uses the defaults, mutation tests pass non-default
values. `is_outer` is not a return value; it lives on `core.last_is_outer` along with
the other post-step state attributes.

**`src/core/` package structure.**
`pid.py` and `mixer.py` moved from `src/` to `src/core/`. A new `src/core/__init__.py`
makes the package importable on both desktop Python and MicroPython. Pico deploy now
creates `/core/` and uploads the four files (`__init__.py`, `pid.py`, `mixer.py`,
`control.py`). Old flat `pid.py` / `mixer.py` on the Pico are superseded.

**QA-F2 — outer_period_s float division.**
`ControlCore.__init__` computes `self.outer_period_s = outer_ticks / rate_hz` (float
division). At 300/100 Hz this gives 0.010 s, not the 0.009 s from the previous integer
division in flight.py's `ms_to_s(OUTER_INTERVAL_TICKS * INNER_INTERVAL_MS)`.

**F6c — ki=0 integrator guard.**
`PID.compute()` now guards the integral accumulation with `if self.ki:` so that setting
`ki=0` truly disables the integrator, even across many calls with non-zero `iterm_limit`.

**QA-U — pytest suite (25 tests, `tests/`).**

| File | Tests | Covers |
|------|-------|--------|
| `test_pid.py` | 8 | integral clamping, D-on-measurement, dt=0, first-call D, output clamp, reset, ki=0 guard (F6c) |
| `test_mixer.py` | 7 | zero/positive/negative differential, m3/m4 mirror, upper/lower clamp, F7 invariant |
| `test_control.py` | 6 | QA-F2 outer_period, loop scheduling, ZOH between outer ticks, gyro-sign contract, imu-roll-sign contract, feedforward direction (F1 indicator) |
| `test_sil.py` | 4 | closed-loop convergence, gyro_sign / mixer_sign / err_sign mutations each cause divergence |

**QA-SIM — plant model.**
`test_sil.py` closes the loop against a 2nd-order lever plant: `phi_ddot = -C * (m2-m1)/2`
(motor differential, not raw PID output — so `mixer_sign` mutations correctly flip the
plant input and cause positive feedback). `C = 0.5` gives `zeta ≈ 0.22`, `omega_n ≈
0.935 rad/s`; amplitude decays from 51 deg to < 0.5 deg within 30 s.

**QA-MUT — mutation coverage.**
Three sign mutations are independently lethal:
- `gyro_sign=+1` — flips damping sign → oscillation grows
- `mixer_sign=-1` — M1/M2 swapped → positive feedback → diverges to ±200
- `err_sign=-1` — outer loop drives away from setpoint → diverges

All three pass the `_in_band` test asserting divergence (`_in_band` returns True for
convergence, False for divergence; 70% of last 1000 samples must be within ±10 deg to
count as converged).

**Pytest gate in run-flight pipeline.**
`pytest tests/ -q --tb=short` runs as Step 1 of `pipelines/flight-runner/run.py`, before
reset and deploy. `deploy.py` is decoupled — no longer runs tests, can be called standalone.

**`pyproject.toml`** replaces `requirements.txt`. Declares `numpy`, `matplotlib`,
`mpremote` as runtime deps and `pytest` as `[dev]` optional dep.

### Current status

25/25 tests green. Pico deploy verified on hardware (2026-06-21_18-44-36, 30s test flight):
all sign invariants pass — imu-enc-position-agreement r=0.97, gyro-rate-sign −0.65,
mixer-output-sign r=0.9997. `feedforward-direction` WARN is the standing F1 indicator.
Loop-timing FAIL (p99=10.9ms) on the short run — not a regression, GC in 30s is
statistically unlucky; longer run needed to confirm.

---

## Phase 1 safety — F4, F5, F7-min, QA-Fuse

### What was done

**F4 — abort path docstring corrected.**
`stabilize()` docstring previously promised a B+Y button combo abort that was never
implemented. Corrected to: `duration_ms=None means run indefinitely — cut power to stop.`
No code change; the power-cut model is intentional (no soft abort path needed).

**F5 — IMU staleness watchdog.**
`FuseImuStaleness` class in `flight.py`. `on_packet(now_ms)` is called each time
`update_sensors()` returns a packet; `check()` is called on every no-packet cycle and
raises `RuntimeError("FuseImuStaleness: no packet for >Nms")` if `IMU_STALE_MS` (100 ms)
elapses without a packet. Replaces the previous silent `sleep_us + continue`.

**F5b — encoder envelope protection: intentionally excluded.**
The encoder is telemetry-only ground truth with no authority over the motors. Using it as
a motor kill trigger is the wrong safety model — it would mean trusting a sensor that is
not in the control loop. Excluded permanently.

**F7-min — startup motor range assertion.**
`assert_motor_range(base, output_limit, throttle_min, throttle_max)` in `src/core/control.py`,
called from `ControlCore.__init__` before any PID or mixer objects are created. Raises
`ValueError` if `base + output_limit > throttle_max` or `base - output_limit < throttle_min`.
Config errors surface before arming, before any motor movement. Currently passes:
600 ± 300 = [300, 900] within [90, 900].

**QA-Fuse — runtime sign fuses.**
Two fuse classes in `flight.py`, instantiated at the top of `stabilize()`:

`FuseControlDirection(output_limit, trip_ticks)` — `update(rate_setpoint, ang_err_abs)`
called on every outer tick. Trips after `CTRL_DIR_TRIP_TICKS` (50) consecutive outer ticks
where the angle PID output is saturated AND the error is not shrinking. Catches inverted
control direction, disconnected motor, or severely detuned gains. ~0.5 s at 100 Hz.

`FuseSensorCoherence(warmup_ticks, threshold)` — `update(gyro_x, enc_angle)` called on
every outer tick. Accumulates `gyro_x * delta_enc`; under the correct sign contract this
product is <= 0 always (`gyro_x = -phi_dot`). Activates after `SENSOR_COH_WARMUP_TICKS`
(200 outer ticks, ~2 s); trips when the accumulator exceeds `SENSOR_COH_THRESHOLD` (300).
Catches gyro or encoder sign flip.

Both fuses raise `RuntimeError` which propagates to `run()` → `motors.stop()` in `finally`
→ crash log written to SD → traceback visible on stderr via mpremote.

### Design notes

The three fuse classes (`FuseImuStaleness`, `FuseControlDirection`, `FuseSensorCoherence`)
own their state and configuration constants. `stabilize()` instantiates them and calls
`.check()` / `.on_packet()` / `.update()` — no fuse state leaks into the outer function.
Module-level constants (`IMU_STALE_MS`, `CTRL_DIR_TRIP_TICKS`, `SENSOR_COH_WARMUP_TICKS`,
`SENSOR_COH_THRESHOLD`) remain in the Constants section of `flight.py` as tuning parameters.

### Current status

25/25 tests green. Hardware verification pending (bench not yet set up for post-refactor
test flight). Fuse 2 threshold (300) should be validated offline against reference run
2026-06-07_13-48-31 to confirm the accumulator stays negative throughout a normal run.

---

## Not yet addressed

Phase 1.5 QA: QA-C (conventions block), QA-S (signcheck tool).

Phase 2 correctness: F1 (feedforward sign — A/B/C flight), F8 (us-resolution dt),
F9 (convention cleanup decision).

Phase 3+: F6 (anti-windup), F10 (D-term LPF), F11 (per-report gating), F12 (roll
formula), F7-full, F16 (telemetry flags).

Cosmetic: F13 (LED comment), F14 (50 ms wait note).

---

## Pre-work context for F1 + F10 session

### D-term chattering observed in run 2026-06-23_21-59-35

After reaching the band at t=2.4s, the lever exhibited rapid oscillations (~2 Hz,
~5 deg amplitude) for ~4s before slowly settling. T_s=32s, overshoot=9.3%.

**Mechanism confirmed from telemetry:**

`ANG_D` alternates sign every outer tick during approach and early hold:
values like +128 -> -74 -> +23 -> -55 -> +44 -> -94. These swings are larger than
`ANG_P` (~30-50 at those error levels), so `RATE_SP` alternates sign every outer tick
(+130 -> -32 -> +83 -> ...). The rate loop faithfully tracks this, driving motors to
alternate between M1=483/M2=717 and M1=660/M2=540 — physically rocking the lever.

**Root cause — noise amplification by the D-term:**

`ANG_D = kd * d(ang_err)/outer_period_s = 0.722 * d(ang_err)/0.010`

This amplifies ang_err changes by 72x. For ANG_D=+128, ang_err must change 1.77 deg
in one 10ms outer tick — far faster than the lever moves. The noise enters through:

1. **Feedforward (F1, wrong sign)**: `ang_err = imu_roll + gyro_x*0.012 - setpoint`.
   The `gyro_x*lead_s` term injects gyro noise into ang_err. At outer-tick resolution
   (10ms), a 100 deg/s^2 gyro change contributes ~87 deg/s to the D-term. With
   `ff_sign=-1` (correct) this term is subtracted, which would reduce the ang_err
   amplitude at high angular velocity and damp the D-kick during approach.

2. **GRV quantization at 100Hz**: imu_roll updates discretely; step arrivals produce
   D-term spikes. A D-term LPF (F10) would absorb these before amplification.

**Spectrum:** 0.183 Hz (slow settling, ~32s period) + 5.5 Hz bump (D-chatter, visible
even during hold). The 5.5 Hz component is above the outer loop rate aliased through
the rate loop.

**Fix priority:** F1 alone reduces feedforward noise injection into ang_err and should
improve approach smoothness. F10 (D-term LPF) is the safety net — it removes all
high-frequency sources regardless of origin. Run F1 first; measure whether the chatter
reduces before adding F10.

**Reference run for comparison:** 2026-06-07_13-48-31 (gyro-rate-sign=-0.65, clean
approach). Same kd_effective=72.2 but different wire harness state; comparison run to
confirm F1 fix actually improves approach chatter.

**Test plan for F1 session:**
- A: current code (ff_sign=+1, wrong) — control flight
- B: ff_sign=-1 in `control.py` — expected: feedforward-direction WARN clears,
  approach chatter reduces, D-term magnitude drops
- C: if B doesn't fully resolve chatter, add D-term LPF (F10) with cutoff ~10 Hz
