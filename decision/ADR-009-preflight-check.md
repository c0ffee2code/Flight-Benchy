# ADR-009: Pre-flight Sign Chain Check

**Status:** Accepted — pending implementation
**Date:** 2026-02-19
**Context:** Cascaded PID commissioning (M4) required diagnosing three independent sign/gain bugs, each costing one full arm-fly-disarm-analyse cycle (~15 min per iteration)

## Context

Commissioning the M4 cascaded PID loop required finding and fixing three bugs in sequence:

1. **Outer loop sign** — `compute(feedforward_roll)` instead of `compute(-feedforward_roll)` → pure positive feedback
2. **Combined gain** — `1.5 × 1.0 = 1.5` too low to resist lever gravity → free-fall from horizontal
3. **Mixer sign** — `m1 = base + output` drove the wrong motor high → lever pinned to ramp

Each bug was diagnosed the same way: arm motors, observe failure, disarm, pull SD card, copy to PC, run analyser. Minimum 10–15 minutes per iteration just in setup/teardown, before any analysis time. All three bugs were detectable from pure arithmetic — no motors, no SD card, no IMU movement required.

The same failure mode will recur any time:
- Mechanical setup changes (motors swapped, lever inverted, IMU remounted)
- Gains are restructured (new loop added, sign convention changed)
- Code is refactored without end-to-end verification

Aviation pre-flight checklists exist precisely to catch this class of error: known-good invariants verified before committing to an irreversible action (takeoff / arming).

## Decision

Add a **pre-flight check** executed after `wait_for_arm()` and before `arm_motors()`. No motors spin during the check. The check:

1. Creates the timestamped run folder on the SD card (RTC read happens here, not at `wait_for_go()`)
2. Simulates the controller at ±N° hypothetical errors using the current gain values
3. Reads the live IMU and encoder in a loop — operator sweeps the lever by hand to verify sign chain against physical reality
4. Writes all results to `preflight.txt` in the run folder
5. Waits for explicit operator confirmation (button A) or abort (B+Y)

The operator can **manually sweep the lever** side-to-side while the check loops — sign chain updates live so any mismatch between displayed direction and observed lever response is immediately apparent.

### SD card and session folder

The run folder (`/sd/blackbox/YYYY-MM-DD_hh-mm-ss/`) is created at pre-flight time, not at `wait_for_go()`. This means:

- `preflight.txt` — written during pre-flight (always present)
- `config.yaml` — written at session start (after A pressed, before Go)
- `log.csv` — written during stabilisation

If the operator aborts (B+Y), the folder exists on the SD card but contains only `preflight.txt` with the abort record. This is useful — an aborted pre-flight with `[✗]` failures is evidence of a configuration problem.

`SdSink` gains a new method `write_preflight(text)` called before `init_session()`.

## Sign chain invariants

For the current cascaded architecture (setpoint = 0°, lever below horizontal = negative IMU roll):

| Condition | Expected |
|-----------|----------|
| `IMU_ROLL < 0` (lever below horizontal) | `ang_err > 0` |
| `ang_err > 0` | `rate_sp > 0` |
| `rate_sp > 0` | `pid_out > 0` |
| `pid_out > 0` | `M2 > M1` (lever lifts) |
| `pid_out > 0` | `M1 < BASE_THROTTLE` and `M2 > BASE_THROTTLE` |

These are printed with a `[✓]` / `[✗]` marker. Any `[✗]` is a hard stop — the operator must abort, fix the code, and re-run.

## Log format (`preflight.txt`)

```
=== Pre-flight check — 2026-02-19 21:30:00 ===
Gains: angle kp=1.5 kd=0.2  rate kp=2.5  base=250 min=70 max=400

Simulated sign chain (P-term only, no integrator state):
  IMU_ROLL= -10°  ang_err=+10.00  rate_sp=+15.00  pid_out=+37.50  M1=212 M2=288  [OK lifts]
  IMU_ROLL=   0°  ang_err= +0.00  rate_sp= +0.00  pid_out= +0.00  M1=250 M2=250  [OK neutral]
  IMU_ROLL= +10°  ang_err=-10.00  rate_sp=-15.00  pid_out=-37.50  M1=288 M2=212  [OK lowers]

Live sweep log (operator moved lever by hand):
  T=  1.2s  ENC=  0.12°  IMU= -4.39°  ang_err= +4.39  pid_out=+16.47  M1=233 M2=267  [OK]
  T=  1.5s  ENC=-31.40°  IMU=-30.21°  ang_err=+30.21  pid_out=+91.70  M1=159 M2=341  [OK]
  T=  1.8s  ENC=-61.88°  IMU=-60.64°  ang_err=+60.64  pid_out=+181.9  M1= 70 M2=400  [OK sat]
  T=  2.1s  ENC=  0.05°  IMU= -4.41°  ang_err= +4.41  pid_out=+16.53  M1=233 M2=267  [OK]
  T=  2.4s  ENC=+30.22°  IMU=+28.95°  ang_err=-28.95  pid_out=-86.85  M1=337 M2=163  [OK]

Result: PASS
Operator confirmed: A pressed at T=4.1s
```

On abort:
```
Result: ABORT — operator held B+Y at T=3.2s
```

On invariant failure (FAIL prevents A from proceeding):
```
  IMU_ROLL= -10°  ang_err=+10.00  rate_sp=+15.00  pid_out=+37.50  M1=288 M2=212  [FAIL lowers]

Result: FAIL — simulated invariants violated, A disabled
```

## What this catches

| Bug class | Caught by |
|-----------|-----------|
| Outer loop sign inverted | `ang_err` sign mismatch on simulated row |
| Mixer sign inverted | M1/M2 labelled `[✗ lowers]` when lever is below horizontal |
| Combined gain too low | `pid_out` column visibly tiny at 10° error |
| PID integral wound from previous run | `ang_err ≠ rate_sp / kp` (non-zero I term visible) |
| Wrong IMU axis used | Live reading shows wrong sign when lever tilted by hand |
| IMU not tared at horizontal | Persistent `ang_err` at ENC=0° |

## Implementation notes

- Simulated rows use `angle_pid.kp × rate_pid.kp` directly — no `compute()` call, avoids mutating integrator state before the run begins
- Live reading calls `imu.update_sensors()` and `encoder.read_raw_angle()` every ~250 ms, does **not** call `angle_pid.compute()` or `rate_pid.compute()`
- Mixer clamping applied to simulated and live M1/M2; saturation marked `[OK sat]` (expected at large angles)
- If any simulated row fails an invariant, log `[FAIL reason]`, set a flag, and disable button A — operator can only abort via B+Y
- Live sweep rows are appended to the in-memory log buffer as they arrive; the full buffer is flushed to `preflight.txt` on SD when the operator presses A or B+Y
- LED stays blue during pre-flight; turns green only after `arm_motors()`
- `SdSink.open_session()` (new method, split from `init_session()`) creates the folder and opens `preflight.txt`; `init_session()` then opens `config.yaml` and `log.csv` as before

## Interaction with ADR-012 (differential expo)

When expo is enabled in `LeverMixer`, the simulated M1/M2 values in the sign chain will reflect the shaped output, not the raw `pid_out`. The sign chain invariants remain valid (expo is monotonic and preserves sign), but the displayed M1/M2 values at ±10° will be lower than `base ± kp × kd × 10` would suggest. The preflight log should record the expo parameter so the operator can interpret any smaller-than-expected motor differential correctly.

## What it does NOT check

- Motor direction under actual thrust (gyro feedback loop direction) — tested separately if needed
- ESC health, motor spin-up time, DShot signal integrity
- SD card availability (already checked at startup by `SdSink` fail-fast, see ADR, commit `60e39eb`)
- Sensor connectivity (already verified at I2C init)

## Consequences

- **Arm sequence changes:** `wait_for_arm()` → **`run_preflight_check(sink)`** → `arm_motors()` → `wait_for_go()` → `init_session()`
- Run folder and RTC timestamp created at pre-flight time; `preflight.txt` always present in every run folder
- Aborted pre-flights leave a folder with only `preflight.txt` — useful evidence, not noise
- Every run folder is now a complete audit trail: pre-flight state, config, and telemetry in one place
- Adds ~5–10 seconds to the arm sequence for a careful operator; a one-sweep lever check takes ~3 seconds
- Operator retains full authority — a PASS does not guarantee correct behaviour, but a FAIL guarantees a bug and prevents arming
