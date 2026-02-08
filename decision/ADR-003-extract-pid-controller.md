# ADR-003: Extract PID Controller from main.py

**Status:** Accepted
**Date:** 2026-02-08
**Context:** Refactoring main.py to separate control logic from orchestration

## Context

After implementing M1 (ADR-001), `main.py` has grown to mix three concerns:

1. **PID controller** — computation, state (integral, prev_error as module globals), reset, gains, anti-windup clamping
2. **State machine** — DISARMED → ARMING → READY → STABILIZING flow, button handling, timing
3. **Hardware init** — I2C bus, encoder, motor pins, display constants

The PID logic is the part most likely to grow as the project progresses:
- **M2** (BNO085 as input): PID gains will need retuning, the controller itself doesn't change but having it isolated makes experimentation easier
- **M2a** (telemetry): logging individual P/I/D contributions requires access to internal PID state — a class exposes this cleanly
- **M4** (cascaded PID): requires two independent PID instances (angle loop + rate loop) — impossible with module-level globals

## Decision

### Extract a `PID` class into `pid.py`

```python
class PID:
    def __init__(self, kp, ki, kd=0.0, integral_limit=200.0):
        ...

    def compute(self, error, dt):
        # Returns output, updates internal state
        ...

    def reset(self):
        # Zeros integral and prev_error
        ...
```

**Why a class instead of functions + globals:**

| Aspect | Globals (current) | Class (proposed) |
|--------|-------------------|------------------|
| State management | `global integral, prev_error` — implicit, error-prone | Instance attributes — explicit, encapsulated |
| Multiple instances | Impossible without renaming globals | `pid_angle = PID(...)`, `pid_rate = PID(...)` — needed for M4 |
| Gains visibility | Constants scattered in main.py | Constructor params — visible at instantiation site |
| Telemetry access | Would need to return tuple or more globals | `pid.last_p`, `pid.last_i`, `pid.last_d` — clean read access for M2a |
| Testing | Must mock globals | Create instance, call compute, assert output |

### What stays in main.py

`main.py` becomes a pure orchestrator:

- Hardware init (I2C, encoder, motors, buttons)
- All configuration constants (calibration, throttle limits, PID gains, timing)
- `clamp()` utility (generic, used for motor output mapping)
- `buttons_by_held()` helper
- 4-state machine in `main()`
- Motor output mapping (`BASE_THROTTLE ± pid_output` → m1, m2) — this is the mixer; its extraction is planned separately as M3

### What moves to pid.py

- PID state (integral, prev_error)
- `pid_compute()` → `PID.compute()`
- `pid_reset()` → `PID.reset()`
- Anti-windup clamping logic

## Consequences

### Positive

- `main.py` reads as a clear state machine — hardware init, then a loop of: read sensor → compute → actuate → display
- PID gains are visible at the instantiation site: `PID(kp=5.0, ki=0.5, kd=0.0)`
- Ready for M4 cascaded PIDs without refactoring main.py again
- PID internals accessible for telemetry logging (M2a) without adding globals
- `pid.py` is independently testable

### Risks

- Additional file to upload to Pico (flat alongside other drivers) — trivial cost
- Slight overhead from method call vs bare function — negligible at 50 Hz

## Files Affected

| File | Action |
|------|--------|
| `pid.py` | Create — PID class with compute/reset |
| `main.py` | Edit — remove PID globals/functions, import and use PID class |
