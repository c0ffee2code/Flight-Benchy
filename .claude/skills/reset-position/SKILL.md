---
name: reset-position
description: Reset Flight Benchy to the standard initial position — M1 end down at the restrictor. Runs an open-loop M1 thrust pulse via mpremote. Use before each test session or whenever the lever needs to be returned to start. Triggers on "reset position", "position reset", "put benchy in start position".
---

# Reset Position

Drives Flight Benchy to the standard initial position: M1 end resting on the restrictor (~+58° encoder). Open-loop — applies a fixed M1 throttle for a fixed duration, then disarms. No encoder feedback.

## Pre-conditions

Before running, check:
- Pico is connected via USB
- Both motors and ESCs are powered and connected
- The bench area is clear — the lever will swing

If the Pico is not connected, tell the user and stop. Do not attempt to run without a confirmed connection.

## Execution

```
mpremote run .claude/skills/reset-position/scripts/reset_position.py
```

The script takes approximately 5–6 seconds:
- ~2s: ESC arming sequence (beeps)
- 3s: M1 thrust pulse
- ~0.5s: cooldown before disarm

Wait for the process to complete. The script prints two lines:
1. `Armed. Applying M1 reset thrust ...`
2. `Done — M1 end at restrictor.`

## Outcome

Tell the user the reset is complete and M1 should be at the restrictor (~+58°). Remind them to verify visually before starting the next run — especially if the run result later shows a non-standard start angle.

If mpremote reports an error (import error, device not found, etc.), report it verbatim and stop. Do not retry.