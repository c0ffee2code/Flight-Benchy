---
name: run-flight
description: Flight pipeline — reset position, deploy config, run flight, pull  the new run from SD card. Returns the new run ID. Does NOT analyse. Use whenever the user wants to execute a flight. Triggers on "run flight".
---

# Run Flight

Executes the flight pipeline end-to-end and returns the new run ID:

1. Reset position
2. Deploy config
3. Run `flight.py` on the Pico — waits for it to complete naturally (duration governed by `session.duration_s` in config)
4. Pull the new run from SD card

## Pre-conditions

- Pico connected on COM7
- Both motors and ESCs are powered and connected
- Bench area is clear — the lever will swing during reset and flight

> **Always use this skill for flight runs.** Never execute the steps manually (deploy → flight.py → fetch). Manual execution silently bypasses the reset-position step, which produces a non-standard start angle and a discarded run.

## Step 0 — Check config

```
python .claude/skills/run-flight/scripts/check_config.py
```

If the script exits non-zero, stop and show its output. Do not proceed.

## Step 1 — Reset position

```
python -m mpremote connect COM7 run .claude/commands/reset_position.py
```

Wait for "Done -- M1 end at restrictor." before continuing.

## Step 2 — Deploy config

```
python .claude/commands/deploy.py
```

Config-only (no `--full`) is sufficient unless the user explicitly asks for a full deploy.

## Step 3 — Run flight

```
python -m mpremote connect COM7 run src/flight.py
```

This blocks until `flight.py` exits — i.e. until `session.duration_s` elapses. Set the mpremote timeout to at least `duration_s + 30` seconds to avoid a premature timeout.

If mpremote exits with a non-zero code or prints a traceback, report the error verbatim and stop. Do not proceed to fetch or analyse — the run may be incomplete or absent from the SD card.

## Step 4 — Pull

```
python .claude/commands/pull_flights.py
```

Identifies the new run (the one not yet in `test_runs/flights/`). If no new run appears, report it — the flight may have crashed before opening the SD session.

## Output

Report the new run ID (the timestamp folder name, e.g. `2026-05-20_14-30-00`) as the result of this skill. The caller decides whether to invoke `/analyse-flight`.
