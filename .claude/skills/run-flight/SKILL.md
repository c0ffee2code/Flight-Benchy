---
name: run-flight
description: Full autonomous flight pipeline — reset position, deploy config, run flight.py, fetch the new run from SD card, and analyse it. Use whenever the user wants to run a fully automated flight. Triggers on "run flight", "autonomous flight", "start flight", "fly".
---

# Run Flight

Executes the full autonomous flight pipeline end-to-end:

1. Reset position (`/reset-position`)
2. Deploy config (`/deploy`)
3. Run `flight.py` on the Pico — waits for it to complete naturally (duration governed by `session.duration_s` in config)
4. Fetch the new run from SD card (`/fetch-flights`)
5. Analyse the new run (`/analyse-flight`)

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

Run the `/reset-position` skill. Wait for "Done — M1 end at restrictor." before continuing.

## Step 2 — Deploy config

Run the `/deploy` skill (no `--full` flag — config-only is sufficient unless the user explicitly asks for a full deploy).

## Step 3 — Run flight

```
python -m mpremote connect COM7 run src/flight.py
```

This blocks until `flight.py` exits — i.e. until `session.duration_s` elapses. Set the mpremote timeout to at least `duration_s + 30` seconds to avoid a premature timeout.

If mpremote exits with a non-zero code or prints a traceback, report the error verbatim and stop. Do not proceed to fetch or analyse — the run may be incomplete or absent from the SD card.

## Step 4 — Fetch

Run the `/fetch-flights` skill. Identifies the new run (the one not yet in `test_runs/flights/`). If no new run appears, report it — the flight may have crashed before opening the SD session.

## Step 5 — Analyse

Run the `/analyse-flight <new_run_id>` skill on the fetched run. The run ID is the timestamp folder name returned by the fetch step.

## Anomaly detection

After analysis, check for early termination: compare the run's actual duration (`last T_MS / 1000` seconds, visible in the profile output) against `session.duration_s` from `config.json`. If actual duration is less than 90% of configured duration, flag it prominently — likely a crash, battery cutoff, or SD write failure mid-run.