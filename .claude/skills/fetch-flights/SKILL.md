# Fetch Flights

Moves new flight runs from the Pico's SD card (`/sd/flights/`) to
`test_runs/flights/` on the local machine, then deletes the originals from
the card. Decoupled from analysis — use `/analyse-flight` on any fetched run.

## When to use

Trigger phrases: "fetch flights", "pull runs from Pico", "get flights off the
SD card", "sync flights". Does NOT analyse runs — that is `/analyse-flight`.

## Prerequisites

- Pico connected on COM7
- `mpremote` installed on the PC: `pip install mpremote`
- Pico should not be mid-session. `mpremote` sends Ctrl+C before running,
  which aborts any live stabilisation run.

## Usage

```
python .claude/skills/fetch-flights/scripts/fetch_flights.py [--yes]
```

`--yes` skips the confirmation prompt and moves all new runs immediately.

Run from the project root.

## Pipeline

1. Mounts SD, lists `/sd/flights/`
2. Diffs against local `test_runs/flights/` — shows only new runs
3. Asks "Fetch all? [Y/n]" (skipped with `--yes`)
4. Transfers each new run: `config.yaml` + `log.csv`, base64 over serial
5. Verifies byte-exact match for every file
6. Deletes only the runs that verified successfully — failures stay on SD
7. Prints a summary

## After the script completes

Report what was fetched. If any runs failed verification, name them — they
remain on the SD card and the user can retry. Suggest
`/analyse-flight <id>` for any run the user wants to examine.