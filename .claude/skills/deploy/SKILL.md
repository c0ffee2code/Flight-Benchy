# Deploy

Uploads `src/config.json` (and optionally all source files) to the Pico via `mpremote`.

## When to use

Trigger phrases: "deploy config", "upload config", "push config to Pico", "deploy to Pico",
"upload to Pico", "reflash Pico", "upload source".

## Prerequisites

- Pico connected on COM7
- `mpremote` installed: `pip install mpremote`
- Any running stabilisation session will be interrupted (mpremote sends Ctrl+C on connect)

## Usage

```
python .claude/skills/deploy/scripts/deploy.py [--full]
```

No flag: uploads `src/config.json` only — use this when tuning PID gains or any other
config parameter between runs.

`--full`: uploads `src/config.json` + all source files — use after code changes.

Run from the project root.

## Deployment layout

All files land flat at the Pico root:

| Local | Pico |
|-------|------|
| `src/config.json` | `config.json` |
| `src/main.py` | `main.py` |
| `src/pid.py` | `pid.py` |
| `src/mixer.py` | `mixer.py` |
| `src/ui.py` | `ui.py` |
| `src/telemetry/recorder.py` | `recorder.py` |
| `src/telemetry/time_source.py` | `time_source.py` |
| `src/telemetry/sdcard.py` | `sdcard.py` |
| `AS5600/driver/as5600.py` | `as5600.py` |
| `BNO085/driver/bno08x.py` | `bno08x.py` |
| `BNO085/driver/i2c.py` | `i2c.py` |
| `DShot/driver/dshot_pio.py` | `dshot_pio.py` |
| `DShot/driver/motor_throttle_group.py` | `motor_throttle_group.py` |

## After the script completes

Report what was uploaded. If any files failed, name them.