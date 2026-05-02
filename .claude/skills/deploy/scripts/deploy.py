"""
deploy.py — upload config.json (and optionally all source files) to the Pico

Run from project root:
  python .claude/skills/deploy/scripts/deploy.py [--full]

  (no flag)  Upload config.json only — use for PID / config tuning between runs
  --full     Upload config.json + all source files — use after code changes

Pico must be connected on COM7. mpremote interrupts any running script on connect.
"""

import subprocess
import sys
from pathlib import Path

PYTHON   = sys.executable
COM_PORT = "COM7"
ROOT     = Path(__file__).resolve().parents[4]

SOURCE_FILES = [
    ("src/main.py",                          "main.py"),
    ("src/flight.py",                        "flight.py"),
    ("src/pid.py",                           "pid.py"),
    ("src/mixer.py",                         "mixer.py"),
    ("src/ui.py",                            "ui.py"),
    ("src/crash_log.py",                     "crash_log.py"),
    ("src/telemetry/recorder.py",            "recorder.py"),
    ("src/telemetry/sdcard.py",              "sdcard.py"),
    ("AS5600/driver/as5600.py",              "as5600.py"),
    ("BNO085/driver/bno08x.py",              "bno08x.py"),
    ("BNO085/driver/i2c.py",                 "i2c.py"),
    ("DShot/driver/dshot_pio.py",            "dshot_pio.py"),
    ("DShot/driver/motor_throttle_group.py", "motor_throttle_group.py"),
]

CONFIG_FILE = ("src/config.json", "config.json")


def _upload(local_rel, remote_name):
    local = ROOT / local_rel
    if not local.exists():
        print(f"  MISSING  {local_rel}")
        return False
    result = subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "cp", str(local), f":{remote_name}"],
        capture_output=True, text=True, timeout=30,
    )
    if result.returncode != 0:
        print(f"  FAIL     {local_rel}: {result.stderr.strip()}")
        return False
    print(f"  OK       {local_rel} -> :{remote_name}")
    return True


def main():
    full = "--full" in sys.argv
    files = ([CONFIG_FILE] + SOURCE_FILES) if full else [CONFIG_FILE]
    mode = "full deploy" if full else "config only"

    print(f"Deploying to Pico on {COM_PORT} ({mode})...")
    ok = sum(_upload(loc, rem) for loc, rem in files)
    failed = len(files) - ok

    print(f"\nDone: {ok} uploaded, {failed} failed.")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
