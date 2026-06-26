"""
deploy.py — upload config.json + specification.json (and optionally all source files) to the Pico

Run from project root:
  python .claude/commands/deploy.py [--full]

  (no flag)  Upload config.json + specification.json — use for PID / config tuning between runs
  --full     Upload config.json + specification.json + all source files — use after code changes

Pico must be connected on COM7. mpremote interrupts any running script on connect.
"""

import subprocess
import sys
from pathlib import Path

PYTHON   = sys.executable
COM_PORT = "COM7"
ROOT     = Path(__file__).resolve().parents[3]

SOURCE_FILES = [
    ("src/main.py",                          "main.py"),
    ("src/flight.py",                        "flight.py"),
    ("src/core/__init__.py",                 "core/__init__.py"),
    ("src/core/control.py",                  "core/control.py"),
    ("src/core/pid.py",                      "core/pid.py"),
    ("src/core/mixer.py",                    "core/mixer.py"),
    ("src/ui.py",                            "ui.py"),
    ("src/telemetry/recorder.py",            "recorder.py"),
    ("dependencies/PCF8523/src/pcf8523.py",               "pcf8523.py"),
    ("src/telemetry/sdcard.py",                           "sdcard.py"),
    ("dependencies/AS5600/driver/as5600.py",              "as5600.py"),
    ("dependencies/BNO085/src/bno08x.py",                 "bno08x.py"),
    ("dependencies/BNO085/src/i2c.py",                    "i2c.py"),
    ("dependencies/DShot/driver/dshot_pio.py",            "dshot_pio.py"),
    ("dependencies/DShot/driver/motor_throttle_group.py", "motor_throttle_group.py"),
]

CONFIG_FILES = [
    ("src/config.json",        "config.json"),
    ("src/specification.json", "specification.json"),
]


def _ensure_core_dir():
    """Create :core/ on the Pico if it does not exist (ignore error if it does)."""
    subprocess.run(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "fs", "mkdir", ":core"],
        capture_output=True, timeout=10,
    )


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
    files = (CONFIG_FILES + SOURCE_FILES) if full else CONFIG_FILES
    mode = "full deploy" if full else "config only"

    print(f"Deploying to Pico on {COM_PORT} ({mode})...")
    if full:
        _ensure_core_dir()
    ok = sum(_upload(loc, rem) for loc, rem in files)
    failed = len(files) - ok

    print(f"\nDone: {ok} uploaded, {failed} failed.")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
