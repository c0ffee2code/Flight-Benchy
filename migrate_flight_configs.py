"""
Migrate test_runs/flights/*/config.json from old layout to new layout.

Old layout:
  { vehicle, bench: { ..., session: {...} }, telemetry: {...} }

New layout:
  { session: {...}, vehicle, bench: { ..., telemetry: {...} } }

Idempotent: already-migrated configs are skipped.
"""

import json
import sys
from pathlib import Path

FLIGHTS_DIR = Path("test_runs/flights")


def needs_migration(cfg):
    return "telemetry" in cfg and "session" in cfg.get("bench", {})


def migrate(cfg):
    bench = dict(cfg["bench"])
    session = bench.pop("session")
    telemetry = cfg.pop("telemetry")
    bench["telemetry"] = telemetry
    return {"session": session, **{k: v for k, v in cfg.items() if k != "bench"}, "bench": bench}


def main():
    if not FLIGHTS_DIR.exists():
        print(f"ERROR: {FLIGHTS_DIR} does not exist")
        sys.exit(1)

    paths = sorted(FLIGHTS_DIR.glob("*/config.json"))
    if not paths:
        print("No config.json files found.")
        return

    migrated = 0
    skipped = 0
    for path in paths:
        try:
            with open(path, encoding="utf-8") as f:
                cfg = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            print(f"SKIP  {path.parent.name}  (unreadable: {e})")
            skipped += 1
            continue

        if not needs_migration(cfg):
            print(f"OK    {path.parent.name}")
            skipped += 1
            continue

        new_cfg = migrate(cfg)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(new_cfg, f, indent=2)
            f.write("\n")
        print(f"MIGR  {path.parent.name}")
        migrated += 1

    print(f"\n{migrated} migrated, {skipped} already up-to-date or skipped.")


if __name__ == "__main__":
    main()
