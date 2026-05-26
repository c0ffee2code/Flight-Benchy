import json
import subprocess
import sys
from pathlib import Path

SCRIPTS = Path(__file__).parent / "scripts"
FLIGHTS_DIR = Path("test_runs/flights")
COM_PORT = "COM7"


def _error_summary(r):
    combined = (r.stdout + "\n" + r.stderr).strip()
    lines = [ln for ln in combined.splitlines() if ln.strip()]
    return lines[-1] if lines else f"exit code {r.returncode}"


def _fail(stage, run_id=None, error_summary="", rig_state="ok"):
    print(json.dumps({
        "status": "failed",
        "stage": stage,
        "run_id": run_id,
        "error_summary": error_summary,
        "rig_state": rig_state,
    }))
    sys.exit(1)


def _snapshot():
    if not FLIGHTS_DIR.exists():
        return set()
    return {p.name for p in FLIGHTS_DIR.iterdir() if p.is_dir()}


def _require(config, *keys):
    val = config
    path = ".".join(keys)
    for key in keys:
        if not isinstance(val, dict) or key not in val:
            _fail("check_config", error_summary=f"{path} is missing from src/config.json")
        val = val[key]
    if val is None:
        _fail("check_config", error_summary=f"{path} is null in src/config.json")
    return val


def main():
    with open("src/config.json", encoding="utf-8") as f:
        config = json.load(f)
    duration_s = _require(config, "bench", "session", "duration_s")
    flight_timeout = int(duration_s) + 30

    # Step 1: check config
    r = subprocess.run(
        ["python", str(SCRIPTS / "check_config.py")],
        capture_output=True, text=True,
    )
    sys.stderr.write(r.stdout); sys.stderr.write(r.stderr)
    if r.returncode != 0:
        _fail("check_config", error_summary=_error_summary(r))

    # Step 2: reset position
    r = subprocess.run(
        ["python", "-m", "mpremote", "connect", COM_PORT, "run",
         str(SCRIPTS / "reset_position.py")],
        capture_output=True, text=True, timeout=60,
    )
    sys.stderr.write(r.stdout); sys.stderr.write(r.stderr)
    if r.returncode != 0 or "Done" not in r.stdout:
        _fail("reset",
              error_summary=_error_summary(r) or "reset did not confirm Done",
              rig_state="needs_human")

    # Step 3: deploy config
    r = subprocess.run(
        ["python", str(SCRIPTS / "deploy.py")],
        capture_output=True, text=True,
    )
    sys.stderr.write(r.stdout); sys.stderr.write(r.stderr)
    if r.returncode != 0:
        _fail("deploy", error_summary=_error_summary(r))

    # Snapshot before flight
    before = _snapshot()

    # Step 4: run flight
    try:
        r = subprocess.run(
            ["python", "-m", "mpremote", "connect", COM_PORT, "run", "src/flight.py"],
            capture_output=True, text=True, timeout=flight_timeout,
        )
    except subprocess.TimeoutExpired:
        _fail("run", error_summary=f"flight.py timed out after {flight_timeout}s")
    sys.stderr.write(r.stdout); sys.stderr.write(r.stderr)
    if r.returncode != 0 or "Traceback" in r.stdout or "Traceback" in r.stderr:
        _fail("run", error_summary=_error_summary(r) or "flight.py failed")

    # Step 5: pull telemetry
    r = subprocess.run(
        ["python", str(SCRIPTS / "pull_flights.py")],
        capture_output=True, text=True,
    )
    sys.stderr.write(r.stdout); sys.stderr.write(r.stderr)
    if r.returncode != 0:
        _fail("pull", error_summary=_error_summary(r))

    # Determine run_id from new folder
    after = _snapshot()
    new_folders = sorted(after - before)
    if not new_folders:
        _fail("pull", error_summary="no new run folder appeared after pull")

    print(json.dumps({"status": "completed", "run_id": new_folders[-1]}))
    sys.exit(0)


if __name__ == "__main__":
    main()
