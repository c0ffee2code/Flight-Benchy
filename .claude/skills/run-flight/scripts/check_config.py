import json, sys

with open("src/config.json") as f:
    config = json.load(f)

duration = config["bench"]["session"]["duration_s"]
if duration is None:
    print("ERROR: session.duration_s is null — set a duration in src/config.json before running a flight.")
    sys.exit(1)

setpoint = config["bench"]["session"]["setpoint"]
print(f"OK  duration_s={duration}s  setpoint={setpoint}")