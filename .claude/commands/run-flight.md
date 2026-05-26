# Run Flight

Runs a complete flight: validates config, resets position, deploys, executes, pulls
telemetry, and analyses the result.

## Steps

1. Run:
   ```
   python pipelines/flight-runner/run.py
   ```
   Parse stdout as JSON. On `failed`: report `stage`, `rig_state`, and `error_summary`. Stop.

2. On `completed`: run:
   ```
   python pipelines/flight-analyser/run.py <run_id>
   ```
   Parse stdout as JSON. On `failed`: report `stage` and `error_summary`. Stop.

3. On `completed`: read `gate.json`, `verdict.json`, and `diagnose.json` from
   `test_runs/flights/<run_id>/analysis/` and present your interpretation to the user.
