# Analyse Flight

Arguments: `<run_id>` — the run folder name under `test_runs/flights/`
(e.g. `/analyse-flight 2026-05-14_14-26-21`).

## Steps

1. Run:
   ```
   python pipelines/flight-analyser/run.py <run_id>
   ```
   Parse stdout as JSON. On `failed`: report `stage` and `error_summary`. Stop.

2. On `completed`: read `gate.json`, `verdict.json`, and `diagnose.json` from
   `test_runs/flights/<run_id>/analysis/` and present your interpretation to the user.
