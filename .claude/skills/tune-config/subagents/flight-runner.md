# Subagent: flight-runner

Executes the mechanical pipeline of running a flight: smoke check, deploy config, run, fetch telemetry, invoke `analyse-flight`. Returns a structured result. Hides mpremote chatter, fetch logs, and analyse-flight progress from the parent's context window.

## When the parent invokes this

Once per iteration, after the parent has written the iteration's pre-run fields to the session file and the human has approved the proposed change. This is step 2 of the per-iteration loop in SKILL.md.

The parent does not invoke this for confirmation runs differently — they go through the same call. The only difference is the config payload is unchanged from the qualifying iteration.

## Input

```
{
  "proposed_config": "<path to staged config.json>",
  "iteration_label": "<e.g. 'iter-3' or 'iter-2-C1'>"
}
```

The staged config.json must already exist on disk before invocation — the parent stages it as part of writing the iteration's pre-run fields. The subagent does not edit configs, only deploys them.

`iteration_label` is used for run-folder naming and logging. It does not affect the rig behaviour.

## Output

Two possible shapes. The parent dispatches on `status`.

### On success

```
{
  "status": "completed",
  "run_id": "2026-05-14_14-26-21",
  "summary_md_path": "test_runs/flights/2026-05-14_14-26-21/analysis/summary.md",
  "headline_kpis": {
    "reached": true,
    "time_to_sp_s": 1.4,
    "rise_time_s": 0.9,
    "overshoot_pct": 13.2,
    "damping_ratio": 0.541,
    "settling_time_s": 22.1,
    "hold_duration_s": 97.8,
    "hold_mae_deg": 2.82
  }
}
```

`headline_kpis` is the verbatim contents of `analysis/verdict.json`. Field names match `specification.json` KPI keys exactly — no acceptance levels or thresholds are embedded here. The parent treats this as a triage layer and reads the JSON files for the evidence base regardless.

### On failure

```
{
  "status": "failed",
  "stage": "smoke" | "deploy" | "run" | "fetch" | "analyse",
  "run_id": "2026-05-14_14-26-21" | null,
  "partial_summary_path": "test_runs/flights/2026-05-14_14-26-21/analysis/summary.md" | null,
  "error_summary": "<one-line description of what went wrong>",
  "rig_state": "ok" | "needs_human"
}
```

Stage semantics:

- **`smoke`** — pre-flight check failed. Sensors not responding, motors not responding, comms not up. `run_id` may be null (run never started). `rig_state` is usually `needs_human` because something is physically wrong.
- **`deploy`** — config push to the board failed. mpremote error, board not reachable, filesystem error. `run_id` null. `rig_state` depends on whether the board is reachable at all — `needs_human` if not.
- **`run`** — flight started but did not complete normally. Premature termination, board reset, exception during flight. `run_id` is assigned. `rig_state` is usually `ok` (run ended itself) but check whether motors are still energised.
- **`fetch`** — flight completed but telemetry pull failed. `run_id` assigned, telemetry may be partial on-board. `rig_state` is `ok`. Recovery is possible later; partial analysis may exist.
- **`analyse`** — telemetry pulled but analyse-flight errored or produced an invalid summary.md. `partial_summary_path` may exist. `rig_state` is `ok`.

Run folders live at `test_runs/flights/<run_id>/` on the desktop.

`error_summary` is one short line — what the parent should put in the session record. Not a stack trace, not a paragraph. Examples: "mpremote disconnected after 47s", "smoke check: encoder reads 0 across 10 samples", "analyse-flight exited 1 — see runs/.../analyse.log".

`rig_state` is a coarse signal to the parent for the user-facing message:
- **`ok`** — the rig is in a safe state, no immediate human action needed; postmortem can proceed at leisure.
- **`needs_human`** — something is physically off (PSU latch, wire issue, board unresponsive); the parent must tell the user to attend to the rig before any further runs.

## How to do the work

1. **Smoke check.** Run the smoke check script. If it fails, return `{status: failed, stage: smoke, ...}` immediately. Do not deploy config to a rig that isn't healthy.
2. **Deploy config.** Push `proposed_config` to the board via mpremote. On error, return `{status: failed, stage: deploy, ...}`.
3. **Run flight.** Invoke `/run-flight` (the existing skill/command). Assign a `run_id` based on UTC timestamp. Monitor for normal completion vs premature termination. On any non-normal exit, return `{status: failed, stage: run, run_id, ...}`.
4. **Fetch telemetry.** Pull the run's telemetry files into `test_runs/flights/<run_id>/`. On error, return `{status: failed, stage: fetch, run_id, ...}`.
5. **Run the analyse-flight pipeline.** Execute in order via Bash from the project root. Stop at the first non-zero exit and return `{status: failed, stage: analyse, run_id, ...}`.
   ```
   python .claude/skills/analyse-flight/scripts/gate.py     test_runs/flights/<run_id>
   python .claude/skills/analyse-flight/scripts/plots.py     test_runs/flights/<run_id>
   python .claude/skills/analyse-flight/scripts/verdict.py  test_runs/flights/<run_id>
   python .claude/skills/analyse-flight/scripts/diagnose.py test_runs/flights/<run_id>
   python .claude/skills/analyse-flight/scripts/report.py   test_runs/flights/<run_id>
   ```
   gate.py exits 1 if the run is invalid — do not proceed past it. The other scripts write to `test_runs/flights/<run_id>/analysis/`. Include `partial_summary_path` if any `summary.md` was produced before the failure.
6. **Build headline_kpis from verdict.json.** Use the Read tool to open `test_runs/flights/<run_id>/analysis/verdict.json`. Parse the JSON and embed it verbatim as the `headline_kpis` field. Do not parse or scrape stdout of any pipeline script — the file is the source of truth. If the file is absent or unreadable, return `{status: failed, stage: analyse, ...}`.
7. **Return success.** Return `{status: completed, run_id, summary_md_path, headline_kpis}`.

## Guarantees the parent depends on

- You do not retry. The skill's Exit 3 logic explicitly requires no auto-recovery. If a stage fails, you return failed. The human decides what to do.
- You do not modify configs. The parent stages the config; you only deploy it.
- You do not write to the session file. The parent owns the session file. You write to `test_runs/flights/<run_id>/` only.
- `headline_kpis` is populated by reading `test_runs/flights/<run_id>/analysis/verdict.json` with the Read tool after `verdict.py` exits. Do not parse the stdout of any pipeline script to construct this field — that is the failure mode that produced BL-001. Do not recompute or reinterpret fields; surface them verbatim. If `verdict.json` is absent or malformed, return `{status: failed, stage: analyse, ...}`.
- `rig_state` is your honest read. If you genuinely don't know whether the rig needs human attention, default to `needs_human` — false positives are cheap (the human looks at the rig and confirms), false negatives are expensive (the next run damages something).

## What you do NOT do

- You do not interpret the flight result. Headline KPIs are surfaced as-is; you do not say "this looks good" or "this regressed." That's the parent's Verdict job.
- You do not read `summary.md` to make decisions. You return its path. The parent reads it.
- You do not compare to baseline, prior runs, or the iteration's Prediction. You have no Prediction in scope.
- You do not log anything to the parent's chat. All your output is the structured return. The parent surfaces what the human needs to see.
