# IDEA-001 — Per-flight acceptance criteria

**Status:** Idea — not yet designed or implemented

---

## Motivation

The analyse-flight skill currently judges flights against hardcoded tolerance bands (±10°).
As the algorithm matures, "good enough" will mean something stricter — but right now there is
no single place that says what success actually means for a given run.

There is also no acceptance gate on rise time or hold duration. A flight can be scored
without anyone having written down whether 20 seconds of hold is acceptable or whether
a 30-second rise is a failure.

---

## Idea

Introduce a small, human-readable `criteria.json` that lives next to `config.json` and is
copied into each run folder by `SdSink` — exactly like `config.json` is today.

This file defines **what success means at this point in the algorithm's development**.
It is separate from `config.json` because the two things evolve independently: gains change
on every tuning session, but the acceptance bar changes only when the team consciously raises it.

The analyse-flight skill reads `criteria.json` from the run folder and uses it to:
- draw the tolerance band on plots (replacing the hardcoded ±10°)
- score each KPI pass / good / excellent
- include the verdict in the analysis report

All scripts in the analyse-flight pipeline that currently use hardcoded constants should
pull from `criteria.json` instead:

| Script | Current hardcoding | Driven by criteria |
|--------|-------------------|-------------------|
| `plots.py` | ±10° grey band | `tolerance_deg` |
| `score_flight.py` | ±10° reach threshold | `tolerance_deg`, `max_rise_s`, `min_hold_s` |
| `profile_flight.py` | ±10° hold-phase boundary | `tolerance_deg` |

The tolerance band rendered on every plot becomes a visual contract — when the band
narrows from ±10° to ±5° to ±3°, every historical plot generated under the new criteria
makes the raised bar immediately visible.

---

## Key dimensions to define (not yet decided)

These are the axes we expect to judge flights on. Exact thresholds, labels, and whether
we need more dimensions need to be worked out before implementation.

| KPI | What it measures | Direction |
|-----|-----------------|-----------|
| Hold MAE | Mean absolute encoder error during hold phase | lower is better |
| Rise time (T→SP) | Seconds to first entry into tolerance band | lower is better |
| Hold time (T@SP) | Total seconds within tolerance band | higher is better |
| Overshoot | Peak excursion beyond setpoint, % of initial error | lower is better |

A natural threshold structure might be three levels — **pass / good / excellent** — so
the report can tell us not just "did it pass" but "how well did it pass".

---

## Versioning concept

The idea also supports rating historical algo versions against the same KPI set, giving a
scoreboard as the algorithm improves over time. Example sketch:

```json
{
  "kpis": [
    { "id": "hold_mae_deg", "thresholds": { "pass": 10, "good": 5, "excellent": 3 } },
    { "id": "rise_time_s",  "thresholds": { "pass": null, "good": 10, "excellent": 5 } },
    { "id": "hold_time_s",  "thresholds": { "pass": null, "good": 60, "excellent": 100 } }
  ],
  "algo_versions": [
    {
      "label": "post-rebuild-baseline",
      "date": "2026-04-07",
      "representative_run": "2026-04-07_16-19-21",
      "observed": { "hold_mae_deg": 6.90, "rise_time_s": 1.3, "hold_time_s": 124.8 }
    },
    {
      "label": "oscillation-fix",
      "date": "2026-05-14",
      "observed": { "hold_mae_deg": { "min": 1.9, "max": 3.3 }, "rise_time_s": { "min": 17, "max": 26 } }
    }
  ]
}
```

---

## Open questions

- Which KPIs are mandatory vs informational?
- What are the right threshold values at the current level of maturity?
- Does `criteria.json` live at project root only, or should there be a way to override per-session?
- How does the analyse-flight skill degrade gracefully for old runs that have no `criteria.json`?
