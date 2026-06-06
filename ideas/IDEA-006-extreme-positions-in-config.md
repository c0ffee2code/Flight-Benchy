# IDEA-006: Add lever extreme positions to config.json bench section [PARTIALLY RESOLVED]

## Observation

Physical lever travel limits (measured 2026-05-30, current CF frame + PETG subframe):

- M1 end lowest (natural mechanical stop, no restrictor): **-51.7 deg**
- M2 end lowest (natural mechanical stop):                **+51.2 deg**

## Idea

Add these as named fields under `bench` in config.json, e.g.:

```json
"bench": {
  "encoder": { ... },
  "limits": {
    "m1_lowest_deg": -51.7,
    "m2_lowest_deg":  51.2
  },
  ...
}
```

## Motivation

- **Gate check**: ~~`gate.py` uses a hardcoded `_STANDARD_START_DEG = 51.0`~~
  **RESOLVED (2026-06-06)**: `bench.start_angle_deg` added to config.json; `gate.py`
  reads it from there. All historical run configs backfilled with 51.0.
- **Reset position validation**: `reset_position.py` could confirm M1 reached
  its expected limit rather than an arbitrary angle.
- **Constraint-aware control**: knowing the physical limits allows the analyser
  and any future planner to reason about how much travel margin exists from a
  given start position.

## Notes

- Values must be re-measured after any mechanical change (frame rebuild,
  encoder repositioning, subframe swap).
- The restrictor reduces effective M1 travel from -51.7 deg to approximately
  +48-49 deg (restrictor-limited start). If the restrictor is reinstalled,
  a separate `m1_restricted_deg` field would be cleaner than overwriting the
  natural limit.