# IDEA-006: Add lever extreme positions to config.json bench section

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

- **Gate check**: `gate.py` uses a hardcoded `_STANDARD_START_DEG = 51.0`
  with a ±10 deg window. Without the restrictor, M1 rests at ~48-49 deg, which
  sits right at the gate floor and causes marginal failures. Reading the actual
  limit from config would let the gate adapt to the current rig geometry.
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