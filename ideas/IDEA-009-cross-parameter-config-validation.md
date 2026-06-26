# IDEA-009: Cross-parameter config and specification validation

## Problem

Parameters in `config.json` and `specification.json` are not independent, but they are
validated independently or not at all. Violations are discovered at runtime (Pico crash) or
during post-flight analysis (KPI structurally unachievable). Either way the run is wasted —
deploy cycle, bench setup, and the full run duration — none of which produce useful data.

## Known dependency classes

### Within config.json

**Motor range:** `base_throttle +/- rate_output_limit` must fit within
`[throttle_min, throttle_max]`. This is already partially enforced by
`assert_motor_range` in `src/core/control.py`, called at `ControlCore` init.
It is the only cross-parameter check currently in the system.

**Loop ratio:** `rate.frequency_hz` must be divisible by `angle.frequency_hz` — the outer
tick counter uses integer division (`rate_hz // angle_hz`). A non-divisible pair produces
a loop that never fires the outer tick at the correct cadence.

**Feedforward lead vs loop period:** `feedforward.lead_ms` should be close to one outer
loop period. A lead_ms that greatly exceeds the outer period extrapolates far past the
next GRV sample and amplifies rather than compensates sensor lag.

### Between config.json and specification.json

**Hold duration vs flight duration:** `specification.kpis.hold_duration_s.thresholds.pass`
requires a hold of N seconds. If `session.duration_s` is shorter than that threshold plus
a realistic time-to-reach, the KPI is structurally unachievable regardless of controller
performance. Example: `hold_duration_s.pass = 60` with `duration_s = 30`.

**Settling time vs flight duration:** `specification.kpis.settling_time_s.thresholds.pass`
sets a T_s ceiling. If `session.duration_s` is shorter than T_s + minimum hold window,
a PASS verdict is impossible even for a perfect run.

**Tolerance band vs start angle:** If `specification.tolerance_deg` is tighter than the
expected IMU-ENC bias floor (currently ~1.4 deg), no run can reliably pass regardless of
gains. This is softer — a warning rather than a hard block — but worth surfacing.

## Proposed approach

A standalone `validate_config.py` script (or a `--validate` flag on the pipeline) that
checks all cross-parameter contracts before a run or before analysis. Returns a list of
violations with clear messages. Two severity levels:

- **ERROR:** contract violation that guarantees a wrong result (motor range overflow,
  non-divisible loop ratio, KPI structurally impossible given duration_s).
- **WARN:** constraint that makes a KPI unlikely but not impossible (feedforward lead
  large relative to outer period, tolerance band near the IMU-ENC floor).

The deploy pipeline (`pipelines/flight-runner/scripts/deploy.py`) is a natural integration
point: validate before uploading to the Pico so config errors are caught on the PC.

The analysis pipeline (`pipelines/flight-analyser/run.py`) could run the same validator
on the run folder's config before producing a verdict, so structurally-impossible KPI
failures are flagged as config issues, not controller failures.

## Relation to existing work

`assert_motor_range` in `control.py` is the seed of this idea — it validates one
cross-parameter contract at Pico boot time. The proposal is to move that class of
validation to the PC side (where errors can be reported clearly) and extend it to cover
the config-spec boundary.
