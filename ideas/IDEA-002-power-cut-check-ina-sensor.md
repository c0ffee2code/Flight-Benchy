# IDEA-002: Rewrite power-cut gate check using INA sensor

## Background

The current `power-cut` check in `pipelines/flight-analyser/scripts/gate.py` uses a
heuristic: flat encoder + active motor differential + setpoint never reached = power cut.

This produces false positives when a working controller fails to reach a non-zero setpoint
(e.g. 15 deg) but holds the lever at an equilibrium below the gravity resting position.
A temporary fix was applied (2026-05-26): skip the check when `mean_enc < gravity_resting - 5 deg`,
indicating the controller has mechanical authority.

## Proposed improvement

An INA sensor (current/voltage monitor) is now available. Wire it to monitor ESC power
delivery. At the end of each flight, log or check:

- Supply voltage drop during run
- ESC current draw vs commanded throttle

Gate check rewrite: replace the encoder/motor heuristic with a direct power-delivery
check -- if ESC voltage collapses or current drops to zero while Pico is commanding
throttle, flag as power cut. This is unambiguous and setpoint-agnostic.

## Acceptance

- Remove `_POWER_CUT_GRAVITY_MARGIN_DEG` workaround from gate.py
- INA telemetry column(s) added to log.csv
- gate.py `check_power_cut` reads voltage/current, not encoder std