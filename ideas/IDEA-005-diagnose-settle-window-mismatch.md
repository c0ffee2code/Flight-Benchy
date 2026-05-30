# IDEA-005: Fix diagnose.py hold_tracking window to match verdict hold_mae window

## Problem

`diagnose.py` computes `hold_tracking` stats (bias, std, p95, max_ae) from the
**first-entry** window (`detect_reach_event` — `ReachEvent`), while `verdict.py`
computes `hold_mae_deg` from the **confirmed hold window** (`detect_hold_window` —
`HoldWindow`, last exit to end of run).

For runs with pre-hold turbulence this creates an apparent contradiction: a run can
show `hold_tracking.p95 = 11.25 deg` (outside the tolerance band) in diagnose.json
while verdict.json reports `hold_mae_deg = 3.18 deg` (clean hold, PASS). Both numbers
are correct but they measure different windows, which confuses anyone reading the output.

## Fix

Compute `diagnose.py`'s `hold_tracking` group from the `HoldWindow` (confirmed hold)
instead of the `ReachEvent` (first entry). If pre-hold approach stats remain useful,
keep them as a separate `approach_tracking` group computed from `ReachEvent.start_idx`
to `HoldWindow.start_idx`.

## Acceptance

After the fix, for any run where verdict reports PASS:
- `diagnose.hold_tracking.p95` <= `tolerance_deg` (no alarming outliers on passing runs)
- `diagnose.hold_tracking.bias` close to `verdict.hold_mae_deg` (same window, same data)

## Priority

Backlog — address after the 2026-05-18 tolerance-band-9deg tuning session closes.