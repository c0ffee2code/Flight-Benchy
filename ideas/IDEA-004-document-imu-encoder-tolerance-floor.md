# IDEA-004: Document the IMU-vs-encoder bias as an architectural tolerance floor

## Status: backlog

## What to document

During the 2026-05-18 tolerance-band-9deg tuning session, a fundamental architectural
constraint was uncovered. It should be written up clearly -- as a decision record or
a standalone analysis note -- so that future tightening sessions start from a correct
mental model rather than rediscovering it through failed runs.

## The constraint in plain terms

The control loop steers **IMU (GRV) -> setpoint**.
The acceptance specification scores **encoder** position.
These are two different coordinate frames with a measurable, persistent disagreement.

When the controller is perfectly satisfied (IMU error = 0), the encoder does not read
zero -- it reads approximately +1.4 deg. This is the IMU-ENC frame offset. It has two
sources:

1. **Tare registration error**: the GRV zero is set during tare, when the lever is placed
   on the jig and the IMU is asked to zero itself. Any residual misalignment between the
   jig-defined horizontal and the encoder-defined AXIS_CENTER contributes a fixed offset.
   Multiple careful tares have reduced this to ~1.4 deg; it appears near the achievable
   floor with the current procedure and hardware placement.

2. **GRV filter bias**: the BNO085 game rotation vector output may carry a small, stable
   DC bias relative to the true angle, independent of tare. This is uncharacterised --
   see IDEA-003 for the fixed-frame measurement that would separate filter bias from tare
   registration error.

On top of the frame offset, mechanical equilibrium effects (wire tension, bearing friction)
create an additional ~1.6 deg hold bias, for a total structural hold bias of ~3 deg at
the current config. The PID I-term addresses the mechanical component; the frame offset
is irreducible without tare improvement or hardware repositioning.

## The floor formula

Minimum achievable hold band (half-width) such that hold_duration and settling_time KPIs
can reliably pass:

    tolerance_deg_floor >= hold_bias_mean + N_sigma * hold_std

Where:
- hold_bias_mean = IMU-ENC frame offset + mechanical equilibrium offset (~3 deg currently)
- hold_std = run-to-run oscillation amplitude (~3 deg, gain-insensitive per spectrum analysis)
- N_sigma = reliability target (N=2 -> ~2.3% per-sample exceedance, coin-flip over 120s run;
                                 N=3 -> ~0.1% per sample, reliable holds)

At current values: floor(N=2) = 3 + 2*3 = 9 deg. This is why +-9 deg is marginal and
+-7 deg is structurally unreachable without first reducing hold_bias_mean.

## Why encoder-in-the-loop is not the answer

The end goal of this project is drone flight. In the air, there is no absolute position
encoder. The control loop must work on IMU only. Adding encoder feedback to close the
loop would produce a tuned system that cannot transfer to the actual flight hardware.
This constraint is permanent and architectural -- it is not a laziness or oversight.

The encoder serves as a ground-truth reference for diagnosing the IMU-controlled system,
not as a feedback signal.

## Implications for future tightening sessions

Before opening a new tolerance-band tightening session:

1. Check current hold_bias_mean from recent runs (diagnose.json -> hold_tracking.bias).
2. Check hold_std from recent runs (diagnose.json -> hold_tracking.std).
3. Compute floor(N=2) = hold_bias_mean + 2 * hold_std.
4. If proposed tolerance_deg <= floor(N=2), the tightening is structurally risky -- runs
   will pass/fail by luck, not by config quality. Prerequisite: reduce hold_bias_mean first
   (better tare, ki tuning, or IMU-ENC alignment improvements from IDEA-003).
5. If proposed tolerance_deg <= IMU-ENC frame offset alone (currently ~1.4 deg), the
   tightening is impossible regardless of PID tuning. Do not open the session.

## What this document should contain

- Diagram or table: encoder frame vs IMU frame, showing the offset and its sources
- The floor formula above, with current measured values filled in
- Sensitivity analysis: how much does hold_bias_mean need to drop to make +-7 deg viable?
- Path to reducing the floor:
  (a) Reduce frame offset: IDEA-003 measurement, BNO085 settings investigation
  (b) Reduce mechanical offset: ki tuning (in progress), or reduced wire tension
  (c) Reduce hold_std: currently ~3 deg and gain-insensitive; spectrum shows 1/f^2 --
      root cause unknown, possibly GRV filter time constant or mechanical resonance
- Reference to DR-005 (BNO085 as primary PID input) as the architectural decision that
  creates this constraint, and the reasoning that makes encoder-in-loop permanently
  off the table

## Suggested output format

A decision record (DR-NNN) or a standalone analysis note under `decision/` or `resources/`.
Not a code change. Should be readable by someone who understands the project but wasn't
in the 2026-05-29 tuning session.