# IDEA-003: Fixed-frame IMU noise measurement and BNO085 fusion tuning

## Background

During the 2026-05-18 tolerance-band-9deg tuning session, analysis of hold-error spectra
(Iter 5 and Conf 1) revealed that hold error power is concentrated entirely in the slow-drift
regime (< 0.3 Hz, 1/f^2 rolloff) with no resonance peak. The dominant source of hold bias
(~3 deg encoder offset at setpoint) decomposes into two components:

1. **IMU-ENC frame offset (~1.4 deg):** The controller targets IMU=0 (game rotation vector).
   When IMU=0, the encoder reads ~+1.4 deg due to residual tare error and/or GRV filter bias.
   Multiple careful tares have brought this to ~1.4 deg; it appears to be near the achievable
   floor with current tare procedure.

2. **Mechanical equilibrium offset (~1.6 deg):** Wire tension and bearing friction create a
   steady-state position error that the I-term (ki=0.05, iterm_limit=5) is too weak to
   fully correct within a 120s run.

## Architectural constraint

The rig uses IMU (GRV) as the primary PID feedback signal. Encoder is telemetry-only.
This is a deliberate architectural choice (DR-005): the end goal is drone flight where no
encoder exists. Therefore **encoder-in-the-loop is permanently off the table**.

Consequence: the IMU-ENC frame offset defines a hard floor on how tight any encoder-based
acceptance band can ever be. If the GRV output has a static bias of X deg relative to the
encoder-measured true angle, then a tolerance band tighter than X deg cannot be reliably
achieved regardless of PID tuning. The current ~1.4 deg offset means the +-1 deg band
class is structurally unreachable; +-5 deg and tighter become increasingly marginal.

## Proposed measurement

Run a dedicated flight with the lever mechanically fixed at precisely horizontal (e.g.,
clamped with the precision jig). Motors off. Duration: 120s (same as standard run).

Log the full telemetry: GRV angle (IMU_QR..QK -> roll), encoder angle (ENC_ROLL, degrees).
Post-process:
- GRV angle noise PSD over 120s: quantify noise floor and any slow drift
- GRV vs encoder: static bias and drift over time
- GRV angle std (at zero true motion): this is the irreducible sensor-noise floor
- Dominant frequency components in GRV output at rest

## What this informs

1. **Tolerance band floor:** the minimum achievable hold band is approximately
   IMU_noise_std + IMU_ENC_static_bias. Provides a data-backed target for future
   specification.json tightening.

2. **BNO085 sensor fusion settings:** if GRV noise std at rest is significantly higher
   than expected (the datasheet quotes ~1 deg RMS for GRV), the fusion parameters may
   be tunable:
   - Report rate (currently 50 Hz for GRV): higher rate may average down noise
   - Sensor calibration quality: check accel/gyro/mag calibration status during the run
   - DCD (Dynamic Calibration) behaviour: whether re-enabling mag fusion changes bias

3. **IMU-ENC tare quality ceiling:** if the static GRV bias at true horizontal (measured
   by encoder) is consistently ~1.4 deg across multiple fixed-frame runs, that is the
   tare floor. If it varies run-to-run, improved tare procedure or external angle reference
   could reduce it.

## Acceptance

- Fixed-frame run script or procedure documented (motors off, frame clamped)
- GRV noise PSD plot produced (same pipeline as 03_spectrum.png, but for GRV angle)
- Static IMU-ENC bias distribution across N >= 3 fixed-frame runs quantified
- Result feeds directly into: minimum achievable tolerance_deg for future sessions,
  and a go/no-go decision on BNO085 settings investigation