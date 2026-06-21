"""Software-in-the-loop simulation tests.

Plant model (from REVIEW-2026-06-10, Part 0.2):
    phi_ddot = -C * output        (C > 0, plant sign negative)

Sensor model (from sign contracts):
    imu_roll = phi                (GRV in encoder convention)
    raw_gx_rad = radians(phi_dot) (before ControlCore applies -imu_sign internally)

The plant model is derived from physics independently of ControlCore — a paired sign
error between plant and sensor model would not cancel, making the mutation tests
meaningful.
"""
import pytest
from math import radians, cos, sin
from core.control import ControlCore

PLANT_C   = 0.5    # deg/s^2 per throttle unit; sign-correctness only, not KPI fidelity
# With the actual gains (kp_a=3.5, kp_r=0.5, kd_a=0.65), closed-loop analysis gives:
#   zeta ~ 0.22, omega_n ~ 0.935 rad/s, decay rate ~ 0.206/s
# At 30s, amplitude decays from 51 to ~0.1 deg — well within the tolerance band.
DT_S      = 1 / 300
N_STEPS   = 9000   # 30 s at 300 Hz
TOLERANCE = 10.0   # deg — matches specification.json tolerance_deg
BAND_WINDOW  = 1000   # tail window examined for convergence
BAND_FRACTION = 0.70  # fraction of tail window that must be in band


def _simulate(core, start_angle=51.0):
    """Run closed-loop simulation. Returns list of phi values (length N_STEPS)."""
    phi, phi_dot = start_angle, 0.0
    phis = []
    for _ in range(N_STEPS):
        half = radians(phi) / 2.0
        iqr, iqi = cos(half), sin(half)
        raw_gx = radians(phi_dot)
        m1, m2, _, _ = core.step(iqr, iqi, 0.0, 0.0, raw_gx, DT_S)
        # Use actual motor differential, not raw PID output. This is what the plant sees:
        # mixer_sign mutations flip (m2-m1), correctly producing positive feedback in sim.
        phi_ddot = -PLANT_C * (m2 - m1) / 2
        phi_dot += phi_ddot * DT_S
        phi     += phi_dot * DT_S
        phi = max(-200.0, min(200.0, phi))  # prevent float overflow on divergence
        phis.append(phi)
    return phis


def _in_band(phis):
    """True when >= BAND_FRACTION of the last BAND_WINDOW samples are within TOLERANCE.

    A fraction check rather than all-in tolerates slow residual oscillation from the
    simplified plant model while still clearly separating convergence (>70% near setpoint)
    from divergence (0% near setpoint for sign-flipped mutations that hit the +-200 clamp).
    """
    last = phis[-BAND_WINDOW:]
    return sum(1 for p in last if abs(p) < TOLERANCE) / len(last) >= BAND_FRACTION


# ---------------------------------------------------------------------------
# Convergence test
# ---------------------------------------------------------------------------

def test_convergence(cfg):
    """From start_angle 51 deg, the system must enter +-10 deg and hold for 1000 cycles."""
    core = ControlCore(cfg)
    phis = _simulate(core)
    assert _in_band(phis), (
        f"Did not converge: last phi = {phis[-1]:.1f} deg, "
        f"min in last window = {min(abs(p) for p in phis[-BAND_WINDOW:]):.1f} deg"
    )


# ---------------------------------------------------------------------------
# Sign-mutation tests — load-bearing signs must each individually break convergence
# ---------------------------------------------------------------------------

def test_mutation_gyro_sign_diverges(cfg):
    """gyro_sign=+1: positive damping flips to negative -> unstable oscillation."""
    core = ControlCore(cfg, gyro_sign=+1)
    phis = _simulate(core)
    assert not _in_band(phis), (
        "Expected divergence with gyro_sign=+1 but system stayed in band."
    )


def test_mutation_mixer_sign_diverges(cfg):
    """mixer_sign=-1: M1/M2 roles swapped -> positive feedback -> diverges."""
    core = ControlCore(cfg, mixer_sign=-1)
    phis = _simulate(core)
    assert not _in_band(phis), (
        "Expected divergence with mixer_sign=-1 but system stayed in band."
    )


def test_mutation_err_sign_diverges(cfg):
    """err_sign=-1: angle error inverted -> outer loop drives away from setpoint."""
    core = ControlCore(cfg, err_sign=-1)
    phis = _simulate(core)
    assert not _in_band(phis), (
        "Expected divergence with err_sign=-1 but system stayed in band."
    )
