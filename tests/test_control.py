"""ControlCore unit tests — loop scheduling, sign contracts, QA-F2, feedforward direction."""
import pytest
from math import radians, cos, sin
from core.control import ControlCore


def _outer_ticks(cfg):
    return cfg["vehicle"]["loops"]["rate"]["frequency_hz"] // cfg["vehicle"]["loops"]["angle"]["frequency_hz"]


def _dt(cfg):
    return 1.0 / cfg["vehicle"]["loops"]["rate"]["frequency_hz"]


def _q_from_phi(phi_deg):
    """Pure single-axis rotation quaternion for a given roll angle."""
    half = radians(phi_deg) / 2.0
    return cos(half), sin(half), 0.0, 0.0


def _step_n(core, n, iqr, iqi, iqj, iqk, raw_gx, dt):
    for _ in range(n):
        core.step(iqr, iqi, iqj, iqk, raw_gx, dt)


# ---------------------------------------------------------------------------
# QA-F2 — outer_period_s must use float division
# ---------------------------------------------------------------------------

def test_outer_period_qa_f2(cfg):
    """QA-F2: outer_period_s == 0.010 s at 300/100 Hz (not 0.009 from integer division)."""
    core = ControlCore(cfg)
    assert core.outer_period_s == pytest.approx(0.010)


# ---------------------------------------------------------------------------
# Loop scheduling
# ---------------------------------------------------------------------------

def test_outer_tick_fires_every_n_cycles(cfg):
    outer_n = _outer_ticks(cfg)
    dt = _dt(cfg)
    core = ControlCore(cfg)
    iqr, iqi, iqj, iqk = _q_from_phi(20.0)
    raw_gx = radians(-5.0)

    outer_count = 0
    for _ in range(outer_n * 3):
        core.step(iqr, iqi, iqj, iqk, raw_gx, dt)
        if core.last_is_outer:
            outer_count += 1

    assert outer_count == 3


def test_rate_setpoint_zoh_between_outer_ticks(cfg):
    """rate_setpoint must hold its value between outer ticks (zero-order hold)."""
    outer_n = _outer_ticks(cfg)
    dt = _dt(cfg)
    core = ControlCore(cfg)
    iqr, iqi, iqj, iqk = _q_from_phi(30.0)
    raw_gx = radians(-10.0)

    _step_n(core, outer_n, iqr, iqi, iqj, iqk, raw_gx, dt)  # trigger first outer tick
    rate_sp_after_outer = core.last_rate_setpoint

    core.step(iqr, iqi, iqj, iqk, raw_gx, dt)  # one inner tick
    assert not core.last_is_outer
    assert core.last_rate_setpoint == rate_sp_after_outer


# ---------------------------------------------------------------------------
# Sign contracts
# ---------------------------------------------------------------------------

def test_gyro_sign_contract(cfg):
    """gyro_x = -phi_dot: positive phi_dot (M1 falling) must give negative gyro_x."""
    core = ControlCore(cfg)
    dt = _dt(cfg)
    phi_dot_pos = 20.0  # deg/s, M1 end going lower (positive in encoder convention)
    raw_gx = radians(phi_dot_pos)
    iqr, iqi, iqj, iqk = _q_from_phi(30.0)
    core.step(iqr, iqi, iqj, iqk, raw_gx, dt)
    assert core.last_gyro_x < 0


def test_imu_roll_sign_contract(cfg):
    """Positive phi must yield positive ang_err at setpoint=0."""
    outer_n = _outer_ticks(cfg)
    dt = _dt(cfg)
    core = ControlCore(cfg)
    iqr, iqi, iqj, iqk = _q_from_phi(30.0)  # positive angle
    raw_gx = 0.0
    _step_n(core, outer_n, iqr, iqi, iqj, iqk, raw_gx, dt)
    assert core.last_ang_err > 0


# ---------------------------------------------------------------------------
# Feedforward direction — F1 indicator
# ---------------------------------------------------------------------------

def test_feedforward_direction(cfg):
    """F1 indicator: with simulated 12ms GRV lag, ff_sign=+1 (current code) makes the
    angle estimate worse than raw GRV alone; ff_sign=-1 (correct per F1) makes it better.

    This test documents the known bug. It will need updating when F1 is fixed and the
    ff_sign default changes to -1.
    """
    phi = 30.0        # current lever angle (deg, truth)
    phi_dot = -20.0   # deg/s, lever falling
    grv_lag_s = 0.012  # GRV is 12ms stale (= configured lead_ms)

    # GRV reports the angle from 12ms ago (lever was higher since it's falling)
    phi_grv = phi - phi_dot * grv_lag_s  # = 30.24

    iqr, iqi, iqj, iqk = _q_from_phi(phi_grv)
    raw_gx = radians(phi_dot)  # core applies -imu_sign internally -> gyro_x = -phi_dot = +20

    outer_n = _outer_ticks(cfg)
    dt = _dt(cfg)

    core_buggy   = ControlCore(cfg)             # ff_sign=+1 (default, current code)
    core_correct = ControlCore(cfg, ff_sign=-1)  # ff_sign=-1 (correct per F1 analysis)

    _step_n(core_buggy,   outer_n, iqr, iqi, iqj, iqk, raw_gx, dt)
    _step_n(core_correct, outer_n, iqr, iqi, iqj, iqk, raw_gx, dt)

    assert core_buggy.last_is_outer and core_correct.last_is_outer

    # setpoint=0, so ang_err == feedforward estimate
    estimate_buggy   = core_buggy.last_ang_err    # 30.24 + 20*0.012 = 30.48
    estimate_correct = core_correct.last_ang_err  # 30.24 - 20*0.012 = 30.00

    # Bug: feedforward makes estimate worse than raw GRV
    assert abs(estimate_buggy - phi) > abs(phi_grv - phi), (
        f"Expected ff_sign=+1 to worsen estimate vs raw GRV. "
        f"buggy={estimate_buggy:.3f}, phi_grv={phi_grv:.3f}, truth={phi:.3f}"
    )
    # Correct sign: estimate is closer to truth
    assert abs(estimate_correct - phi) < abs(estimate_buggy - phi), (
        f"Expected ff_sign=-1 to be better than ff_sign=+1. "
        f"correct={estimate_correct:.3f}, buggy={estimate_buggy:.3f}, truth={phi:.3f}"
    )
