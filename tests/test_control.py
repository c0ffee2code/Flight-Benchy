"""ControlCore unit tests — loop scheduling, sign contracts, outer period precision, feedforward direction."""
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
# outer_period_s must use float division
# ---------------------------------------------------------------------------

def test_outer_period_float_division(cfg):
    """outer_period_s == 0.010 s at 300/100 Hz (not 0.009 from integer division)."""
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
# Feedforward direction
# ---------------------------------------------------------------------------

def test_feedforward_direction(cfg):
    """ControlCore must produce an angle estimate closer to truth than raw GRV.

    GRV has a lag. The feedforward lead compensates by extrapolating forward using
    gyro rate: phi_predicted = phi_grv + phi_dot * lead = phi_grv - gyro_x * lead.
    That requires ff_sign=-1 in the formula (imu_roll + ff_sign * gyro_x * lead).
    ff_sign=+1 retrodicates instead — making the estimate worse than raw GRV.
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

    core_default = ControlCore(cfg)                     # ff_sign=-1 from config (correct)
    cfg["vehicle"]["signs"]["ff_sign"] = 1
    core_wrong   = ControlCore(cfg)                     # ff_sign=+1 retrodicates

    _step_n(core_default, outer_n, iqr, iqi, iqj, iqk, raw_gx, dt)
    _step_n(core_wrong,   outer_n, iqr, iqi, iqj, iqk, raw_gx, dt)

    assert core_default.last_is_outer and core_wrong.last_is_outer

    # setpoint=0, so ang_err == feedforward estimate
    estimate_default = core_default.last_ang_err  # with ff_sign=-1: 30.24 - 20*0.012 = 30.00
    estimate_wrong   = core_wrong.last_ang_err    # with ff_sign=+1: 30.24 + 20*0.012 = 30.48

    # Default (ff_sign=-1): estimate is closer to truth than raw GRV
    assert abs(estimate_default - phi) < abs(phi_grv - phi), (
        f"Expected default ff_sign to improve estimate vs raw GRV. "
        f"default={estimate_default:.3f}, phi_grv={phi_grv:.3f}, truth={phi:.3f}"
    )
    # Wrong sign (ff_sign=+1): estimate is worse than raw GRV
    assert abs(estimate_wrong - phi) > abs(phi_grv - phi), (
        f"Expected ff_sign=+1 to worsen estimate vs raw GRV. "
        f"wrong={estimate_wrong:.3f}, phi_grv={phi_grv:.3f}, truth={phi:.3f}"
    )
