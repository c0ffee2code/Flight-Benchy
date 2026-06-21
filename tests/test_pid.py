import pytest
from core.pid import PID


def test_integral_clamps_at_positive_limit():
    pid = PID(kp=0.0, ki=1.0, iterm_limit=5.0)
    for _ in range(100):
        pid.compute(10.0, 0.01)
    assert pid._integral == pytest.approx(5.0)


def test_integral_clamps_at_negative_limit():
    pid = PID(kp=0.0, ki=1.0, iterm_limit=5.0)
    for _ in range(100):
        pid.compute(-10.0, 0.01)
    assert pid._integral == pytest.approx(-5.0)


def test_d_on_measurement_no_kick_on_setpoint_step():
    """D-on-measurement: unchanged measurement must produce D=0 even when error steps."""
    pid = PID(kp=0.0, ki=0.0, kd=1.0)
    pid.compute(5.0, 0.01, measurement=5.0)  # first call establishes prev_measurement
    out = pid.compute(10.0, 0.01, measurement=5.0)  # error steps, measurement stays
    assert out == pytest.approx(0.0, abs=1e-9)


def test_dt_zero_returns_zero_derivative():
    pid = PID(kp=0.0, ki=0.0, kd=1.0)
    pid.compute(1.0, 0.01)
    out = pid.compute(2.0, 0.0)
    assert out == pytest.approx(0.0, abs=1e-9)


def test_first_call_d_is_zero_on_measurement_path():
    """D-on-measurement path must return D=0 on the first call (no prev_measurement)."""
    pid = PID(kp=0.0, ki=0.0, kd=10.0)
    out = pid.compute(100.0, 0.01, measurement=100.0)
    assert out == pytest.approx(0.0, abs=1e-9)


def test_output_clamp():
    pid = PID(kp=100.0, ki=0.0, output_limit=5.0)
    out = pid.compute(10.0, 0.01)
    assert out == pytest.approx(5.0)


def test_reset_clears_all_state():
    pid = PID(kp=1.0, ki=1.0, kd=1.0, iterm_limit=100.0)
    pid.compute(5.0, 0.01, measurement=5.0)
    pid.reset()
    assert pid._integral == 0.0
    assert pid._prev_error == 0.0
    assert pid._prev_measurement is None


def test_ki_zero_integral_stays_zero():
    """F6c: with ki=0 the integrator should never accumulate."""
    pid = PID(kp=0.0, ki=0.0, kd=0.0, iterm_limit=50.0)
    for _ in range(100):
        pid.compute(10.0, 0.01)
    assert pid._integral == pytest.approx(0.0)
