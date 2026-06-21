import pytest
from core.mixer import LeverMixer


@pytest.fixture
def mixer():
    return LeverMixer(throttle_base=600, throttle_min=90, throttle_max=900)


def test_zero_output_returns_base(mixer):
    m1, m2, _, _ = mixer.compute(0)
    assert m1 == 600 and m2 == 600


def test_positive_output_differential(mixer):
    """Positive output: m1 decreases, m2 increases by the same amount."""
    m1, m2, _, _ = mixer.compute(50)
    assert m1 == 550 and m2 == 650


def test_negative_output_differential(mixer):
    m1, m2, _, _ = mixer.compute(-50)
    assert m1 == 650 and m2 == 550


def test_m3_m4_equal_m1_m2(mixer):
    m1, m2, m3, m4 = mixer.compute(100)
    assert m3 == m1 and m4 == m2


def test_upper_clamp(mixer):
    _, m2, _, _ = mixer.compute(400)  # 600 + 400 = 1000 -> 900
    assert m2 == 900


def test_lower_clamp(mixer):
    _, m2, _, _ = mixer.compute(-600)  # 600 + (-600) = 0 -> 90
    assert m2 == 90


def test_f7_invariant():
    """F7: base +/- output_limit must stay within [throttle_min, throttle_max]."""
    mixer = LeverMixer(throttle_base=600, throttle_min=90, throttle_max=900)
    output_limit = 300  # from config
    assert mixer.throttle_base + output_limit <= mixer.throttle_max
    assert mixer.throttle_base - output_limit >= mixer.throttle_min
