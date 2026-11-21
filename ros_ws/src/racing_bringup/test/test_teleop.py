"""teleop.py: the rate limiter behind `slam.launch.py drive:=teleop`."""

import pytest
from racing_bringup import teleop

LIMITS = teleop.Limits(
    max_speed=1.5,
    max_reverse_speed=0.5,
    acceleration=0.8,
    deceleration=1.5,
    max_steering_angle=0.35,
    steering_rate=1.0,
    input_timeout=0.5,
    max_step=0.1,
    rate_hz=50.0,
)
DT = 0.02


def _run(limiter, speed, steering, seconds, dt=DT):
    """Step `limiter` for `seconds`; every (speed, steering) it returned."""
    history = []
    for _ in range(round(seconds / dt)):
        history.append(limiter.step(speed, steering, dt))
    return history


def test_bringup_1998_speed_ramps_up_at_the_acceleration() -> None:
    """BRINGUP-1998: a step to the top speed rises at acceleration, so
    1 s in it is 0.8 m/s, and it never passes max_speed."""
    limiter = teleop.Limiter(LIMITS)
    history = _run(limiter, 1.5, 0.0, 1.0)
    assert history[-1][0] == pytest.approx(0.8, abs=1e-9)
    history += _run(limiter, 1.5, 0.0, 3.0)
    assert max(speed for speed, _ in history) <= 1.5
    assert history[-1][0] == pytest.approx(1.5, abs=1e-9)


def test_bringup_1998_speed_ramps_down_at_the_deceleration() -> None:
    """BRINGUP-1998: from 1.5 m/s a target of 0 reaches 0 after 1.0 s at
    the deceleration, and never goes below 0."""
    limiter = teleop.Limiter(LIMITS, speed=1.5)
    history = _run(limiter, 0.0, 0.0, 1.0)
    assert history[-1][0] == pytest.approx(0.0, abs=1e-9)
    assert min(speed for speed, _ in history) >= 0.0
    assert history[24][0] == pytest.approx(0.75, abs=1e-9)


def test_bringup_1998_reversing_stops_first_then_accelerates() -> None:
    """BRINGUP-1998: +0.5 to -0.5 decelerates to 0 at the deceleration,
    then accelerates in reverse at the acceleration; a target past the
    reverse speed is clamped to -max_reverse_speed."""
    limiter = teleop.Limiter(LIMITS, speed=0.5)
    speeds = [speed for speed, _ in _run(limiter, -2.0, 0.0, 2.0)]
    stopped = next(i for i, speed in enumerate(speeds) if speed <= 0.0)
    assert (stopped + 1) * DT == pytest.approx(0.5 / 1.5, abs=DT)
    assert all(speed > -1e-12 for speed in speeds[: stopped + 1])
    assert speeds[stopped + 25] == pytest.approx(-0.4, abs=2 * 0.8 * DT)
    assert min(speeds) == pytest.approx(-0.5, abs=1e-9)


def test_bringup_1998_steering_is_clamped_and_rate_limited() -> None:
    """BRINGUP-1998: a steering target of 1.0 is clamped to 0.35 and the
    angle rises at no more than steering_rate."""
    limiter = teleop.Limiter(LIMITS)
    history = [steering for _, steering in _run(limiter, 0.0, 1.0, 1.0)]
    assert history[9] == pytest.approx(0.2, abs=1e-9)
    assert max(history) == pytest.approx(0.35, abs=1e-9)
    previous = 0.0
    for value in history:
        assert value - previous <= 1.0 * DT + 1e-9
        previous = value
    negative = teleop.Limiter(LIMITS)
    assert _run(negative, 0.0, -1.0, 1.0)[-1][1] == pytest.approx(-0.35)


@pytest.mark.parametrize("dt", [-0.015, 0.0])
def test_bringup_1998_a_non_positive_dt_holds_the_output(dt) -> None:
    """BRINGUP-1998 (gotcha #45): the wall clock steps back, so a dt <= 0
    leaves the output as it was."""
    limiter = teleop.Limiter(LIMITS, speed=0.4, steering=0.1)
    assert limiter.step(1.5, 0.35, dt) == (0.4, 0.1)


def test_bringup_1998_a_large_dt_is_capped_at_max_step() -> None:
    """BRINGUP-1998 (gotcha #45): a forward clock step reads as a gap, and
    one 0.5 s dt moves the output by max_step (0.1 s) worth only."""
    limiter = teleop.Limiter(LIMITS)
    speed, steering = limiter.step(1.5, 0.35, 0.5)
    assert speed == pytest.approx(0.8 * 0.1, abs=1e-9)
    assert steering == pytest.approx(0.1, abs=1e-9)


def test_bringup_1998_old_or_missing_input_is_a_zero_target() -> None:
    """BRINGUP-1998: input older than input_timeout, or never received,
    gives targets (0, 0); fresh input passes through as speed in m/s and
    steering angle in rad."""
    assert teleop.targets(1.0, 0.2, 0.1, 0.5) == (1.0, 0.2)
    assert teleop.targets(1.0, 0.2, 0.6, 0.5) == (0.0, 0.0)
    assert teleop.targets(1.0, 0.2, None, 0.5) == (0.0, 0.0)
