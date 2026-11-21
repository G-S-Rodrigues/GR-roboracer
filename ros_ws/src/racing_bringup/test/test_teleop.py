"""teleop.py: the rate limiter behind `slam.launch.py drive:=teleop`."""

import dataclasses
from pathlib import Path

import pytest
import yaml
from racing_bringup import teleop

REPOSITORY_ROOT = Path(__file__).parents[1].parents[2]
KEYBOARD = REPOSITORY_ROOT / "config" / "teleop" / "autodrive_keyboard.yaml"
VEHICLES = (
    REPOSITORY_ROOT / "config" / "vehicles" / "autodrive_roboracer.yaml",
    REPOSITORY_ROOT / "config" / "vehicles" / "autodrive_roboracer_fast.yaml",
)

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


@pytest.mark.parametrize(
    ("linear_x", "angular_z"),
    [(float("nan"), 0.0), (0.0, float("inf")), (float("-inf"), 0.0)],
)
def test_bringup_1998_a_non_finite_input_is_a_zero_target(
    linear_x, angular_z
) -> None:
    """BRINGUP-1998: a nan or infinite component reads as a fault, not as
    full command: min/max would pass it through as the limit."""
    assert teleop.targets(linear_x, angular_z, 0.0, 0.5) == (0.0, 0.0)


def _write(tmp_path: Path, **overrides) -> Path:
    """The default file's parameters with `overrides`; a None value drops
    the key."""
    document = yaml.safe_load(KEYBOARD.read_text(encoding="utf-8"))
    parameters = document["racing_bringup_teleop"]["ros__parameters"]
    parameters.update(overrides)
    for key in [k for k, v in parameters.items() if v is None]:
        del parameters[key]
    path = tmp_path / "teleop.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_bringup_1999_the_default_file_is_the_gentle_set() -> None:
    """BRINGUP-1999: the committed file loads into Limits, with the gentle
    values (1.5 m/s, 0.8 m/s^2, 0.35 rad) the plan settled."""
    assert teleop.load(KEYBOARD) == teleop.Limits(
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


@pytest.mark.parametrize("vehicle", VEHICLES, ids=lambda p: p.stem)
def test_bringup_1999_the_default_is_within_both_vehicles_caps(
    vehicle,
) -> None:
    """BRINGUP-1999: the committed default passes check_caps for both
    AutoDRIVE vehicle files."""
    teleop.check_caps(teleop.load(KEYBOARD), vehicle)


def test_bringup_1999_a_missing_or_extra_key_is_refused_by_name(
    tmp_path: Path,
) -> None:
    """BRINGUP-1999: a missing key and an unknown key each raise a
    RuntimeError naming it."""
    with pytest.raises(RuntimeError, match="rate_hz"):
        teleop.load(_write(tmp_path, rate_hz=None))
    with pytest.raises(RuntimeError, match="top_speed"):
        teleop.load(_write(tmp_path, top_speed=9.0))


@pytest.mark.parametrize(
    ("key", "value", "match"),
    [
        ("max_speed", float("nan"), "max_speed"),
        ("deceleration", -0.8, "deceleration"),
        ("acceleration", 1, "write 1.0"),
        ("rate_hz", 0.0, "rate_hz"),
        ("max_reverse_speed", -0.1, "max_reverse_speed"),
    ],
)
def test_bringup_1999_a_non_finite_wrong_type_or_non_positive_limit_is_refused(
    tmp_path: Path, key: str, value: float, match: str
) -> None:
    """BRINGUP-1999: load refuses nan, a non-positive value (a negative
    max_reverse_speed; zero is allowed there only) and an integer, which the
    node's DOUBLE parameter would reject at start, naming the key."""
    with pytest.raises(RuntimeError, match=match):
        teleop.load(_write(tmp_path, **{key: value}))


def test_bringup_1999_a_zero_max_reverse_speed_is_allowed(
    tmp_path: Path,
) -> None:
    """BRINGUP-1999: max_reverse_speed 0.0 (no reverse) loads."""
    limits = teleop.load(_write(tmp_path, max_reverse_speed=0.0))
    assert limits.max_reverse_speed == 0.0


@pytest.mark.parametrize(
    ("key", "value", "cap"),
    [
        ("max_speed", 5.0, "maximum_speed"),
        ("acceleration", 5.0, "maximum_acceleration"),
        ("max_steering_angle", 0.9, "maximum_steering_angle"),
        ("steering_rate", 3.0, "maximum_steering_rate"),
        ("deceleration", 5.0, "maximum_acceleration"),
    ],
)
def test_bringup_1999_a_limit_above_the_vehicle_cap_is_refused(
    tmp_path: Path, key: str, value: float, cap: str
) -> None:
    """BRINGUP-1999: a limit above the vehicle's racing_safety_supervisor
    cap raises, naming the key, the cap and both values."""
    limits = teleop.load(_write(tmp_path, **{key: value}))
    with pytest.raises(RuntimeError) as raised:
        teleop.check_caps(limits, VEHICLES[0])
    assert key in str(raised.value)
    assert cap in str(raised.value)
    assert str(value) in str(raised.value)


def test_bringup_1999_reverse_may_not_exceed_the_forward_cap(
    tmp_path: Path,
) -> None:
    """BRINGUP-1999: max_reverse_speed above maximum_speed is refused."""
    limits = dataclasses.replace(teleop.load(KEYBOARD), max_reverse_speed=4.0)
    with pytest.raises(RuntimeError, match="max_reverse_speed"):
        teleop.check_caps(limits, VEHICLES[0])
