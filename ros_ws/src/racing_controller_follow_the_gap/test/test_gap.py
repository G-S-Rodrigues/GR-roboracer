"""FTG-1010..1060, 1080: the follow-the-gap core on synthetic scans.

The core reads one scan and nothing else. REP-103: positive angle and
steering are to the left. FTG-1070 (Porto's geometry) is in
``test_gap_porto.py``.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from racing_controller_follow_the_gap import gap  # noqa: E402

ANGLE_MIN = -2.35619
INCREMENT = 0.004363323
BEAMS = 1081
RANGE_MIN = 0.06
P = gap.Parameters()


def angles() -> np.ndarray:
    return ANGLE_MIN + INCREMENT * np.arange(BEAMS)


def corridor(half_width: float, ahead: float = 10.0) -> np.ndarray:
    """Straight corridor: walls at y = +-half_width, open ahead and behind."""
    a = angles()
    with np.errstate(divide="ignore"):
        side = half_width / np.abs(np.sin(a))
    ahead_cap = np.where(np.abs(a) < math.pi / 2, ahead, 10.0)
    return np.minimum(side, ahead_cap)


def walled(depth: float = 0.5) -> np.ndarray:
    return np.full(BEAMS, depth)


def opening(centre: float, width: float, far: float = 5.0, depth=0.5):
    """A wall all round with one opening; the nearest return sits on the
    side opposite the opening, so the safety bubble never covers it."""
    r = walled(depth)
    a = angles()
    r[int(np.argmin(np.abs(a - math.copysign(1.5, -centre))))] = 0.4
    r[np.abs(a - centre) <= width / 2] = far
    return r


def run(ranges, p=P):
    return gap.command(ranges, ANGLE_MIN, INCREMENT, RANGE_MIN, p)


def test_ftg_1010_clean_ranges():
    r = np.array([math.inf, math.nan, 0.01, 2.0, 99.0, 1.5, 1.5])
    a, c = gap.clean_ranges(r, -0.01, INCREMENT, RANGE_MIN, P)  # all in view
    assert len(a) == len(c) == len(r)
    assert np.all(np.isfinite(c))
    assert c[0] == P.max_range and c[1] == P.max_range  # inf, NaN
    assert c[2] == P.max_range  # below range_min: a returns-nothing beam
    assert c[3] == 2.0
    assert c[4] == P.max_range  # clipped
    # Beams outside +-fov/2 are ignored.
    full = np.full(BEAMS, 3.0)
    a, c = gap.clean_ranges(full, ANGLE_MIN, INCREMENT, RANGE_MIN, P)
    assert np.all(np.abs(a) <= P.fov / 2 + 1e-9)
    assert 0 < len(a) < BEAMS


def test_ftg_1020_bubble_zeroes_beams_round_the_nearest_return():
    a = angles()
    r = np.full(BEAMS, 4.0)
    near = 700
    r[near] = 0.5
    out = gap.apply_bubble(a, r, P.__class__(bubble_radius=0.25))
    half = math.asin(0.25 / 0.5)  # 30 degrees
    inside = np.abs(a - a[near]) <= half - 1e-6
    outside = np.abs(a - a[near]) > half + 1e-6
    assert np.all(out[inside] == 0.0)
    assert np.all(out[outside] == 4.0)
    # A return nearer than the radius clears the half-width: asin(min(1, .))
    r2 = np.full(BEAMS, 4.0)
    r2[near] = 0.1
    out2 = gap.apply_bubble(a, r2, P.__class__(bubble_radius=0.25))
    assert np.all(out2[np.abs(a - a[near]) <= math.pi / 2 - 1e-6] == 0.0)


def test_ftg_1030_widest_gap_deepest_beam_tie_nearer_ahead():
    a = angles()
    thr = P.gap_threshold
    # Wide gap on the left (60 deg), narrow on the right (20 deg).
    r = np.zeros(BEAMS)
    wide = np.abs(a - 1.0) <= math.radians(30)
    narrow = np.abs(a + 1.0) <= math.radians(10)
    r[wide] = thr + 1.0
    r[narrow] = thr + 1.0
    spike = int(np.argmin(np.abs(a - 0.9)))
    r[spike] = thr + 3.0
    assert gap.choose_target(a, r, P) == spike
    # Equal widths: the gap nearer straight ahead wins.
    r = np.zeros(BEAMS)
    near = np.abs(a - 0.35) <= math.radians(12)
    far = np.abs(a + 1.2) <= math.radians(12)
    r[near] = thr + 1.0
    r[far] = thr + 1.0
    # make the widths exactly equal in beams
    assert near.sum() == far.sum()
    i = gap.choose_target(a, r, P)
    assert near[i] and not far[i]
    # No gap at all.
    assert gap.choose_target(a, np.zeros(BEAMS), P) is None


def test_ftg_1040_steering_is_the_target_angle_left_positive():
    steering, _ = run(opening(math.radians(10), math.radians(8)))
    assert steering == pytest.approx(math.radians(10), abs=0.02)
    steering, _ = run(opening(math.radians(-10), math.radians(8)))
    assert steering == pytest.approx(math.radians(-10), abs=0.02)
    steering, _ = run(opening(math.radians(70), math.radians(10)))
    assert steering == P.maximum_steering_angle  # clipped, left positive
    steering, _ = run(opening(math.radians(-70), math.radians(10)))
    assert steering == -P.maximum_steering_angle
    steering, _ = run(corridor(1.0))
    assert abs(steering) < 0.01


def test_ftg_1050_speed_falls_with_steering_and_rises_with_clearance():
    full = P.clearance_for_full_speed
    s = gap.speed
    assert s(0.0, full, P) == P.maximum_speed
    assert s(0.3, full, P) < s(0.1, full, P) < s(0.0, full, P)
    assert s(0.0, full / 2, P) < s(0.0, full, P)
    assert s(P.maximum_steering_angle, 0.0, P) == P.minimum_speed
    for steering in (-0.5, -0.1, 0.0, 0.2, 0.5):
        for clearance in (0.0, 0.5, 2.0, 10.0):
            v = s(steering, clearance, P)
            assert P.minimum_speed <= v <= P.maximum_speed
    # No free gap: stop.
    steering, speed = run(walled(0.4))
    assert speed == 0.0 and steering == 0.0
    # A corridor open ahead drives faster than one closed 1 m ahead.
    assert run(corridor(1.0, 10.0))[1] > run(corridor(1.0, 1.2))[1]


@pytest.mark.parametrize(
    "bad",
    [
        {"bubble_radius": 0.0},
        {"max_range": 0.0},
        {"fov": 0.0},
        {"fov": 7.0},
        {"gap_threshold": 0.0},
        {"gap_threshold": 99.0},
        {"maximum_steering_angle": 0.0},
        {"minimum_speed": -0.1},
        {"minimum_speed": 3.0, "maximum_speed": 2.0},
        {"clearance_for_full_speed": 0.0},
        {"steering_gain": 0.0},
        {"launch_acceleration": 0.0},
    ],
)
def test_ftg_1060_invalid_parameters_raise(bad):
    with pytest.raises(ValueError):
        gap.Parameters(**bad).validate()
    gap.Parameters().validate()


def test_ftg_1080_launch_ramp_limits_speed_from_the_start():
    p = gap.Parameters(
        minimum_speed=0.5, maximum_speed=2.0, launch_acceleration=1.0
    )
    assert gap.launch_limit(0.0, p) == 0.5  # starts at the minimum speed
    assert gap.launch_limit(0.5, p) == pytest.approx(1.0)
    assert gap.launch_limit(10.0, p) == 2.0  # never above the maximum
    assert gap.launch_limit(-1.0, p) == 0.5  # before the start: the minimum
    assert gap.limit_speed(2.0, 0.5, p) == pytest.approx(1.0)
    assert gap.limit_speed(0.3, 0.5, p) == 0.3  # a lower command is kept
