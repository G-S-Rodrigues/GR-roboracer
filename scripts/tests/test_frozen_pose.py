"""Tests for scripts.frozen_pose: the one definition of a frozen pose.

Tier 1, stdlib only.
"""

from __future__ import annotations

from scripts.frozen_pose import FROZEN_EPSILON_M, FrozenPose

LENGTH = 100.0
WINDOW = 1.0


def feed(samples, window=WINDOW) -> FrozenPose:
    pose = FrozenPose(window, LENGTH)
    for t, s, d in samples:
        pose.update(t, s, d)
    return pose


def ticks(start, stop, s=10.0, d=0.0, dt=0.01):
    n = round((stop - start) / dt)
    return [(start + i * dt, s, d) for i in range(n + 1)]


def test_check_1540_exact_repeat_tail_at_the_window_is_frozen() -> None:
    moving = [(0.01 * i, 0.3 * i, 0.0) for i in range(100)]
    pose = FrozenPose(WINDOW, LENGTH)
    results = [pose.update(t, s, d) for t, s, d in moving]
    assert not any(results)
    last_s = moving[-1][1]
    flags = [pose.update(t, s, d) for t, s, d in ticks(1.0, 2.5, s=last_s)]
    assert flags[-1] and pose.frozen
    # first true exactly when the still stretch spans the window
    first = flags.index(True)
    assert 0.0 < first * 0.01 <= WINDOW + 1e-9


def test_check_1540_short_pause_is_not_frozen() -> None:
    samples = ticks(0.0, 0.02, s=5.0)
    samples += [(0.03 + 0.01 * i, 5.0 + 0.3 * (i + 1), 0.0) for i in range(300)]
    assert not feed(samples).frozen


def test_check_1540_half_millimetre_creep_stays_frozen() -> None:
    samples = [(0.01 * i, 10.0 + 0.0005 * (i / 150), 0.0) for i in range(151)]
    assert FROZEN_EPSILON_M > 0.0005
    assert feed(samples).frozen


def test_check_1540_two_millimetre_creep_is_not_frozen() -> None:
    samples = [(0.01 * i, 10.0 + 0.002 * (i / 150), 0.0) for i in range(151)]
    assert not feed(samples).frozen


def test_check_1540_lateral_motion_is_not_frozen() -> None:
    samples = [(0.01 * i, 10.0, 0.002 * (i / 150)) for i in range(151)]
    assert not feed(samples).frozen


def test_check_1540_still_stretch_wraps_across_s_zero() -> None:
    samples = [
        (0.01 * i, (LENGTH - 0.0004) if i % 2 else 0.0004, 0.0)
        for i in range(151)
    ]
    assert feed(samples).frozen


def test_check_1540_motion_after_a_stretch_resets_it() -> None:
    pose = feed(ticks(0.0, 2.0, s=10.0))
    assert pose.frozen
    assert not pose.update(2.01, 10.5, 0.0)
    assert not pose.frozen
