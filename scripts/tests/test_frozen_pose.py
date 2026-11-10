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


def test_check_2374_frozen_at_is_the_anchor_once_frozen() -> None:
    """CHECK-2374: the freeze time and place, for a failure's text."""
    pose = FrozenPose(2.0, 343.3)
    assert pose.frozen_at is None
    for i in range(51):  # moving until 110.50
        pose.update(110.0 + 0.01 * i, 173.0 + 0.0159 * i, -0.88)
        assert pose.frozen_at is None
    # constant from 110.51
    for i in range(1, 260):
        pose.update(110.50 + 0.01 * i, 173.8, -0.88)
    assert pose.frozen
    t, s, d = pose.frozen_at
    assert (round(t, 2), s, d) == (110.51, 173.8, -0.88)


def test_check_2374_frozen_at_is_none_for_a_moving_pose() -> None:
    pose = FrozenPose(1.0, LENGTH)
    for t, s, d in [(0.01 * i, 0.3 * i, 0.0) for i in range(300)]:
        pose.update(t, s, d)
    assert pose.frozen_at is None


# ---- CHECK-2374: the live wait reports where the pose froze ----


def _wait_harness(monkeypatch, samples, metrics_after=None):
    """_wait_for_metrics_or_frozen_pose on a fake node: each spin feeds one
    state sample; the next spin delivers metrics once finalize was called, or
    after `metrics_after` samples (a lap that ends with no freeze)."""
    import sys
    from pathlib import Path
    from types import SimpleNamespace

    lib = Path(__file__).resolve().parents[2] / "tests" / "lib"
    if str(lib) not in sys.path:
        sys.path.insert(0, str(lib))
    from racing_test_keywords import scenario_runner

    callbacks: dict = {}
    called: list = []
    queue = list(samples)
    fed = [0]
    metrics = SimpleNamespace(lap_completed=False)

    class Client:
        def call_async(self, request):
            called.append(request)

    class Node:
        def create_client(self, *args):
            return Client()

        def create_subscription(self, kind, topic, callback, qos):
            callbacks[topic] = callback
            return topic

        def destroy_subscription(self, subscription):
            pass

        def destroy_client(self, client):
            pass

    def spin_once(node, timeout_sec=None):
        done = metrics_after is not None and fed[0] >= metrics_after
        if called or done:
            callbacks[scenario_runner.METRICS_TOPIC](metrics)
            return
        if not queue:
            return
        t, s, d = queue.pop(0)
        fed[0] += 1
        stamp = SimpleNamespace(sec=int(t), nanosec=round((t % 1) * 1e9))
        callbacks[scenario_runner.STATE_TOPIC](
            SimpleNamespace(header=SimpleNamespace(stamp=stamp), s=s, d=d)
        )

    monkeypatch.setattr(scenario_runner.rclpy, "spin_once", spin_once)
    return scenario_runner, Node(), metrics


def test_check_2374_wait_reports_frozen_at(monkeypatch) -> None:
    moving = [(110.0 + 0.01 * i, 173.0 + 0.0159 * i, -0.88) for i in range(51)]
    still = [(110.51 + 0.01 * i, 173.8, -0.88) for i in range(300)]
    runner, node, metrics = _wait_harness(monkeypatch, moving + still)
    message, frozen_at = runner._wait_for_metrics_or_frozen_pose(
        node, 5.0, 2.0, 343.3
    )
    assert message is metrics
    assert (round(frozen_at[0], 2), frozen_at[1], frozen_at[2]) == (
        110.51,
        173.8,
        -0.88,
    )


def test_check_2374_wait_reports_none_when_metrics_come_unfrozen(
    monkeypatch,
) -> None:
    moving = [(0.01 * i, 0.3 * i, 0.0) for i in range(50)]
    runner, node, metrics = _wait_harness(monkeypatch, moving, metrics_after=50)
    message, frozen_at = runner._wait_for_metrics_or_frozen_pose(
        node, 5.0, 2.0, 343.3
    )
    assert message is metrics
    assert frozen_at is None
