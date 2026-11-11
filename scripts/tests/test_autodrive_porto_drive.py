"""CHECK-2483: what `drive_porto` waits for before the t = 0 reset when the
pose source is an estimator (1V-fix-b step b2).

An estimator that starts publishing after the drive stream does is repo-gotcha
#19's latch. The gym runners wait for it (`scenario_runner.POSE_SOURCE_NODES`);
the AutoDRIVE driver now does the same, from the same table. `estimator_waits`
is the pure part: what to wait for, in order. `_porto_drive` imports rclpy, so
this runs in `dev` with ROS sourced, like `test_scenario_runner_t0.py`.
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
for extra in (ROOT / "tests" / "lib", ROOT / "tests" / "autodrive"):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

import _porto_drive  # noqa: E402
from racing_test_keywords import scenario_runner  # noqa: E402


def test_check_2483_estimator_waits_per_pose_source() -> None:
    """CHECK-2483: ground truth waits for nothing (SIM-3730/3750 are
    unchanged); amcl waits for its node, its /scan subscription, then the
    first message on the estimate topic; mola and slam_toolbox wait for their
    node and /scan subscription only (the gym runner's reasons: MOLA's
    estimate needs time to advance, slam_toolbox's needs motion); an unknown
    pose source raises ValueError naming it; every pose source the gym runner
    knows is known here, from the same table."""
    waits = _porto_drive.estimator_waits
    assert waits("ground_truth") == ()
    assert waits("amcl") == (
        ("node", "amcl", "/scan"),
        ("subscription", "amcl", "/scan"),
        ("message", "", scenario_runner.ESTIMATE_TOPIC),
    )
    for source, node in (
        ("mola", "mola_bridge_ros2"),
        ("slam_toolbox", "slam_toolbox"),
    ):
        assert waits(source) == (
            ("node", node, "/scan"),
            ("subscription", node, "/scan"),
        ), source
    for source in scenario_runner.POSE_SOURCE_NODES:
        assert waits(source), source
    with pytest.raises(ValueError, match="far_lio"):
        waits("far_lio")


def test_check_2483_wait_for_estimator_uses_one_deadline(monkeypatch) -> None:
    """CHECK-2483: `wait_for_estimator` runs the waits in order through the
    ros_helpers/scenario_runner functions, each handed the time left of ONE
    deadline (never a fresh budget per wait); ground truth makes no call."""
    calls: list[tuple] = []

    def record(kind):
        def fake(*args):
            calls.append((kind, *args[1:-1], args[-1]))

        return fake

    monkeypatch.setattr(_porto_drive, "wait_for_node", record("node"))
    monkeypatch.setattr(
        _porto_drive, "wait_for_endpoints", record("subscription")
    )
    monkeypatch.setattr(
        _porto_drive,
        "wait_for_message",
        lambda *a: calls.append(("message", a)),
    )
    _porto_drive.wait_for_estimator(object(), "ground_truth", 5.0)
    assert calls == []
    _porto_drive.wait_for_estimator(object(), "amcl", 5.0)
    kinds = [c[0] for c in calls]
    assert kinds == ["node", "subscription", "message"], calls
    budgets = [c[-1] for c in calls[:2]]
    assert all(0.0 < b <= 5.0 for b in budgets), budgets
    assert budgets[1] <= budgets[0], budgets


def test_check_2487_hold_error_names_what_the_hold_saw() -> None:
    """CHECK-2487 (a)-(c): `hold_error(reasons, stale_count)` is None only
    when the hold saw the emergency stop latch (REASON_ESTOP_LATCHED), no
    REASON_STALE_INPUT and `stale_count == 0`: the release clears a STALE
    latch too (supervisor.cpp), so it must never do so silently. Otherwise it
    returns a message naming what it saw."""
    err = _porto_drive.hold_error
    latched = Counter({"REASON_NONE": 3, "REASON_ESTOP_LATCHED": 40})
    assert err(latched, 0) is None
    stale = latched + Counter({"REASON_STALE_INPUT": 1})
    message = err(stale, 0)
    assert message is not None and "REASON_STALE_INPUT" in message, message
    message = err(latched, 2)
    assert message is not None and "2" in message, message
    message = err(Counter({"REASON_NONE": 5}), 0)
    assert message is not None and "REASON_ESTOP_LATCHED" in message, message
    assert err(Counter(), 0) is not None


def test_check_2487_released_reasons_drops_the_leading_latch_only() -> None:
    """CHECK-2487 (e): `released_reasons(names)` is the reasons received
    after the clear response, minus the leading run of REASON_ESTOP_LATCHED
    (statuses queued before the clear took effect). A STALE right at the
    release, then its latch, is kept; all-latched is empty."""
    out = _porto_drive.released_reasons
    n, e, s = "REASON_NONE", "REASON_ESTOP_LATCHED", "REASON_STALE_INPUT"
    assert out([e, e, n, n]) == Counter({n: 2})
    assert out([e, s, e]) == Counter({s: 1, e: 1})
    assert out([e, e]) == Counter()
    assert out([]) == Counter()
    assert out([n, e]) == Counter({n: 1, e: 1})
