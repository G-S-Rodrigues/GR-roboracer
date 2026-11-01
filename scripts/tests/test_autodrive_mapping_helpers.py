"""The pure parts of the live mapping run SIM-3740 (#44 part 11a): the
settle wait before the controller is released, the no-ground-truth loop
closure, and the dev-ground-truth shape check. The sim and the run itself
are live and local; these need neither.
"""

from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
for extra in (
    REPO_ROOT / "tests" / "lib",
    REPO_ROOT / "ros_ws" / "src" / "racing_evaluation",
):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

from racing_evaluation.map_accuracy import Grid  # noqa: E402
from racing_test_keywords import autodrive_mapping as mapping  # noqa: E402

SIM_3740 = REPO_ROOT / "tests" / "autodrive" / "test_sim_3740_porto_mapping.py"


def test_check_2280_the_settle_wait_needs_a_window_without_a_scan_gap() -> None:
    """CHECK-2280: the controller is released only once /scan has arrived
    for `window_s` with no gap over `max_gap_s`, counting the gap up to now.
    Too few scans, a gap inside the window, or silence up to now are not
    settled (K29/K30: a stalled or slow sim makes the first seconds after a
    reset unrepresentative)."""
    steady = [i * 0.01 for i in range(0, 301)]  # 100 Hz for 3 s
    assert mapping.quiet_for(steady, 3.0, 1.0, 0.05)
    assert not mapping.quiet_for([], 3.0, 1.0, 0.05)
    assert not mapping.quiet_for([2.99], 3.0, 1.0, 0.05)  # one scan only
    gap = [t for t in steady if not 2.3 < t < 2.4]  # a 0.11 s hole
    assert not mapping.quiet_for(gap, 3.0, 1.0, 0.05)
    assert mapping.quiet_for(gap, 3.0, 0.5, 0.05)  # the hole is older
    assert not mapping.quiet_for(steady, 3.2, 1.0, 0.05)  # silent to now
    slow = [i * 0.1 for i in range(0, 31)]  # 10 Hz: the degraded sim
    assert not mapping.quiet_for(slow, 3.0, 1.0, 0.05)


def _room(half: float = 3.0, res: float = 0.05) -> Grid:
    """A square room, wall cells on the boundary of a (2*half)^2 grid."""
    n = int(round(2 * half / res))
    occupied = np.zeros((n, n), dtype=bool)
    occupied[0, :] = occupied[-1, :] = True
    occupied[:, 0] = occupied[:, -1] = True
    return Grid(occupied, ~occupied, res, (-half, -half))


def _scan_from(pose, grid: Grid, beams: int = 181):
    """Ranges of a lidar at `pose` (x, y, yaw) in a room, by ray marching."""
    half = -grid.origin[0] - grid.resolution  # the free side of the wall
    angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    heading = pose[2] + angles
    ranges = []
    for a in heading:
        c, s = math.cos(a), math.sin(a)
        ts = []
        for wall, d, p in ((half, c, pose[0]), (-half, c, pose[0])):
            if abs(d) > 1e-9 and (wall - p) / d > 0:
                ts.append((wall - p) / d)
        for wall, d, p in ((half, s, pose[1]), (-half, s, pose[1])):
            if abs(d) > 1e-9 and (wall - p) / d > 0:
                ts.append((wall - p) / d)
        ranges.append(min(ts))
    return np.array(ranges), angles[0], angles[1] - angles[0]


def _lap(shift=(0.0, 0.0), yaw_bias=0.0, per_lap=120, n=160):
    """A circuit round the room plus a third of a lap more (the run's
    extra travel past the lap line), and the scans it would see; `shift` and
    `yaw_bias` are an odometry that has drifted by the end."""
    grid = _room()
    poses, scans = [], []
    for k in range(n):
        phase = 2 * math.pi * k / per_lap
        truth = (1.5 * math.cos(phase), 1.5 * math.sin(phase), phase + math.pi / 2)
        ranges, a0, da = _scan_from(truth, grid)
        w = k / (n - 1)
        poses.append(
            (truth[0] + shift[0] * w, truth[1] + shift[1] * w, truth[2] + yaw_bias * w)
        )
        scans.append(ranges)
    return grid, np.array(poses), np.array(scans), a0, da


def test_check_2290_loop_closure_is_zero_without_drift_and_sees_drift() -> None:
    """CHECK-2290: the last metres of scans, fitted onto a grid built from the
    first metres, give a correction near zero when the odometry returned to
    where it started, and the drift when it did not (the lap's
    dead-reckoning closure error, with no ground truth)."""
    grid, poses, scans, a0, da = _lap()
    closed = mapping.loop_closure(poses, scans, a0, da, 0.0, 30.0, travel_m=3.0)
    assert math.hypot(closed.dx, closed.dy) < 0.1, closed
    assert abs(closed.dyaw) < 0.05, closed

    grid, poses, scans, a0, da = _lap(shift=(0.4, 0.0))
    drifted = mapping.loop_closure(poses, scans, a0, da, 0.0, 30.0, travel_m=3.0)
    assert math.hypot(drifted.dx, drifted.dy) > 0.25, drifted


def test_check_2300_shape_after_the_fit_ignores_a_rigid_offset() -> None:
    """CHECK-2300 (ruling 2026-10-31): the shape of a produced map against
    truth is scored after the best rigid (x, y, yaw) fit, so a map whose
    frame sits 0.3 m off truth (the odom anchor) scores as the shape it has;
    the raw offset is reported beside it. A wrinkled wall still scores bad."""
    truth = _room()
    shifted = Grid(  # the same walls, 0.3 m off in x and in y
        truth.occupied,
        truth.free,
        truth.resolution,
        (truth.origin[0] + 0.3, truth.origin[1] + 0.3),
    )
    _, poses, scans, a0, da = _lap()
    fit = mapping.shape_after_fit(
        shifted, truth, poses + [0.3, 0.3, 0.0], scans, a0, da, 0.0, 30.0
    )
    assert fit.raw.median == pytest.approx(0.3, abs=0.06)
    assert fit.median < 0.06, fit
    assert fit.p95 < 0.12, fit
    assert math.hypot(fit.correction.dx, fit.correction.dy) == pytest.approx(
        0.42, abs=0.1
    )


def test_check_2310_sim_3740_releases_the_hold_from_its_own_node() -> None:
    """CHECK-2310 (L2): the live mapping test releases follow_the_gap's hold
    through a parameter client on its own rclpy node, never by spawning
    `ros2 param set`; its one subprocess besides the launch is the map
    saver. A spawned CLI is a second ROS process created in the middle of a
    run, which K29 cannot rule out as the stall's trigger."""
    text = SIM_3740.read_text(encoding="utf-8")
    tree = ast.parse(text)
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    assert "SetParameters" in names | attrs
    words = {
        n.value
        for n in ast.walk(tree)
        if isinstance(n, ast.Constant) and isinstance(n.value, str)
    }
    assert "param" not in words, "a `ros2 param ...` command line"
    assert "ros2 param set" not in text
