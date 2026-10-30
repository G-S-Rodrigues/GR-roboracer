"""BRINGUP-1580 and BRINGUP-2030: a track with no committed raceline can still
be driven on its centerline (#44 part 8, coordinator ruling 2026-10-30).

Porto has no `Porto_raceline.csv` (part 11 owns one), yet the support node
needed that file even for `planner:=track_centerline`, so
`simulator:=autodrive` could not start. With the planner on the centerline
and no raceline file, `track_parameters` now gives the support node an empty
`raceline_path`, and the node publishes the track's own centerline as
`/trajectory`. A track that has a raceline file keeps it (the gym goldens read
it), and `optimized_raceline` still fails closed with no fallback.
"""

import math
import os
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from racing_bringup.scenario_parameters import track_parameters
from racing_bringup.support_node import SupportNode
from racing_interfaces.msg import Trajectory
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
PORTO = REPOSITORY_ROOT / "config/scenarios/porto.yaml"
SPIELBERG = REPOSITORY_ROOT / "config/scenarios/spielberg_1080.yaml"
PORTO_TRACK = REPOSITORY_ROOT / "config/tracks/porto.yaml"
LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def test_bringup_1580_no_raceline_file_means_the_centerline_planner() -> None:
    """A scenario whose map has no `<map>_raceline.csv` gets an empty
    raceline_path under track_centerline; one that has the file keeps it, and
    optimized_raceline never falls back to the centerline."""
    porto = track_parameters(PORTO)
    assert porto["racing_bringup_support"]["raceline_path"] == ""
    assert not (
        REPOSITORY_ROOT / "config/scenarios/maps/Porto/Porto_raceline.csv"
    ).exists(), "Porto gained a raceline: this fallback is no longer needed"

    spielberg = track_parameters(SPIELBERG)["racing_bringup_support"]
    assert spielberg["raceline_path"].endswith("_raceline.csv")
    assert Path(spielberg["raceline_path"]).is_file()

    with pytest.raises(FileNotFoundError):
        track_parameters(
            PORTO,
            planner="optimized_raceline",
            vehicle_path=REPOSITORY_ROOT
            / "config/vehicles/autodrive_roboracer.yaml",
        )


def test_bringup_2030_empty_raceline_publishes_the_centerline() -> None:
    """With raceline_path empty the latched /trajectory is the track's
    centerline: the same x, y in order, s the arc length from the first
    point, psi along the tangent, and v 0 (a centerline carries no speed; the
    controller's own schedule sets it)."""
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=200 + os.getpid() % 30)
    support = SupportNode(
        context=context,
        # A parameter override, not `-p raceline_path:=`: the command line
        # cannot spell an empty string.
        parameter_overrides=[
            Parameter("track_path", value=str(PORTO_TRACK)),
            Parameter("raceline_path", value=""),
        ],
    )
    probe = rclpy.create_node("bringup_2030_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(support)
    executor.add_node(probe)
    received: list[Trajectory] = []
    probe.create_subscription(
        Trajectory, "/trajectory", received.append, LATCHED
    )
    try:
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
    finally:
        executor.shutdown()
        probe.destroy_node()
        support.destroy_node()
        rclpy.shutdown(context=context)
    assert received, "no /trajectory"
    trajectory = received[0]
    centerline = yaml.safe_load(PORTO_TRACK.read_text())["centerline"]
    assert trajectory.source == "porto_centerline"
    assert len(trajectory.points) == len(centerline)
    points = trajectory.points
    for point, row in zip(points, centerline, strict=True):
        assert (point.x, point.y) == pytest.approx((row[0], row[1]))
        assert point.v == 0.0
    assert points[0].s == 0.0
    for first, second in zip(points, points[1:], strict=False):
        step = math.hypot(second.x - first.x, second.y - first.y)
        assert second.s - first.s == pytest.approx(step)
    # psi is the direction of travel: the next point lies along it.
    middle = len(points) // 2
    here, there = points[middle], points[middle + 1]
    along = math.atan2(there.y - here.y, there.x - here.x)
    assert math.remainder(here.psi - along, math.tau) == pytest.approx(
        0.0, abs=0.35
    )
