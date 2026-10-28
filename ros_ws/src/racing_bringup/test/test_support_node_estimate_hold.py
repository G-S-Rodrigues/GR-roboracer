"""BRINGUP-2030: no estimate is composed until the estimator has applied
the start pose.

MOLA-LO takes /initialpose only after the ICP of its first scan, so until
then its map -> odom is the pose it would have without being told: tens of
metres off, and a controller or an evaluator reading it sees the car there.
With ``hold_estimate_until_initial_pose`` set, the support node composes
nothing until the estimator's composed pose is near the start pose it
published - a judgement from the estimator's own output, with no ground
truth and no seed. Default off: amcl and slam_toolbox are unchanged.

Runs the node in-process on a private ROS domain, like BRINGUP-2010.
"""

import math
import os
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from racing_bringup.support_node import SupportNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster

REPOSITORY_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
TRACK = os.path.join(REPOSITORY_ROOT, "config/tracks/analytic_circle.yaml")
RACELINE = os.path.join(
    REPOSITORY_ROOT,
    "config/scenarios/maps/analytic_circle/analytic_circle_raceline.csv",
)
COMPOSED_TOPIC = "/localization/odom"
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _spin_until(executor, condition, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.02)
    return condition()


def _odometry(stamp_sec: int, frame: str) -> Odometry:
    message = Odometry()
    message.header.stamp.sec = stamp_sec
    message.header.frame_id = frame
    message.child_frame_id = "base_link"
    message.pose.pose.orientation.w = 1.0
    return message


def _map_to_odom(stamp_sec: int, x: float) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp.sec = stamp_sec
    transform.header.frame_id = "map"
    transform.child_frame_id = "odom"
    transform.transform.translation.x = x
    transform.transform.rotation.w = 1.0
    return transform


def _run(hold: bool):
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=200 + os.getpid() % 30)
    support = SupportNode(
        context=context,
        cli_args=[
            "--ros-args",
            "-p",
            f"track_path:={TRACK}",
            "-p",
            f"raceline_path:={RACELINE}",
            "-p",
            "publish_map_to_odom:=false",
            "-p",
            f"composed_odometry_topic:={COMPOSED_TOPIC}",
            "-p",
            "initial_pose_topic:=/initialpose",
            "-p",
            f"hold_estimate_until_initial_pose:={str(hold).lower()}",
        ],
    )
    probe = rclpy.create_node("bringup_2030_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(support)
    executor.add_node(probe)
    return context, support, probe, executor


def _scenario(hold: bool, start: str = "truth") -> list[float]:
    """x of every composed pose: a stale (stamp 1) then an applied (stamp 2)
    map -> odom, each followed by one /odom. ``start`` is where the start pose
    comes from: the sim's ground truth, or an operator's /initialpose with no
    ground truth at all (a real car)."""
    context, support, probe, executor = _run(hold)
    try:
        composed: list[Odometry] = []
        probe.create_subscription(
            Odometry, COMPOSED_TOPIC, composed.append, RELIABLE_QOS
        )
        truth = probe.create_publisher(
            Odometry, "/ground_truth/odom", RELIABLE_QOS
        )
        dead = probe.create_publisher(Odometry, "/odom", RELIABLE_QOS)
        estimator = TransformBroadcaster(probe)
        initial = probe.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", LATCHED_QOS
        )
        assert _spin_until(
            executor,
            lambda: (
                truth.get_subscription_count() and dead.get_subscription_count()
            ),
            5.0,
        )
        # The start pose is published at stamp 1 (the held sim's stamp).
        if start == "truth":
            truth.publish(_odometry(1, "map"))
        else:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp.sec = 1
            pose.header.frame_id = "map"
            pose.pose.pose.orientation.w = 1.0
            initial.publish(pose)
        _spin_until(executor, lambda: False, 0.3)
        # The estimator's pre-/initialpose map -> odom, stamped 1, tens of
        # metres off.
        estimator.sendTransform(_map_to_odom(1, 34.0))
        _spin_until(executor, lambda: False, 0.3)
        dead.publish(_odometry(1, "odom"))
        _spin_until(executor, lambda: False, 0.3)
        # After it applied /initialpose: stamped later.
        estimator.sendTransform(_map_to_odom(2, 0.5))
        _spin_until(executor, lambda: False, 0.3)
        dead.publish(_odometry(2, "odom"))
        _spin_until(executor, lambda: len(composed) >= (1 if hold else 2), 3.0)
        _spin_until(executor, lambda: False, 0.3)
        return [m.pose.pose.position.x for m in composed]
    finally:
        executor.shutdown()
        probe.destroy_node()
        support.destroy_node()
        rclpy.shutdown(context=context)


def test_bringup_2030_withholds_the_estimate_until_the_start_pose_is_applied():
    xs = _scenario(hold=True)
    assert xs == [pytest.approx(0.5)], (
        f"the pre-/initialpose estimate was composed: {xs}"
    )


def test_bringup_2030_off_by_default_composes_every_estimate():
    xs = _scenario(hold=False)
    assert xs[0] == pytest.approx(34.0) and math.isclose(xs[-1], 0.5)


def test_bringup_2030_takes_the_start_pose_from_initialpose_without_truth():
    """A real car has no /ground_truth/odom: the operator's /initialpose is
    the only start pose, and the hold must still release on it."""
    xs = _scenario(hold=True, start="initialpose")
    assert xs == [pytest.approx(0.5)], (
        f"the hold never released on /initialpose alone: {xs}"
    )
