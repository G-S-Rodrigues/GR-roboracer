"""BRINGUP-2020: the support node hands an estimator its start pose once.

nav2_amcl has to be told where it starts, and the start pose is drawn from
the run's seed (``f110_env.reset`` places the car at a random arc length),
so no static pose can be configured. With ``initial_pose_topic`` set, the
support node publishes the first ground-truth pose there - latched, so it
reaches AMCL however late the lifecycle manager activates it - and never
publishes again: re-seeding a particle filter mid-lap would erase the
divergence the run is measuring rather than record it.

The covariance is deliberately not zero. It stands in for RViz's "2D Pose
Estimate" - a human putting the car roughly where it is - and a zero one
would start AMCL exactly at the truth, making its convergence unmeasurable.

Default (empty topic) publishes nothing at all, which is what keeps the
ground-truth reference stack's behaviour unchanged.

Runs the node in-process on a private ROS domain, like BRINGUP-2010.
"""

import math
import os
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from racing_bringup.support_node import (
    INITIAL_POSE_VARIANCES,
    SupportNode,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

REPOSITORY_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
TRACK = os.path.join(REPOSITORY_ROOT, "config/tracks/analytic_circle.yaml")
RACELINE = os.path.join(
    REPOSITORY_ROOT,
    "config/scenarios/maps/analytic_circle/analytic_circle_raceline.csv",
)
INITIAL_POSE_TOPIC = "/initialpose"
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
START = (12.0, -3.0, 0.9)
LATER = (13.0, -3.5, 1.1)


def _yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2)
    )


def _spin_until(executor, condition, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.02)
    return condition()


def _odometry(stamp_sec: int, pose) -> Odometry:
    message = Odometry()
    message.header.stamp.sec = stamp_sec
    message.header.frame_id = "map"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = pose[0]
    message.pose.pose.position.y = pose[1]
    message.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
    message.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
    return message


def _graph(context, extra_parameters):
    parameters = [
        "--ros-args",
        "-p",
        f"track_path:={TRACK}",
        "-p",
        f"raceline_path:={RACELINE}",
        *extra_parameters,
    ]
    support = SupportNode(context=context, cli_args=parameters)
    probe = rclpy.create_node("bringup_2020_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(support)
    executor.add_node(probe)
    return support, probe, executor


@pytest.fixture
def context():
    context = rclpy.Context()
    # A private domain: --fast may run while a tier-3 graph is live.
    rclpy.init(context=context, domain_id=200 + os.getpid() % 30)
    yield context
    rclpy.shutdown(context=context)


def test_bringup_2020_publishes_the_start_pose_once_latched(context):
    support, probe, executor = _graph(
        context, ["-p", f"initial_pose_topic:={INITIAL_POSE_TOPIC}"]
    )
    try:
        truth = probe.create_publisher(
            Odometry, "/ground_truth/odom", RELIABLE_QOS
        )
        assert _spin_until(
            executor, lambda: truth.get_subscription_count() > 0, 5.0
        ), "support node never subscribed /ground_truth/odom"

        # The node keeps only the latest /ground_truth/odom (depth 1), so
        # each is delivered separately - two in a row and the first is
        # simply overwritten before it is taken.
        truth.publish(_odometry(1, START))
        _spin_until(executor, lambda: False, 0.3)
        truth.publish(_odometry(2, LATER))
        _spin_until(executor, lambda: False, 0.3)

        # Subscribing only now: an estimator activated by a lifecycle
        # manager is late, and a volatile publisher would have nothing for
        # it.
        received: list[PoseWithCovarianceStamped] = []
        probe.create_subscription(
            PoseWithCovarianceStamped,
            INITIAL_POSE_TOPIC,
            received.append,
            LATCHED_QOS,
        )
        assert _spin_until(executor, lambda: len(received) >= 1, 5.0), (
            "no latched initial pose"
        )

        truth.publish(_odometry(3, LATER))
        _spin_until(executor, lambda: False, 0.5)
        assert len(received) == 1, "the start pose was published more than once"

        initial = received[0]
        assert initial.header.frame_id == "map"
        pose = initial.pose.pose
        assert pose.position.x == pytest.approx(START[0], abs=1e-9)
        assert pose.position.y == pytest.approx(START[1], abs=1e-9)
        assert _yaw(pose.orientation) == pytest.approx(START[2], abs=1e-9)
        x_var, y_var, yaw_var = INITIAL_POSE_VARIANCES
        assert initial.pose.covariance[0] == pytest.approx(x_var)
        assert initial.pose.covariance[7] == pytest.approx(y_var)
        assert initial.pose.covariance[35] == pytest.approx(yaw_var)
        assert x_var > 0.0 and yaw_var > 0.0
    finally:
        executor.shutdown()
        probe.destroy_node()
        support.destroy_node()


def test_bringup_2020_publishes_nothing_by_default(context):
    support, probe, executor = _graph(context, [])
    try:
        received: list[PoseWithCovarianceStamped] = []
        probe.create_subscription(
            PoseWithCovarianceStamped,
            INITIAL_POSE_TOPIC,
            received.append,
            LATCHED_QOS,
        )
        truth = probe.create_publisher(
            Odometry, "/ground_truth/odom", RELIABLE_QOS
        )
        assert _spin_until(
            executor, lambda: truth.get_subscription_count() > 0, 5.0
        )
        truth.publish(_odometry(1, START))
        _spin_until(executor, lambda: False, 0.5)
        assert received == []
    finally:
        executor.shutdown()
        probe.destroy_node()
        support.destroy_node()
