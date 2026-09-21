"""BRINGUP-2010: under an estimator, the support node composes the pose the
controller drives on and leaves map -> odom to the estimator.

An estimator (slam_toolbox, nav2_amcl) owns map -> odom and publishes it at
scan rate; the controller needs a nav_msgs/Odometry in ``map`` at control
rate. With ``publish_map_to_odom: false`` and ``composed_odometry_topic``
set, the node must publish, per ``/odom`` message, the latest map -> odom
composed with that dead reckoning - and must never broadcast map -> odom
itself, or the TF tree has two owners of one edge.

Runs the node in-process on a private ROS domain. racing_bringup is an
ament_python package, so colcon runs every pytest file here under --fast;
this test costs ~1 s.
"""

import math
import os
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from racing_bringup.support_node import SupportNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage
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
RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# The estimator's correction, and the dead reckoning it is applied to.
MAP_TO_ODOM = (1.0, 2.0, math.pi / 2)
DEAD_RECKONING = (4.0, -1.0, 0.7)
EXPECTED = (2.0, 6.0, 0.7 + math.pi / 2)


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


@pytest.fixture
def graph():
    context = rclpy.Context()
    # A private domain: --fast may run while a tier-3 graph is live.
    rclpy.init(context=context, domain_id=200 + os.getpid() % 30)
    parameters = [
        "--ros-args",
        "-p",
        f"track_path:={TRACK}",
        "-p",
        f"raceline_path:={RACELINE}",
        "-p",
        "publish_map_to_odom:=false",
        "-p",
        f"composed_odometry_topic:={COMPOSED_TOPIC}",
    ]
    support = SupportNode(context=context, cli_args=parameters)
    probe = rclpy.create_node("bringup_2010_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(support)
    executor.add_node(probe)
    yield support, probe, executor
    executor.shutdown()
    probe.destroy_node()
    support.destroy_node()
    rclpy.shutdown(context=context)


def test_bringup_2010_composes_the_estimate_and_leaves_tf_alone(graph):
    support, probe, executor = graph
    composed: list[Odometry] = []
    support_transforms: list[TransformStamped] = []
    probe.create_subscription(
        Odometry, COMPOSED_TOPIC, composed.append, RELIABLE_QOS
    )

    def _on_tf(message: TFMessage) -> None:
        support_transforms.extend(
            t for t in message.transforms if t.header.frame_id == "map"
        )

    probe.create_subscription(TFMessage, "/tf", _on_tf, RELIABLE_QOS)
    dead_reckoning = probe.create_publisher(Odometry, "/odom", RELIABLE_QOS)
    truth = probe.create_publisher(Odometry, "/ground_truth/odom", RELIABLE_QOS)
    estimator = TransformBroadcaster(probe)
    assert _spin_until(
        executor,
        lambda: (
            dead_reckoning.get_subscription_count()
            and truth.get_subscription_count()
        ),
        5.0,
    ), "support node never subscribed /odom and /ground_truth/odom"

    def _odometry(stamp_sec: int, pose) -> Odometry:
        message = Odometry()
        message.header.stamp.sec = stamp_sec
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = pose[0]
        message.pose.pose.position.y = pose[1]
        message.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        message.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        message.twist.twist.linear.x = 2.5
        return message

    # Before the estimator has published map -> odom there is no pose to
    # drive on: nothing is composed, rather than a pose in the wrong frame.
    dead_reckoning.publish(_odometry(1, DEAD_RECKONING))
    _spin_until(executor, lambda: False, 0.3)
    assert composed == []

    transform = TransformStamped()
    transform.header.stamp.sec = 1
    transform.header.frame_id = "map"
    transform.child_frame_id = "odom"
    transform.transform.translation.x = MAP_TO_ODOM[0]
    transform.transform.translation.y = MAP_TO_ODOM[1]
    transform.transform.rotation.z = math.sin(MAP_TO_ODOM[2] / 2.0)
    transform.transform.rotation.w = math.cos(MAP_TO_ODOM[2] / 2.0)
    estimator.sendTransform(transform)
    _spin_until(executor, lambda: False, 0.3)
    support_transforms.clear()  # the estimator's own broadcast

    for stamp in (2, 3):
        # Truth paired with dead reckoning by stamp is exactly what made
        # the ground-truth source broadcast its correction.
        truth.publish(_odometry(stamp, (9.0, 9.0, 0.0)))
        dead_reckoning.publish(_odometry(stamp, DEAD_RECKONING))
        # The node keeps only the latest /odom (depth 1): deliver each.
        _spin_until(executor, lambda: False, 0.2)
    assert _spin_until(executor, lambda: len(composed) >= 2, 5.0)
    _spin_until(executor, lambda: False, 0.3)

    for message in composed:
        assert message.header.frame_id == "map"
        assert message.child_frame_id == "base_link"
        pose = message.pose.pose
        assert pose.position.x == pytest.approx(EXPECTED[0], abs=1e-9)
        assert pose.position.y == pytest.approx(EXPECTED[1], abs=1e-9)
        assert math.remainder(
            _yaw(pose.orientation) - EXPECTED[2], 2 * math.pi
        ) == pytest.approx(0.0, abs=1e-9)
        assert message.twist.twist.linear.x == 2.5
    assert [m.header.stamp.sec for m in composed] == [2, 3]
    assert support_transforms == [], "support node broadcast map -> odom"
