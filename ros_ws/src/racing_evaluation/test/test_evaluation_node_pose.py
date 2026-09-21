"""EVAL-2020: the evaluation node scores a PoseWithCovarianceStamped estimate,
the type slam_toolbox (``/pose``) and nav2_amcl (``/amcl_pose``) publish,
exactly as it scores a nav_msgs/Odometry one.

``estimate_type: pose_with_covariance`` selects it. Truth drives along the
analytic circle's centerline and the estimate sits 0.3 m to its left, so the
node must report 0.3 m of position and lateral error. Only the node's message
translation differs from EVAL-2010; the core is the same.
"""

import math
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import racing_common
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from racing_interfaces.msg import LocalizationError
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
TRACK = REPOSITORY_ROOT / "config" / "tracks" / "analytic_circle.yaml"
ESTIMATE_TOPIC = "/test/pose_estimate"
TRUTH_TOPIC = "/ground_truth/odom"
ERROR_TOPIC = "/evaluation/localization_error"
OFFSET = 0.3
PERIOD = 0.01

RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
def generate_test_description():
    evaluation = launch_ros.actions.Node(
        package="racing_evaluation",
        executable="racing_evaluation_node",
        parameters=[
            {
                "estimate_topic": ESTIMATE_TOPIC,
                "estimate_type": "pose_with_covariance",
                "track_path": str(TRACK),
            }
        ],
        output="screen",
    )
    return launch.LaunchDescription(
        [evaluation, launch_testing.actions.ReadyToTest()]
    )


def _stamp(message, stamp: float) -> None:
    message.header.frame_id = "map"
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)


def _set_pose(pose, x: float, y: float, yaw: float) -> None:
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)


class TestEvaluationNodePose(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("racing_evaluation_pose_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_eval_2020_scores_a_pose_with_covariance_estimate(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not (
            self.node.get_subscriptions_info_by_topic(ESTIMATE_TOPIC)
            and self.node.count_publishers(ERROR_TOPIC)
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        (endpoint,) = self.node.get_subscriptions_info_by_topic(ESTIMATE_TOPIC)
        self.assertEqual(
            endpoint.topic_type, "geometry_msgs/msg/PoseWithCovarianceStamped"
        )

        received: list[LocalizationError] = []
        self.node.create_subscription(
            LocalizationError, ERROR_TOPIC, received.append, RELIABLE_QOS
        )
        truth = self.node.create_publisher(Odometry, TRUTH_TOPIC, RELIABLE_QOS)
        estimate = self.node.create_publisher(
            PoseWithCovarianceStamped, ESTIMATE_TOPIC, RELIABLE_QOS
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not (
            truth.get_subscription_count() and estimate.get_subscription_count()
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        settle = time.monotonic() + 1.0
        while time.monotonic() < settle:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        track = racing_common.Track.from_yaml(TRACK)
        samples = 50
        for index in range(samples):
            stamp = 1.0 + index * PERIOD
            # Inside the octagon's first edge, as in EVAL-2010.
            s = 1.0 + 0.02 * index
            on_line = track.to_cartesian(racing_common.FrenetPoint(s, 0.0, 0.0))
            left = track.to_cartesian(racing_common.FrenetPoint(s, OFFSET, 0.0))
            odometry = Odometry()
            _stamp(odometry, stamp)
            _set_pose(odometry.pose.pose, on_line.x, on_line.y, on_line.psi)
            truth.publish(odometry)
            pose = PoseWithCovarianceStamped()
            _stamp(pose, stamp)
            _set_pose(pose.pose.pose, left.x, left.y, left.psi)
            estimate.publish(pose)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(PERIOD)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and len(received) < samples // 2:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        available = [message for message in received if message.available]
        self.assertGreaterEqual(len(available), samples // 2)
        for message in available:
            self.assertEqual(message.estimate_topic, ESTIMATE_TOPIC)
            self.assertAlmostEqual(message.position_error, OFFSET, places=6)
            self.assertAlmostEqual(message.lateral_error, OFFSET, places=6)
            self.assertAlmostEqual(message.longitudinal_error, 0.0, places=6)
            self.assertAlmostEqual(message.heading_error, 0.0, places=9)
