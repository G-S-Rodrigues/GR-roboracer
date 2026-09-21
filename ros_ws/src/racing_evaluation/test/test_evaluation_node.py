"""EVAL-2010: the evaluation node publishes LocalizationError for a synthetic
estimate - scored while it publishes, unavailable once it stalls.

A synthetic truth drives along the analytic circle's centerline; the
estimate is the same pose 0.3 m to its left. The node must report 0.3 m of
position and lateral error, then, once the estimate stops, NaN errors with
``available`` false - never a zero.
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
EVALUATION_NODE = "racing_evaluation"
ESTIMATE_TOPIC = "/test/estimate"
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
            {"estimate_topic": ESTIMATE_TOPIC, "track_path": str(TRACK)}
        ],
        output="screen",
    )
    return launch.LaunchDescription(
        [evaluation, launch_testing.actions.ReadyToTest()]
    )


def _odometry(stamp: float, x: float, y: float, yaw: float) -> Odometry:
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


class TestEvaluationNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("racing_evaluation_contract_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _require_evaluator(self) -> None:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.node.count_subscribers(
                ESTIMATE_TOPIC
            ) and self.node.count_publishers(ERROR_TOPIC):
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail(f"node not found: {EVALUATION_NODE}")

    def test_eval_2010_scores_a_synthetic_estimate_then_its_stall(self):
        self._require_evaluator()
        received: list[LocalizationError] = []
        self.node.create_subscription(
            LocalizationError, ERROR_TOPIC, received.append, RELIABLE_QOS
        )
        truth = self.node.create_publisher(Odometry, TRUTH_TOPIC, RELIABLE_QOS)
        estimate = self.node.create_publisher(
            Odometry, ESTIMATE_TOPIC, RELIABLE_QOS
        )
        # Discovery of the two new publishers, and of this node's error
        # subscription by the evaluator, before anything is sent.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not (
            truth.get_subscription_count() and estimate.get_subscription_count()
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        settle = time.monotonic() + 1.0
        while time.monotonic() < settle:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # Poses generated from Frenet coordinates, so the answer is known by
        # construction: truth on the centerline, the estimate 0.3 m left.
        track = racing_common.Track.from_yaml(TRACK)
        live_samples = 50
        stall_samples = 100  # 1 s: twice the default 0.5 s window
        for index in range(live_samples + stall_samples):
            stamp = 1.0 + index * PERIOD
            # Inside the first of the octagon's 7.65 m edges: at a vertex a
            # point 0.3 m off the line is equally near two edges.
            s = 1.0 + 0.02 * index
            on_line = track.to_cartesian(racing_common.FrenetPoint(s, 0.0, 0.0))
            truth.publish(_odometry(stamp, on_line.x, on_line.y, on_line.psi))
            if index < live_samples:
                left = track.to_cartesian(
                    racing_common.FrenetPoint(s, OFFSET, 0.0)
                )
                estimate.publish(_odometry(stamp, left.x, left.y, left.psi))
            # At the simulator's own 100 Hz: the estimate subscription is
            # best effort, and a burst would be dropped, not scored.
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(PERIOD)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not any(
            not message.available for message in received
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)

        available = [message for message in received if message.available]
        stale = [message for message in received if not message.available]
        self.assertTrue(available, "no sample was ever scored")
        for message in available:
            self.assertEqual(message.estimate_topic, ESTIMATE_TOPIC)
            self.assertAlmostEqual(message.position_error, OFFSET, places=6)
            self.assertAlmostEqual(message.lateral_error, OFFSET, places=6)
            self.assertAlmostEqual(message.longitudinal_error, 0.0, places=6)
            self.assertAlmostEqual(message.heading_error, 0.0, places=9)
        self.assertTrue(stale, "a stalled estimate was never unavailable")
        for message in stale:
            self.assertTrue(math.isnan(message.position_error))
        last = received[-1]
        self.assertFalse(last.available)
        # The node's own counters, independent of what reached this test.
        self.assertGreaterEqual(last.available_count, live_samples - 1)
        self.assertLess(last.available_count, last.sample_count)
        self.assertAlmostEqual(last.position_rmse, OFFSET, places=6)

    def test_eval_2010_accepts_a_best_effort_estimate(self):
        """repo-gotchas #6: a best-effort pose source must still be heard."""
        self._require_evaluator()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            endpoints = self.node.get_subscriptions_info_by_topic(
                ESTIMATE_TOPIC
            )
            if endpoints:
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)
        (endpoint,) = endpoints
        self.assertEqual(
            endpoint.qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT
        )
        self.assertEqual(
            endpoint.qos_profile.durability, DurabilityPolicy.VOLATILE
        )
