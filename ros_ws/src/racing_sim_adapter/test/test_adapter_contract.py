"""Tier-2 conformance tests shared by every simulator backend."""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racing_interfaces.srv import Reset
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSCompatibility,
    QoSProfile,
    ReliabilityPolicy,
    qos_check_compatible,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan

ADAPTER_NODE = "racing_sim"
SCAN_TOPIC = "/scan"
ODOMETRY_TOPIC = "/odom"
GROUND_TRUTH_SCAN_TOPIC = "/ground_truth/scan"
GROUND_TRUTH_ODOMETRY_TOPIC = "/ground_truth/odom"
DRIVE_TOPIC = "/drive"
IMU_TOPIC = "/imu"
CLOCK_TOPIC = "/clock"
RESET_SERVICE = f"/{ADAPTER_NODE}/reset"
EXPECTED_RATE_HZ = 100.0
RATE_TOLERANCE = 0.20

SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# Matches rclpy's own TimeSource subscription QoS (rclpy/time_source.py) -
# the ClockQoS a use_sim_time consumer expects. A durability mismatch here
# is gotcha #6's shape: the graph stalls at t=0 with nothing in the logs.
CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
def generate_test_description():
    backend = launch_ros.actions.Node(
        package="racing_sim_gym_jax",
        executable="racing_sim_gym_jax_node",
        output="screen",
    )
    return launch.LaunchDescription(
        [backend, launch_testing.actions.ReadyToTest()]
    )


def _stamp_ns(message: Clock) -> int:
    return message.clock.sec * 1_000_000_000 + message.clock.nanosec


class TestAdapterContract(unittest.TestCase):
    """Assertions that define the simulator boundary."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("racing_sim_adapter_contract_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _require_adapter(self) -> None:
        # The JAX backend deliberately compiles before it publishes. A cold
        # process import plus compile is slower than ordinary ROS discovery.
        required_topics = {SCAN_TOPIC, ODOMETRY_TOPIC, IMU_TOPIC}
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            topics = {
                name for name, _types in self.node.get_topic_names_and_types()
            }
            if (
                ADAPTER_NODE in self.node.get_node_names()
                and required_topics.issubset(topics)
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail(f"node not found: {ADAPTER_NODE}")

    def _collect(self, message_type, topic, qos, count, timeout=3.0):
        messages = []
        subscription = self.node.create_subscription(
            message_type, topic, messages.append, qos
        )
        deadline = time.monotonic() + timeout
        try:
            while len(messages) < count and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        finally:
            self.node.destroy_subscription(subscription)
        self.assertGreaterEqual(len(messages), count, f"no data on {topic}")
        return messages

    def _reset(self, seed: int, scenario_id: str):
        client = self.node.create_client(Reset, RESET_SERVICE)
        self.assertTrue(
            client.wait_for_service(timeout_sec=2.0), "reset unavailable"
        )
        request = Reset.Request(seed=seed, scenario_id=scenario_id)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        self.assertTrue(future.done(), "reset timed out")
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success, response.message)
        self.node.destroy_client(client)

    def test_adapt_2010_publishes_standard_sensor_types(self):
        """ADAPT-2010: scan, odometry, and IMU use standard ROS types."""
        self._require_adapter()
        expected = {
            SCAN_TOPIC: "sensor_msgs/msg/LaserScan",
            ODOMETRY_TOPIC: "nav_msgs/msg/Odometry",
            IMU_TOPIC: "sensor_msgs/msg/Imu",
        }
        discovered = dict(self.node.get_topic_names_and_types())
        for topic, message_type in expected.items():
            self.assertIn(topic, discovered)
            self.assertIn(message_type, discovered[topic])

    def test_adapt_2020_uses_canonical_frames(self):
        """ADAPT-2020: world, vehicle, and sensor frame names are stable."""
        self._require_adapter()
        scan = self._collect(LaserScan, SCAN_TOPIC, SENSOR_QOS, 1)[0]
        odometry = self._collect(Odometry, ODOMETRY_TOPIC, STATE_QOS, 1)[0]
        imu = self._collect(Imu, IMU_TOPIC, SENSOR_QOS, 1)[0]
        self.assertEqual(scan.header.frame_id, "laser")
        self.assertEqual(odometry.header.frame_id, "odom")
        self.assertEqual(odometry.child_frame_id, "base_link")
        self.assertEqual(imu.header.frame_id, "base_link")

    def test_adapt_2110_dead_reckoning_and_ground_truth_differ(self):
        """ADAPT-2110: /odom is dead reckoning in `odom`, /ground_truth/odom
        is the exact pose in `map`, and once the vehicle moves they differ.

        The last clause is the point: asserting only that both topics exist
        passes with the odometry noise disabled.
        """
        self._require_adapter()
        publisher = self.node.create_publisher(
            AckermannDriveStamped, DRIVE_TOPIC, STATE_QOS
        )
        estimates = {}
        truths = {}

        def key(message):
            return (message.header.stamp.sec, message.header.stamp.nanosec)

        subscriptions = [
            self.node.create_subscription(
                Odometry,
                ODOMETRY_TOPIC,
                lambda message: estimates.__setitem__(key(message), message),
                STATE_QOS,
            ),
            self.node.create_subscription(
                Odometry,
                GROUND_TRUTH_ODOMETRY_TOPIC,
                lambda message: truths.__setitem__(key(message), message),
                STATE_QOS,
            ),
        ]
        command = AckermannDriveStamped()
        command.drive.speed = 2.0
        command.drive.steering_angle = 0.1
        deadline = time.monotonic() + 2.0
        try:
            while time.monotonic() < deadline:
                publisher.publish(command)
                rclpy.spin_once(self.node, timeout_sec=0.01)
        finally:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
            self.node.destroy_publisher(publisher)

        common = sorted(set(estimates) & set(truths))
        self.assertTrue(common, "no simultaneous /odom and ground truth")
        estimate = estimates[common[-1]]
        truth = truths[common[-1]]
        self.assertEqual(estimate.header.frame_id, "odom")
        self.assertEqual(truth.header.frame_id, "map")
        self.assertEqual(estimate.child_frame_id, "base_link")
        self.assertEqual(truth.child_frame_id, "base_link")
        first = truths[common[0]].pose.pose.position
        moved = math.hypot(
            truth.pose.pose.position.x - first.x,
            truth.pose.pose.position.y - first.y,
        )
        self.assertGreater(moved, 0.5, "the vehicle did not move")
        drift = math.hypot(
            estimate.pose.pose.position.x - truth.pose.pose.position.x,
            estimate.pose.pose.position.y - truth.pose.pose.position.y,
        )
        self.assertGreater(drift, 1e-4, "odometry equals ground truth")

    def test_adapt_2030_publish_rate_is_within_tolerance(self):
        """ADAPT-2030: the backend publishes at its configured 100 Hz rate."""
        self._require_adapter()
        messages = self._collect(Odometry, ODOMETRY_TOPIC, STATE_QOS, 25)
        first = messages[0].header.stamp
        last = messages[-1].header.stamp
        elapsed = (last.sec - first.sec) + (last.nanosec - first.nanosec) / 1e9
        measured_rate = (len(messages) - 1) / elapsed
        self.assertTrue(math.isfinite(measured_rate))
        self.assertLessEqual(
            abs(measured_rate - EXPECTED_RATE_HZ) / EXPECTED_RATE_HZ,
            RATE_TOLERANCE,
        )

    def test_adapt_2040_qos_matches_on_both_ends(self):
        """ADAPT-2040: publisher and consumer QoS profiles are compatible."""
        self._require_adapter()
        for topic, expected_qos in (
            (SCAN_TOPIC, SENSOR_QOS),
            (ODOMETRY_TOPIC, STATE_QOS),
            (IMU_TOPIC, SENSOR_QOS),
        ):
            publishers = self.node.get_publishers_info_by_topic(topic)
            self.assertEqual(
                len(publishers), 1, f"publisher missing on {topic}"
            )
            publisher_qos = publishers[0].qos_profile
            self.assertEqual(
                publisher_qos.reliability, expected_qos.reliability
            )
            self.assertEqual(publisher_qos.durability, expected_qos.durability)
            compatibility, reason = qos_check_compatible(
                publisher_qos, expected_qos
            )
            self.assertNotEqual(compatibility, QoSCompatibility.ERROR, reason)

    def test_adapt_2100_both_scans_publish_with_sensor_qos(self):
        """ADAPT-2100: /scan and /ground_truth/scan both publish, and both
        publishers carry the sensor QoS *profile*; ground-truth odometry
        carries the state profile (repo-gotchas #6, as ADAPT-2040).
        """
        self._require_adapter()
        for topic, message_type, expected_qos in (
            (SCAN_TOPIC, LaserScan, SENSOR_QOS),
            (GROUND_TRUTH_SCAN_TOPIC, LaserScan, SENSOR_QOS),
            (GROUND_TRUTH_ODOMETRY_TOPIC, Odometry, STATE_QOS),
        ):
            publishers = self.node.get_publishers_info_by_topic(topic)
            self.assertEqual(
                len(publishers), 1, f"publisher missing on {topic}"
            )
            publisher_qos = publishers[0].qos_profile
            self.assertEqual(
                publisher_qos.reliability, expected_qos.reliability
            )
            self.assertEqual(publisher_qos.durability, expected_qos.durability)
            compatibility, reason = qos_check_compatible(
                publisher_qos, expected_qos
            )
            self.assertNotEqual(compatibility, QoSCompatibility.ERROR, reason)
            self._collect(message_type, topic, expected_qos, 1)

    def test_adapt_2050_seeded_reset_is_idempotent(self):
        """ADAPT-2050: equal reset seeds reproduce the initial state."""
        self._require_adapter()
        self._reset(seed=2050, scenario_id="contract_test")
        first = self._collect(Odometry, ODOMETRY_TOPIC, STATE_QOS, 1)[0]
        self._reset(seed=2050, scenario_id="contract_test")
        second = self._collect(Odometry, ODOMETRY_TOPIC, STATE_QOS, 1)[0]
        self.assertEqual(first.pose.pose, second.pose.pose)
        self.assertEqual(first.twist.twist, second.twist.twist)

    def test_adapt_2070_clock_is_published_and_tracks_time_scale(self):
        """ADAPT-2070: /clock advances monotonically at time_scale x
        control_period, on a QoS profile a use_sim_time consumer accepts."""
        self._require_adapter()
        publishers = self.node.get_publishers_info_by_topic(CLOCK_TOPIC)
        self.assertEqual(len(publishers), 1, "clock publisher missing")
        publisher_qos = publishers[0].qos_profile
        self.assertEqual(publisher_qos.reliability, CLOCK_QOS.reliability)
        self.assertEqual(publisher_qos.durability, CLOCK_QOS.durability)
        compatibility, reason = qos_check_compatible(publisher_qos, CLOCK_QOS)
        self.assertNotEqual(compatibility, QoSCompatibility.ERROR, reason)

        messages = self._collect(Clock, CLOCK_TOPIC, CLOCK_QOS, 20)
        stamps_ns = [
            message.clock.sec * 1_000_000_000 + message.clock.nanosec
            for message in messages
        ]
        for earlier, later in zip(stamps_ns, stamps_ns[1:], strict=False):
            self.assertLess(
                earlier, later, "/clock must advance strictly monotonically"
            )
        elapsed = (stamps_ns[-1] - stamps_ns[0]) / 1e9
        measured_rate = (len(messages) - 1) / elapsed
        self.assertTrue(math.isfinite(measured_rate))
        self.assertLessEqual(
            abs(measured_rate - EXPECTED_RATE_HZ) / EXPECTED_RATE_HZ,
            RATE_TOLERANCE,
        )

    def test_adapt_2075_use_sim_time_consumer_follows_clock(self):
        """ADAPT-2075: a use_sim_time consumer reads /clock, not the wall.

        Proves ADAPT-2070's /clock is actually *consumable*, not merely
        published - and that the sim node publishing it does not stall
        itself waiting for the clock it is responsible for advancing (the
        plan's one unproven assumption).
        """
        self._require_adapter()
        self._collect(Clock, CLOCK_TOPIC, CLOCK_QOS, 5)

        consumer = rclpy.create_node(
            "adapt_2075_use_sim_time_consumer",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            ],
        )
        try:
            deadline = time.monotonic() + 5.0
            while (
                consumer.get_clock().now().nanoseconds == 0
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(consumer, timeout_sec=0.05)
            consumer_now_ns = consumer.get_clock().now().nanoseconds
        finally:
            consumer.destroy_node()

        self.assertGreater(
            consumer_now_ns,
            0,
            "use_sim_time consumer never received /clock "
            "(the circular-clock case, or a QoS mismatch)",
        )
        # A node still on the wall clock reads ~1.7e18 ns (POSIX epoch); a
        # consumer following simulated time reads a small tick counter, so
        # the two are never confusable.
        self.assertLess(consumer_now_ns, 1e15)

    def test_adapt_2085_runtime_reset_does_not_rewind_clock(self):
        """ADAPT-2085: resetting a sim that was never held keeps /clock
        monotonic across the reset.

        This launch (unlike test_adapter_start_held.py's) never sets
        `start_held`, so the backend has been advancing and publishing
        /clock since startup - exactly the case ADAPT-2080's held-then-
        released sim does not cover. Zeroing the simulated-time counter on
        this reset would publish a stamp behind ones a use_sim_time
        consumer already followed; racing_metrics aborts a run on a
        non-monotonic stamp for exactly this reason.
        """
        self._require_adapter()
        before = self._collect(Clock, CLOCK_TOPIC, CLOCK_QOS, 5)
        self._reset(seed=2085, scenario_id="")
        after = self._collect(Clock, CLOCK_TOPIC, CLOCK_QOS, 5)
        stamps_ns = [_stamp_ns(message) for message in before + after]
        for earlier, later in zip(stamps_ns, stamps_ns[1:], strict=False):
            self.assertLessEqual(
                earlier, later, "/clock must not rewind across a reset"
            )
