"""Tier-2 conformance tests shared by every simulator backend."""

import math
import time
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry
from racing_interfaces.srv import Reset
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSCompatibility,
    QoSProfile,
    ReliabilityPolicy,
    qos_check_compatible,
)
from sensor_msgs.msg import Imu, LaserScan

ADAPTER_NODE = "racing_sim"
SCAN_TOPIC = "/scan"
ODOMETRY_TOPIC = "/odom"
IMU_TOPIC = "/imu"
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


@pytest.mark.launch_test
def generate_test_description():
    backend = launch.actions.ExecuteProcess(
        cmd=[
            "python3",
            "-c",
            "import signal, subprocess; "
            "subprocess.run(['ros2', 'run', 'racing_sim_gym_jax', "
            "'racing_sim_gym_jax_node']); signal.pause()",
        ],
        output="screen",
    )
    return launch.LaunchDescription(
        [backend, launch_testing.actions.ReadyToTest()]
    )


@unittest.skip(
    "Step 9 implements the racing_sim_gym_jax backend and turns "
    "ADAPT-2010..2050 green"
)
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
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if ADAPTER_NODE in self.node.get_node_names():
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
        self.assertEqual(odometry.header.frame_id, "map")
        self.assertEqual(odometry.child_frame_id, "base_link")
        self.assertEqual(imu.header.frame_id, "base_link")

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
            compatibility = qos_check_compatible(publisher_qos, expected_qos)
            self.assertNotEqual(
                compatibility.compatibility,
                QoSCompatibility.ERROR,
                compatibility.reason,
            )

    def test_adapt_2050_seeded_reset_is_idempotent(self):
        """ADAPT-2050: equal reset seeds reproduce the initial state."""
        self._require_adapter()
        self._reset(seed=2050, scenario_id="contract_test")
        first = self._collect(Odometry, ODOMETRY_TOPIC, STATE_QOS, 1)[0]
        self._reset(seed=2050, scenario_id="contract_test")
        second = self._collect(Odometry, ODOMETRY_TOPIC, STATE_QOS, 1)[0]
        self.assertEqual(first.pose.pose, second.pose.pose)
        self.assertEqual(first.twist.twist, second.twist.twist)
