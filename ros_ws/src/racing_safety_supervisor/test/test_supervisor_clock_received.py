"""SAFE-2010/2020: the supervisor reports the first /clock it heard (F1).

rclcpp's TimeSource reader is invisible to other nodes, and a held sim's
/clock = 0 leaves now() at 0, so the supervisor publishes the first sample it
heard on a latched `~/clock_received` (only under `use_sim_time`). The t=0
wait in tests/lib reads it. Every node and topic here lives under
`/test_safe2010/`: colcon runs tier-2 tests in parallel on one domain and the
gym publishes /clock.
"""

import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

PREFIX = "/test_safe2010"
TRACK = (
    Path(__file__).resolve().parents[4] / "config/tracks/analytic_circle.yaml"
)
TIMEOUT_SECONDS = 10.0

CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _supervisor(name, use_sim_time):
    return launch_ros.actions.Node(
        package="racing_safety_supervisor",
        executable="racing_safety_supervisor_node",
        name=name,
        parameters=[{"track_path": str(TRACK), "use_sim_time": use_sim_time}],
        remappings=[
            ("/drive", f"{PREFIX}/{name}/drive"),
            ("/safety/status", f"{PREFIX}/{name}/status"),
            ("/controller/drive", f"{PREFIX}/{name}/controller_drive"),
            ("/ground_truth/odom", f"{PREFIX}/{name}/odom"),
            ("/clock", f"{PREFIX}/clock"),
        ],
        output="screen",
    )


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            _supervisor("safe_sim", True),
            _supervisor("safe_wall", False),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _clock(nanoseconds: int) -> Clock:
    message = Clock()
    message.clock.sec = nanoseconds // 1_000_000_000
    message.clock.nanosec = nanoseconds % 1_000_000_000
    return message


def _stamp_ns(message: Clock) -> int:
    return message.clock.sec * 1_000_000_000 + message.clock.nanosec


class TestClockReceived(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("safe2010_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, condition, seconds=TIMEOUT_SECONDS):
        deadline = time.monotonic() + seconds
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        return condition()

    def _spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def test_safe_2010_first_heard_clock_is_latched_once(self):
        topic = "/safe_sim/clock_received"
        self.assertTrue(
            self._spin_until(lambda: self.node.count_publishers(topic) >= 1),
            "no publisher on /safe_sim/clock_received",
        )
        listener_a = []
        self.node.create_subscription(
            Clock, topic, listener_a.append, LATCHED_QOS
        )
        self._spin_for(0.5)
        self.assertEqual(listener_a, [], "published before any /clock")

        # The held sim: stamp 0 every 10 ms until the supervisor reports.
        clock = self.node.create_publisher(Clock, f"{PREFIX}/clock", CLOCK_QOS)
        deadline = time.monotonic() + TIMEOUT_SECONDS
        while not listener_a and time.monotonic() < deadline:
            clock.publish(_clock(0))
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(listener_a, "supervisor never reported /clock")

        for _ in range(5):
            clock.publish(_clock(10_000_000))
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self._spin_for(0.5)
        self.assertEqual(len(listener_a), 1)
        self.assertEqual(_stamp_ns(listener_a[0]), 0)

        listener_b = []
        self.node.create_subscription(
            Clock, topic, listener_b.append, LATCHED_QOS
        )
        self._spin_until(lambda: bool(listener_b))
        self._spin_for(0.3)
        self.assertEqual(len(listener_b), 1)
        self.assertEqual(_stamp_ns(listener_b[0]), 0)

    def test_safe_2020_no_clock_endpoints_without_use_sim_time(self):
        # The service is created after the clock block in the constructor, so
        # its discovery implies the block's endpoints would be announced.
        client = self.node.create_client(
            Trigger,
            "/safe_wall/clear_emergency_stop",
        )
        self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT_SECONDS))
        self._spin_for(0.5)
        self.assertEqual(
            self.node.count_publishers("/safe_wall/clock_received"), 0
        )
        subscribers = self.node.get_subscriptions_info_by_topic(
            f"{PREFIX}/clock"
        )
        self.assertNotIn("safe_wall", [info.node_name for info in subscribers])
