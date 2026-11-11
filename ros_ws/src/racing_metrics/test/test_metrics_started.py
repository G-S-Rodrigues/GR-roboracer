"""METRICS-2020/2030: ``~/started``, the latched first accepted sample.

The scenario runner proves t=0 by delivery on each link a seeded metric reads
(findings 2026-11-11, sim3030 root cause). racing_metrics emits nothing until
a lap ends, so it latches the first sample it *accepts* (it needs both a
``/ground_truth/scan`` and a state) on ``~/started``, under ``use_sim_time``
only, as the supervisor's ``~/clock_received`` is.

Both nodes run in their own namespace with every absolute name (``/clock``
included) remapped: colcon runs other packages' tier-2 tests in parallel on
one domain and the gym publishes ``/clock`` (repo-gotchas #39).

Step 1's samples carry their own s, and an acknowledged flush separates them
from the first scan, so the test fails only if a state is accepted without a
scan.
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
from racing_interfaces.msg import TrackRelativeState
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

PREFIX = "/test_metrics2020"
NODE = "metrics_2020"
PREFIX_OFF = "/test_metrics2030"
NODE_OFF = "metrics_2030"
OUTPUT = Path(f"/tmp/test_metrics2020_{time.time_ns()}.json")
OUTPUT_OFF = Path(f"/tmp/test_metrics2030_{time.time_ns()}.json")
TRACK = (
    Path(__file__).resolve().parents[4]
    / "config"
    / "tracks"
    / "analytic_circle.yaml"
)
TIMEOUT_SECONDS = 10.0
SENT_S = 1.25
# More than the node's state depth (QoS(10), node.cpp:132): once these are
# all acked, every step-1 state was taken before any scan or evicted.
FLUSH_STATES = 20

RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _node(prefix, name, output, use_sim_time):
    return launch_ros.actions.Node(
        package="racing_metrics",
        executable="racing_metrics_node",
        name=name,
        namespace=prefix,
        parameters=[
            {
                "track_path": str(TRACK),
                "output_path": str(output),
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[
            ("/scenario/metrics", f"{prefix}/metrics"),
            ("/ground_truth/scan", f"{prefix}/scan"),
            ("/safety/status", f"{prefix}/status"),
            ("/ground_truth/track_relative_state", f"{prefix}/state"),
            ("/clock", f"{prefix}/clock"),
        ],
        output="screen",
    )


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            _node(PREFIX, NODE, OUTPUT, True),
            _node(PREFIX_OFF, NODE_OFF, OUTPUT_OFF, False),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMetricsStarted(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("metrics2020_test")
        cls.scans = cls.node.create_publisher(
            LaserScan, f"{PREFIX}/scan", qos_profile_sensor_data
        )
        cls.states = cls.node.create_publisher(
            TrackRelativeState, f"{PREFIX}/state", RELIABLE_QOS
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        OUTPUT.unlink(missing_ok=True)
        OUTPUT_OFF.unlink(missing_ok=True)

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _state(self, nanosec, s):
        state = TrackRelativeState()
        state.header.stamp.sec = 0
        state.header.stamp.nanosec = nanosec
        state.s = s
        self.states.publish(state)

    def _drain_states(self):
        """Barrier: no step-1 state is left unread in the node's reader."""
        for _ in range(FLUSH_STATES):
            self._state(0, SENT_S)
        self.assertTrue(
            self.states.wait_for_all_acked(Duration(seconds=TIMEOUT_SECONDS)),
            "flush states not acked by racing_metrics",
        )

    def test_metrics_2020_started_latches_first_accepted_sample(self):
        received = []
        self.node.create_subscription(
            TrackRelativeState,
            f"{PREFIX}/{NODE}/started",
            received.append,
            LATCHED_QOS,
        )
        # 0. Wait for the node's state reader, so step 1 is not vacuous.
        deadline = time.monotonic() + TIMEOUT_SECONDS
        while (
            self.states.get_subscription_count() < 1
            and time.monotonic() < deadline
        ):
            self._spin(0.05)
        self.assertGreaterEqual(self.states.get_subscription_count(), 1)

        # 1. State only: no clearance yet, so no sample is accepted.
        for _ in range(50):
            self._state(0, 0.0)
            self._spin(0.02)
        self.assertEqual(received, [])

        # 1b. Flush: every step-1 state is now taken (rejected) or evicted, so
        # a step-1 s can only be latched by a node that accepts without a scan.
        self._drain_states()
        self.assertEqual(received, [])

        # 2. A scan (best effort, so keep sending it), then stamp-0 state.
        scan = LaserScan()
        scan.ranges = [1.0, 2.0]
        deadline = time.monotonic() + TIMEOUT_SECONDS
        while not received and time.monotonic() < deadline:
            self.scans.publish(scan)
            self._state(0, SENT_S)
            self._spin(0.05)
        self.assertTrue(received, "no ~/started after a scan and a state")

        # 3. It carries the accepted sample's stamp and s.
        self.assertEqual(received[0].header.stamp.sec, 0)
        self.assertEqual(received[0].header.stamp.nanosec, 0)
        self.assertEqual(received[0].s, SENT_S)

        # 4. Later samples do not publish it again.
        for i in range(1, 11):
            self.scans.publish(scan)
            self._state(i * 10_000_000, SENT_S + 0.1 * i)
            self._spin(0.03)
        self._spin(0.5)
        self.assertEqual(len(received), 1)

        # 5. A late joiner gets exactly the one latched message.
        late = []
        self.node.create_subscription(
            TrackRelativeState,
            f"{PREFIX}/{NODE}/started",
            late.append,
            LATCHED_QOS,
        )
        self._spin(1.0)
        self.assertEqual(len(late), 1)
        self.assertEqual(late[0].s, SENT_S)

    def test_metrics_2030_no_started_endpoint_without_use_sim_time(self):
        client = self.node.create_client(
            Trigger, f"{PREFIX_OFF}/{NODE_OFF}/finalize"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT_SECONDS))
        self._spin(0.5)
        self.assertEqual(
            self.node.count_publishers(f"{PREFIX_OFF}/{NODE_OFF}/started"), 0
        )
