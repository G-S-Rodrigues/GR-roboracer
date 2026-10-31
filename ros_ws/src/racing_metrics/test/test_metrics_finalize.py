"""METRICS-1030: ``~/finalize`` on a lap that never completed.

Nothing called ``racing_metrics``'s finalize service before the frozen-pose
stop (``run_scenario(stop_on_frozen_pose=...)``) did. The contract it relies
on: the first call publishes one ``ScenarioMetrics`` with
``lap_completed=false`` and writes the JSON, and a second call succeeds without
publishing again.

Every node and topic is renamed to a `/test_metrics1030/` name: colcon runs
other packages' tier-2 tests in parallel on one domain.
"""

import json
import subprocess
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from racing_interfaces.msg import ScenarioMetrics, TrackRelativeState
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

PREFIX = "/test_metrics1030"
NODE = "metrics_1030"
OUTPUT = Path(f"/tmp/test_metrics1030_{time.time_ns()}.json")
TRACK = (
    Path(__file__).resolve().parents[4]
    / "config"
    / "tracks"
    / "analytic_circle.yaml"
)
TIMEOUT_SECONDS = 10.0
# METRICS-2010: a second node, laps:=2, on its own names.
PREFIX2 = "/test_metrics2010"
NODE2 = "metrics_2010"
OUTPUT2 = Path(f"/tmp/test_metrics2010_{time.time_ns()}.json")
CIRCLE_LENGTH = 61.2293492  # 8 * 20 * sin(pi/8): analytic_circle.yaml

# node.cpp: reliable, depth 10 on the state input; reliable + transient_local
# depth 1 on the metrics output.
RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
METRICS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="racing_metrics",
                executable="racing_metrics_node",
                name=NODE,
                parameters=[
                    {"track_path": str(TRACK), "output_path": str(OUTPUT)}
                ],
                remappings=[
                    ("/scenario/metrics", f"{PREFIX}/metrics"),
                    ("/ground_truth/scan", f"{PREFIX}/scan"),
                    ("/safety/status", f"{PREFIX}/status"),
                    (
                        "/ground_truth/track_relative_state",
                        f"{PREFIX}/state",
                    ),
                ],
                output="screen",
            ),
            launch_ros.actions.Node(
                package="racing_metrics",
                executable="racing_metrics_node",
                name=NODE2,
                parameters=[
                    {
                        "track_path": str(TRACK),
                        "output_path": str(OUTPUT2),
                        "laps": 2,
                    }
                ],
                remappings=[
                    ("/scenario/metrics", f"{PREFIX2}/metrics"),
                    ("/ground_truth/scan", f"{PREFIX2}/scan"),
                    ("/safety/status", f"{PREFIX2}/status"),
                    (
                        "/ground_truth/track_relative_state",
                        f"{PREFIX2}/state",
                    ),
                ],
                output="screen",
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMetricsFinalize(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("metrics1030_test")
        cls.received = []
        cls.node.create_subscription(
            ScenarioMetrics,
            f"{PREFIX}/metrics",
            cls.received.append,
            METRICS_QOS,
        )
        cls.scans = cls.node.create_publisher(
            LaserScan, f"{PREFIX}/scan", qos_profile_sensor_data
        )
        cls.states = cls.node.create_publisher(
            TrackRelativeState, f"{PREFIX}/state", RELIABLE_QOS
        )
        cls.client = cls.node.create_client(Trigger, f"/{NODE}/finalize")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        OUTPUT.unlink(missing_ok=True)
        OUTPUT2.unlink(missing_ok=True)

    def _spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _feed(self):
        scan = LaserScan()
        scan.ranges = [1.0, 2.0]
        for i in range(10):
            self.scans.publish(scan)
            state = TrackRelativeState()
            state.header.stamp.sec = 0
            state.header.stamp.nanosec = (i + 1) * 10_000_000
            state.s = 0.1 * i
            state.d = 0.0
            self.states.publish(state)
            self._spin(0.02)

    def _finalize(self):
        self.assertTrue(
            self.client.wait_for_service(timeout_sec=TIMEOUT_SECONDS)
        )
        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=TIMEOUT_SECONDS
        )
        self.assertTrue(future.done())
        return future.result()

    def test_metrics_1030_finalize_publishes_once_and_writes_json(self):
        self._feed()
        self.assertEqual(self.received, [])

        first = self._finalize()
        self.assertTrue(first.success, first.message)
        self.assertEqual(first.message, str(OUTPUT))
        self._spin(1.0)
        self.assertEqual(len(self.received), 1)
        self.assertFalse(self.received[0].lap_completed)
        self.assertEqual(self.received[0].lap_time, 0.0)
        document = json.loads(OUTPUT.read_text(encoding="utf-8"))
        self.assertFalse(document["lap_completed"])
        self.assertEqual(document["lap_time"], 0.0)

        second = self._finalize()
        self.assertTrue(second.success, second.message)
        self._spin(1.0)
        self.assertEqual(len(self.received), 1)

    def test_metrics_2010_laps_two_publishes_both_laps_once(self):
        """METRICS-2010: with ``laps:=2`` nothing is published after the first
        lap; on the second ``laps_completed`` is 2 and ``lap_times`` holds both
        durations (the first lap's time is ``lap_time``). ``laps:=0`` is
        refused at start-up."""
        received = []
        self.node.create_subscription(
            ScenarioMetrics, f"{PREFIX2}/metrics", received.append, METRICS_QOS
        )
        scans = self.node.create_publisher(
            LaserScan, f"{PREFIX2}/scan", qos_profile_sensor_data
        )
        states = self.node.create_publisher(
            TrackRelativeState, f"{PREFIX2}/state", RELIABLE_QOS
        )
        scan = LaserScan()
        scan.ranges = [1.0, 2.0]
        for _ in range(20):  # let discovery settle, then give it a clearance
            scans.publish(scan)
            self._spin(0.05)
        step = CIRCLE_LENGTH / 8.0
        counts = []
        for i in range(1, 2 * 8 + 3):
            scans.publish(scan)
            state = TrackRelativeState()
            state.header.stamp.sec = 1 + i // 100
            state.header.stamp.nanosec = (i % 100) * 10_000_000
            state.s = (step * i) % CIRCLE_LENGTH
            states.publish(state)
            self._spin(0.03)
            counts.append(len(received))
        self._spin(0.5)
        self.assertEqual(len(received), 1, counts)
        self.assertEqual(max(counts[:8]), 0, counts)  # silent after lap 1
        message = received[0]
        self.assertTrue(message.lap_completed)
        self.assertEqual(message.laps_completed, 2)
        self.assertEqual(len(message.lap_times), 2)
        self.assertAlmostEqual(message.lap_times[0], message.lap_time)
        self.assertGreater(min(message.lap_times), 0.0)

        refused = subprocess.run(
            [
                "ros2",
                "run",
                "racing_metrics",
                "racing_metrics_node",
                "--ros-args",
                "-p",
                f"track_path:={TRACK}",
                "-p",
                "laps:=0",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertNotEqual(refused.returncode, 0)
        self.assertIn("laps must be at least 1", refused.stderr)
