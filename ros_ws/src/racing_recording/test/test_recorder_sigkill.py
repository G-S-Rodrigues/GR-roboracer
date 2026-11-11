"""REC-2010: a SIGKILLed recorder leaves only whole, parseable lines.

Launch escalates a node that ignores SIGINT and SIGTERM to SIGKILL, and
SIGKILL skips the destructor. The nightly of 2026-11-05 lost the tail of a
SIM-3510 recording that way (``JSONDecodeError`` at char 271, one Odometry
record less its closing ``}``). Each record must be on disk, whole, as soon as
``record()`` returns, so what a killed node leaves still parses.

Every node and topic is renamed under ``/test_rec``: colcon runs other
packages' tier-2 tests in parallel on one domain (gotcha 39). The guard below
fails if a later subscription in ``node.cpp`` escapes the prefix.
"""

import json
import os
import signal
import subprocess
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import NodeNameNonExistentError
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

PREFIX = "/test_rec"
NODE = "rec_2010"
OUTPUT = Path(f"/tmp/test_rec2010_{time.time_ns()}.jsonl")
EVENTS = 300
HISTORY_DEPTH = 10  # the subscription's queue: what a kill may lose
TIMEOUT_SECONDS = 10.0
INFRASTRUCTURE_TOPICS = frozenset({"/rosout", "/parameter_events"})

# node.cpp subscribes these six on absolute names; each is remapped.
ABSOLUTE_TOPICS = (
    "/drive",
    "/ground_truth/track_relative_state",
    "/safety/status",
    "/scenario/metrics",
    "/ground_truth/odom",
    "/evaluation/localization_error",
)
ODOM = f"{PREFIX}/odom"
ODOM_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=HISTORY_DEPTH,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
def generate_test_description():
    # The recorder is started by the test itself (subprocess), not by launch:
    # launch_testing ends the run when a process under test dies, and this
    # test kills it on purpose.
    # launch_testing ends a run with no live process at once, so one idle
    # process (a `sleep`, not a ROS node) holds it open.
    return launch.LaunchDescription(
        [
            launch.actions.ExecuteProcess(cmd=["sleep", "120"]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _recorder_command():
    executable = (
        Path(get_package_prefix("racing_recording"))
        / "lib"
        / "racing_recording"
        / "racing_recording_node"
    )
    remaps = []
    for topic in ABSOLUTE_TOPICS:
        target = ODOM if topic == "/ground_truth/odom" else f"{PREFIX}{topic}"
        remaps += ["-r", f"{topic}:={target}"]
    return [
        str(executable),
        "--ros-args",
        "-r",
        f"__node:={NODE}",
        "-r",
        f"__ns:={PREFIX}",
        "-p",
        f"output_path:={OUTPUT}",
        *remaps,
    ]


def _data_topics(node, name, namespace, timeout=TIMEOUT_SECONDS):
    """Topics a node publishes or subscribes, minus the two every rcl node
    holds (the shape of racing_evaluation's graph_isolation, EVAL-2030)."""
    deadline = time.monotonic() + timeout
    found: set[str] = set()
    stable_since = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            pubs = node.get_publisher_names_and_types_by_node(name, namespace)
            subs = node.get_subscriber_names_and_types_by_node(name, namespace)
        except NodeNameNonExistentError:
            stable_since = None
            continue
        current = {topic for topic, _ in pubs + subs} - INFRASTRUCTURE_TOPICS
        if current != found or stable_since is None:
            found = current
            stable_since = time.monotonic()
        elif time.monotonic() - stable_since >= 0.5:
            break
    return found


class TestRecorderSigkill(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("rec2010_test", namespace=PREFIX)
        cls.process = subprocess.Popen(_recorder_command())

    @classmethod
    def tearDownClass(cls):
        cls.process.kill()
        cls.process.wait()
        cls.node.destroy_node()
        rclpy.shutdown()
        OUTPUT.unlink(missing_ok=True)

    def test_rec_2010_sigkill_leaves_only_parseable_lines(self):
        publisher = self.node.create_publisher(Odometry, ODOM, ODOM_QOS)

        # Gotcha 39: nothing of the recorder's or of this node's may sit
        # outside the prefix, or it leaks onto the shared domain.
        for name in (NODE, "rec2010_test"):
            topics = _data_topics(self.node, name, PREFIX)
            outside = sorted(
                t for t in topics if not t.startswith(PREFIX + "/")
            )
            self.assertEqual(outside, [], f"{name}: topics outside {PREFIX}")
            self.assertTrue(topics, f"{name}: no topics discovered")

        deadline = time.monotonic() + TIMEOUT_SECONDS
        while (
            publisher.get_subscription_count() < 1
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertGreaterEqual(publisher.get_subscription_count(), 1)

        for index in range(EVENTS):
            message = Odometry()
            message.header.stamp.sec = 1 + index // 100
            message.header.stamp.nanosec = (index % 100) * 10_000_000 + 1
            message.header.frame_id = "map"
            message.child_frame_id = "base_link"
            message.pose.pose.position.x = 0.01 * index
            publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.01)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        os.kill(self.process.pid, signal.SIGKILL)
        # Not assertExitCodes: the kill is the point. Assert it happened, so
        # the file below was read after a SIGKILL and not after a clean exit.
        self.assertEqual(
            self.process.wait(timeout=TIMEOUT_SECONDS), -signal.SIGKILL
        )

        lines = OUTPUT.read_text(encoding="utf-8").splitlines()
        records = [json.loads(line) for line in lines]  # every line parses
        self.assertEqual(records[0]["record_type"], "manifest")
        events = [r for r in records if r["record_type"] == "event"]
        self.assertGreaterEqual(len(events), EVENTS - HISTORY_DEPTH)
