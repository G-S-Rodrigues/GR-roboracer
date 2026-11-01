"""Tier 2: the Socket.IO server node against the replayed recording.

SIMAD-2010..2040, 2270. The server listens on an ephemeral port (never 4567) and
`racing_sim_autodrive.replay` stands in for the sim. Needs the pip closure
(plan D2) and ROS_DOMAIN_ID set by the caller.
"""

import json
import re
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racing_interfaces.msg import SimCounters
from racing_interfaces.srv import Reset
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))

import _server_launch as launch_support  # noqa: E402

PORT = launch_support.free_port()
SCAN = launch_support.topic("/scan")
DRIVE = launch_support.topic("/drive")
ODOM = launch_support.topic("/odom")
TRUTH = launch_support.topic("/ground_truth/odom")
COUNTERS = f"{launch_support.PREFIX}/racing_sim/autodrive_counters"
RESET = f"{launch_support.PREFIX}/racing_sim/reset"
FRAMES = 300
SENSOR = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=2000,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
STATE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=2000,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
def generate_test_description():
    return launch_support.server_description(PORT)[1]


def _stamp(message) -> int:
    return message.header.stamp.sec * 1_000_000_000 + (
        message.header.stamp.nanosec
    )


class TestServerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("autodrive_server_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_server(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if (
                "racing_sim",
                launch_support.PREFIX,
            ) in self.node.get_node_names_and_namespaces():
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail("racing_sim not found")

    def _replay(self, tmp_path, *extra, publish=None, collect=()):
        """Run the replay lock-step while collecting `collect` topics.

        Returns (replies, {topic: messages}). `publish(node)` runs once
        after the subscriptions exist and before the replay starts.
        """
        self._wait_for_server()
        out = {}
        subscriptions = []
        for message_type, topic, qos in collect:
            out[topic] = []
            subscriptions.append(
                self.node.create_subscription(
                    message_type, topic, out[topic].append, qos
                )
            )
        if publish is not None:
            publish()
        # let discovery settle so the first frames are not published unheard
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        replies_file = tmp_path
        process = subprocess.Popen(
            launch_support.replay_command(
                PORT,
                "--pace",
                "lockstep",
                "--replies-out",
                str(replies_file),
                *extra,
            )
        )
        while process.poll() is None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)
        self.assertEqual(process.returncode, 0, "replay stalled or failed")
        replies = [
            json.loads(line)["reply"]
            for line in Path(replies_file).read_text().splitlines()
        ]
        return replies, out

    def test_simad_2010_every_frame_is_answered_and_scanned(self):
        """SIMAD-2010: 300 frames replayed (the lidar-less first included)
        get 300 replies; /scan carries the 299 frames that have a lidar
        (at least 98 % = 293: a best-effort topic, with a deep subscriber
        queue).

        Observed: 12 runs on 2026-10-28 (dev container, load about 8-9,
        domain 68) gave /scan counts 299, 299, 299, 299, 299, 299, 299, 299,
        299, 299, 299, 299 out of 299, a minimum of 100 %. The 98 % bound
        leaves room for the occasional best-effort drop under host load
        without being able to hide a real loss."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            replies, got = self._replay(
                Path(tmp) / "replies.jsonl",
                "--count",
                str(FRAMES),
                collect=[(LaserScan, SCAN, SENSOR)],
            )
        self.assertEqual(len(replies), FRAMES)
        self.assertGreaterEqual(len(got[SCAN]), int(0.98 * (FRAMES - 1)))
        self.assertLessEqual(len(got[SCAN]), FRAMES - 1)
        self.assertEqual(len(got[SCAN][0].ranges), 1081)
        header = (
            launch_support.REPO
            / "ros_ws/src/racing_sim_adapter/include/racing_sim_adapter"
            / "contract.hpp"
        ).read_text()
        frame = re.search(r'kLaserFrame\{"([^"]+)"\}', header).group(1)
        self.assertEqual(got[SCAN][0].header.frame_id, frame)

    def test_simad_2020_drive_commands_reach_the_reply(self):
        """SIMAD-2020: /drive 2.0 m/s, 0.1 rad -> replies carry throttle > 0
        and steering 0.1 / max_steering_angle."""
        import tempfile

        publisher = self.node.create_publisher(
            AckermannDriveStamped, DRIVE, STATE
        )

        def send():
            message = AckermannDriveStamped()
            message.drive.speed = 2.0
            message.drive.steering_angle = 0.1
            for _ in range(5):
                publisher.publish(message)
                rclpy.spin_once(self.node, timeout_sec=0.05)

        with tempfile.TemporaryDirectory() as tmp:
            replies, _ = self._replay(
                Path(tmp) / "replies.jsonl",
                "--start-frame",
                "100",
                "--count",
                "60",
                publish=send,
            )
        steering = launch_support.VEHICLE["max_steering_angle"]
        last = replies[-1]
        self.assertGreater(float(last["V1 Throttle"]), 0.0)
        self.assertAlmostEqual(
            float(last["V1 Steering"]), 0.1 / steering, places=3
        )

    def test_simad_2030_reset_service_sets_reset_true_once(self):
        """SIMAD-2030: <prefix>/racing_sim/reset -> the next reply has V1 Reset
        True, the one after False."""
        import tempfile

        self._wait_for_server()
        client = self.node.create_client(Reset, RESET)
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        future = client.call_async(Reset.Request(seed=0, scenario_id=""))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        self.assertTrue(future.result().success)
        with tempfile.TemporaryDirectory() as tmp:
            replies, _ = self._replay(
                Path(tmp) / "replies.jsonl", "--count", "5"
            )
        flags = [r["V1 Reset"] for r in replies]
        self.assertEqual(flags[:2], ["True", "False"], flags)
        self.assertNotIn("True", flags[2:])
        rejected = client.call_async(Reset.Request(seed=0, scenario_id="x"))
        rclpy.spin_until_future_complete(self.node, rejected, timeout_sec=3.0)
        self.assertFalse(rejected.result().success)

    def test_simad_2040_odom_and_truth_share_strictly_rising_stamps(self):
        """SIMAD-2040: per frame /odom and /ground_truth/odom carry one
        stamp, and stamps strictly increase (the node clock at receipt)."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            _, got = self._replay(
                Path(tmp) / "replies.jsonl",
                "--count",
                "100",
                collect=[
                    (Odometry, ODOM, STATE),
                    (Odometry, TRUTH, STATE),
                ],
            )
        odom = [_stamp(m) for m in got[ODOM]]
        truth = [_stamp(m) for m in got[TRUTH]]
        self.assertGreaterEqual(len(odom), 100)
        self.assertEqual(odom[-100:], truth[-100:])
        self.assertTrue(
            all(b > a for a, b in zip(odom, odom[1:], strict=False))
        )

    def test_simad_2060_timings_out_has_a_latency_per_sent_frame(self):
        """SIMAD-2060: `--timings-out` writes one {frame, sent_ns,
        reply_ns} row per frame sent, each reply later than its send and
        within the replay's reply wait (the figures part 8's rate bar is
        read from; the default replay writes nothing)."""
        import tempfile

        from racing_sim_autodrive import replay

        with tempfile.TemporaryDirectory() as tmp:
            timings = Path(tmp) / "timings.jsonl"
            self._replay(
                Path(tmp) / "replies.jsonl",
                "--count",
                "300",
                "--timings-out",
                str(timings),
            )
            rows = [
                json.loads(line) for line in timings.read_text().splitlines()
            ]
        self.assertEqual(len(rows), 300)
        for row in rows:
            latency = (row["reply_ns"] - row["sent_ns"]) * 1e-9
            self.assertGreater(latency, 0.0, row)
            self.assertLess(latency, replay.REPLY_WAIT_S, row)

    def test_simad_2080_every_frame_is_answered_under_a_drive_flood(self):
        """SIMAD-2080: a controller publishing /drive in a tight loop must not
        starve the Socket.IO hub. 300 frames at recorded pace (the sim's own
        pace, many frames in flight) get 300 replies.

        Why: in CI the rclpy executor thread, busy with the /drive flood,
        held the GIL and the hub's replies drained at 9-30 per second, so
        the replay ended with replies missing (the ADAPT-2110 stall). Lockstep
        cannot see it: one frame in flight, so no reply is ever late."""
        import tempfile

        publisher = self.node.create_publisher(
            AckermannDriveStamped, DRIVE, STATE
        )
        message = AckermannDriveStamped()
        message.drive.speed = 2.0
        message.drive.steering_angle = 0.1
        stop = threading.Event()

        def flood():
            while not stop.is_set():
                publisher.publish(message)

        thread = threading.Thread(target=flood, daemon=True)
        with tempfile.TemporaryDirectory() as tmp:
            thread.start()
            try:
                replies, _ = self._replay(
                    Path(tmp) / "replies.jsonl",
                    "--pace",
                    "recorded",
                    "--count",
                    str(FRAMES),
                )
            finally:
                stop.set()
                thread.join(timeout=5.0)
        self.assertEqual(len(replies), FRAMES)

    def test_simad_2090_a_silent_client_gets_one_stall_warning(
        self, proc_output
    ):
        """SIMAD-2090: 20 frames, then the client holds the socket open and
        silent for 1.5 s: the server logs the stall with its counters
        (entered = exited = emitted: every frame it took it answered; the
        counts are cumulative over the server's life, shared by this file's
        tests) and dumps the stacks of every thread, so a stuck server is
        never silent (gotcha 29)."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            self._replay(
                Path(tmp) / "replies.jsonl",
                "--count",
                "20",
                "--hold-s",
                "1.5",
            )
        proc_output.assertWaitFor(
            "stall: no Bridge frame handled for", timeout=5
        )
        proc_output.assertWaitFor(
            re.compile(r"entered=(\d+) exited=\1 emitted=\1"), timeout=5
        )
        proc_output.assertWaitFor("Current thread", timeout=5)

    def test_simad_2270_the_sims_counters_are_published_per_frame(self):
        """SIMAD-2270: `<node>/autodrive_counters` carries the sim's own lap and
        collision counters, one message per frame, in order. The fixture's
        collisions read 0, then 1 at frame 82, 2 at 131, and 0 after the
        reset at 451; its lap count is 0 throughout (it never completes a
        lap). A debug and cross-check topic only (league rules restrict the
        counters at race time): no stack node subscribes to it."""
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            _, got = self._replay(
                Path(tmp) / "replies.jsonl",
                "--count",
                "460",
                collect=[(SimCounters, COUNTERS, STATE)],
            )
        counters = got[COUNTERS]
        self.assertEqual(len(counters), 460)
        collisions = [m.collisions for m in counters]
        self.assertEqual(collisions[0], 0)
        self.assertEqual(collisions[81:83], [0, 1])
        self.assertEqual(collisions[130:132], [1, 2])
        self.assertEqual(collisions[450:452], [2, 0])
        self.assertEqual({m.lap_count for m in counters}, {0})
        self.assertEqual(counters[0].last_lap_time, float("inf"))
        self.assertAlmostEqual(counters[82].lap_time, 5.6838, places=3)
        self.assertTrue(
            all(m.header.stamp.sec or m.header.stamp.nanosec for m in counters)
        )
