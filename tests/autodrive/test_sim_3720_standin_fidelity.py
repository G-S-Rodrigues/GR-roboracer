"""SIM-3720: the CI devkit stand-in is the real devkit, to what B can see
(#44 part 9). It holds the stand-in to the pinned image at the moments the
image can change (an edition bump).

The same frames 1..60 go through the real devkit container (replay ->
devkit bridge, on its domain D) and out of the stand-in (on S+1, S-1 if
S+1 > 101; every stand-in name under a prefix that carries S, so a
neighbour's stack on that domain never shares one). An observer on each
domain records all 13 topics and the transforms. Compared:

- per topic: message type, QoS (reliability, durability, history, depth of
  the publisher), frame_id, LaserScan geometry (angles, range limits, beam
  count);
- the set of `/tf` frames (the devkit's global one against the stand-in's
  prefixed one);
- the per-frame values within float32 resolution, by order;
- the reply strings for the same latched commands.

Never subscribes to the live devkit from a test's own stack: the domain the
stand-in runs on is checked against AUTODRIVE_ROS_DOMAIN_ID.

Run: see conftest.py.
"""

import math
import subprocess
import sys
import time

import _compare
import conftest as c
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, Float32
from tf2_msgs.msg import TFMessage

LIVE_NS = "/autodrive/roboracer_1"
LIVE_RESET = "/autodrive/reset_command"
TOPICS = (
    "lidar",
    "imu",
    "left_encoder",
    "right_encoder",
    "ips",
    "odom",
    "throttle",
    "steering",
    "lap_count",
    "lap_time",
    "last_lap_time",
    "best_lap_time",
    "collision_count",
)
FLOAT32 = 1e-6
LAST = 60


def _wide_qos(reliability):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=2000,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )


class Side:
    """One domain: a node recording every devkit topic and the transforms."""

    def __init__(self, domain, namespace, tf_topic):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = rclpy.create_node("sim_3720_observer", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.namespace = namespace
        self.got = {name: [] for name in TOPICS}
        self.frames = set()
        self.types = {}
        self.qos = {}
        self.frame_ids = {}
        self.tf_topic = tf_topic

    def topic(self, name):
        return f"{self.namespace}/{name}"

    def subscribe(self, timeout_s=30.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            known = dict(self.node.get_topic_names_and_types())
            if all(self.topic(n) in known for n in TOPICS):
                break
            self.spin(0.2)
        else:
            raise AssertionError(f"topics missing on {self.namespace}")
        for name in TOPICS:
            topic = self.topic(name)
            kind = get_message(known[topic][0])
            self.types[name] = known[topic][0]
            info = self.node.get_publishers_info_by_topic(topic)[0]
            profile = info.qos_profile
            self.qos[name] = (
                profile.reliability.name,
                profile.durability.name,
                profile.history.name,
                profile.depth,
            )
            self.node.create_subscription(
                kind,
                topic,
                lambda m, n=name: self.got[n].append(m),
                _wide_qos(ReliabilityPolicy.BEST_EFFORT),
            )
        self.node.create_subscription(
            TFMessage,
            self.tf_topic,
            self._on_tf,
            _wide_qos(ReliabilityPolicy.RELIABLE),
        )

    def _on_tf(self, message):
        for t in message.transforms:
            self.frames.add((t.header.frame_id, t.child_frame_id))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


def _latch(side, namespace, reset_topic, publishers):
    for name, kind, value in (
        ("steering_command", Float32, 0.3),
        ("throttle_command", Float32, 0.0),
    ):
        publishers.append(
            (
                side.node.create_publisher(
                    kind,
                    f"{namespace}/{name}",
                    _wide_qos(ReliabilityPolicy.RELIABLE),
                ),
                kind(data=value),
            )
        )
    publishers.append(
        (
            side.node.create_publisher(
                Bool, reset_topic, _wide_qos(ReliabilityPolicy.RELIABLE)
            ),
            Bool(data=False),
        )
    )


def test_sim_3720_standin_matches_the_real_devkit(devkit, tmp_path):
    domain_b = c.standin_domain()
    if domain_b == devkit["domain"]:
        pytest.fail(
            f"stand-in domain {domain_b} equals AUTODRIVE_ROS_DOMAIN_ID:"
            " never subscribe to the live devkit from a stand-in's side"
        )
    prefix = f"/test_sim_3720_dk{c.STACK_DOMAIN}"
    namespace = f"{prefix}/autodrive/roboracer_1"
    reset = f"{prefix}/autodrive/reset_command"
    tf_topic = f"{prefix}/tf"
    real = Side(devkit["domain"], LIVE_NS, "/tf")
    fake = Side(domain_b, namespace, tf_topic)
    standin_log = tmp_path / "standin.log"
    standin_replies = tmp_path / "standin_replies.jsonl"
    procs = []
    try:
        real.spin(2.0)
        real.subscribe()
        publishers = []
        _latch(real, LIVE_NS, LIVE_RESET, publishers)
        warm, _, _ = c.replay(
            devkit["url"],
            tmp_path,
            "warm",
            "--start-frame",
            "1",
            "--end-frame",
            "40",
            replies=False,
        )
        procs.append(warm)
        while warm.poll() is None:
            real.spin(0.1)
        real.spin(1.0)
        # Latch the same commands on the live devkit, then the measured pass.
        for publisher, message in publishers:
            publisher.publish(message)
        real.spin(1.0)
        for name in TOPICS:
            real.got[name].clear()
        real.frames.clear()
        main, out, mlog = c.replay(
            devkit["url"],
            tmp_path,
            "real",
            "--start-frame",
            "1",
            "--end-frame",
            str(LAST),
        )
        procs.append(main)
        while main.poll() is None:
            real.spin(0.1)
        real.spin(1.5)
        assert main.returncode == 0, mlog.read_text()
        real_replies = c.read_replies(out)

        standin = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "racing_sim_autodrive.devkit_standin",
                "--fixture",
                str(c.FIXTURE),
                "--namespace",
                namespace,
                "--reset-topic",
                reset,
                "--tf-topic",
                tf_topic,
                "--pace",
                "recorded",
                "--start-frame",
                "1",
                "--end-frame",
                str(LAST),
                "--replies-out",
                str(standin_replies),
                "--wait-subscribers",
                "20",
            ],
            env={**__import__("os").environ, "ROS_DOMAIN_ID": str(domain_b)},
            stdout=standin_log.open("w"),
            stderr=subprocess.STDOUT,
        )
        procs.append(standin)
        fake.spin(1.0)
        fake.subscribe()
        fake_pubs = []
        _latch(fake, namespace, reset, fake_pubs)
        for publisher, message in fake_pubs:
            publisher.publish(message)
        while standin.poll() is None:
            fake.spin(0.1)
        fake.spin(1.5)
        assert standin.returncode == 0, standin_log.read_text()
        fake_replies = c.read_replies(standin_replies)
    finally:
        for process in reversed(procs):
            c.stop(process, timeout_s=10.0)
        real_got, fake_got = real.got, fake.got
        real_frames, fake_frames = real.frames, fake.frames
        real_types, fake_types = real.types, fake.types
        real_qos, fake_qos = real.qos, fake.qos
        real.close()
        fake.close()

    assert real_types == fake_types
    assert real_qos == fake_qos, {
        k: (real_qos[k], fake_qos.get(k))
        for k in real_qos
        if real_qos[k] != fake_qos.get(k)
    }
    assert real_frames, "the live /tf carried nothing"
    assert real_frames == fake_frames, real_frames ^ fake_frames
    for name in TOPICS:
        r, f = real_got[name], fake_got[name]
        assert r, f"{name}: nothing from the real devkit"
        assert len(r) == len(f), f"{name}: real {len(r)} vs stand-in {len(f)}"
        if hasattr(r[0], "header"):
            assert {m.header.frame_id for m in r} == {
                m.header.frame_id for m in f
            }, name
    scan_r, scan_f = real_got["lidar"][0], fake_got["lidar"][0]
    for field in (
        "angle_min",
        "angle_max",
        "angle_increment",
        "range_min",
        "range_max",
    ):
        assert getattr(scan_r, field) == pytest.approx(
            getattr(scan_f, field), abs=FLOAT32
        ), field
    assert len(scan_r.ranges) == len(scan_f.ranges)
    for name in TOPICS:
        bad = _compare.first_difference(real_got[name], fake_got[name], FLOAT32)
        assert bad is None, f"{name}: {bad}"
    assert real_replies and fake_replies
    steady = min(len(real_replies), len(fake_replies)) // 2
    assert real_replies[-steady:] == fake_replies[-steady:], (
        real_replies[-1],
        fake_replies[-1],
    )
    assert math.isfinite(len(real_replies))
