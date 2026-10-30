"""Tier 2: front end B against the devkit stand-in (SIMAD-2200 to 2260).

B (`racing_sim_autodrive.devkit_node`) and `racing_sim_autodrive.devkit_standin`
run as processes on one ROS graph (this file's devkit domain is
`_devkit_launch.devkit_domain()`; step 7 moves it to `S+1`). Names: stack side
under `/test_simad_devkit`, devkit side under `/test_simad_devkit_dk<S>`.
Needs ROS_DOMAIN_ID set by the caller. Plain pytest: each test starts its own
B and stand-in, which a launch_test file cannot restart per test.
"""

import math
import re
import sys
import time
from pathlib import Path

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racing_interfaces.srv import Reset
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent))

import _devkit_launch as dl  # noqa: E402

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
FRAMES = 120
BRIDGE_UP = "devkit bridge on"


@pytest.fixture(scope="module")
def observer():
    rclpy.init()
    node = rclpy.create_node("simad_devkit_observer")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def spin(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def spin_until(node, done, timeout_s, what):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if done():
            return
    raise AssertionError(f"timed out: {what}")


@pytest.fixture
def run(tmp_path, observer):
    """Start B; the test starts the stand-in. Everything is stopped after."""
    started = []

    def go(*standin, bridge_parameters=None, replies="replies.jsonl"):
        bridge, blog = dl.start_bridge(tmp_path, **(bridge_parameters or {}))
        started.append(bridge)
        dl.wait_for_log(bridge, blog, BRIDGE_UP)
        spin(observer, 1.0)  # discovery
        standin, out, slog = dl.start_standin(
            tmp_path, *standin, replies=replies
        )
        started.append(standin)
        return bridge, blog, standin, out, slog

    yield go
    for process in reversed(started):
        dl.stop(process, timeout_s=10.0)


def test_simad_2200_every_contract_topic_flows_from_the_devkit(run, observer):
    """SIMAD-2200: B against the stand-in on 120 frames of the stationary
    opening publishes `/scan` (frame `lidar`, 1081 beams), `/ground_truth/scan`,
    `/ground_truth/odom` at the centre of gravity (z = h, the rear axle moved
    lr along the heading), `/odom` and `/imu`; the `/scan` count is at least
    98 % of frames - 1 (SIMAD-2020's bound; the first burst teaches the
    assembler its offsets and is not published)."""
    scans, truth = [], []
    observer.create_subscription(
        LaserScan, dl.topic("/scan"), scans.append, SENSOR
    )
    observer.create_subscription(
        Odometry, dl.topic("/ground_truth/odom"), truth.append, STATE
    )
    imus, odoms = [], []
    from sensor_msgs.msg import Imu

    observer.create_subscription(Imu, dl.topic("/imu"), imus.append, SENSOR)
    observer.create_subscription(
        Odometry, dl.topic("/odom"), odoms.append, STATE
    )
    spin(observer, 0.5)
    _, _, standin, _, _ = run(
        *dl.STATIONARY, "--pace", "recorded", "--count", str(FRAMES)
    )
    spin_until(
        observer, lambda: standin.poll() is not None, 60.0, "stand-in done"
    )
    spin(observer, 1.0)
    assert len(scans) >= int(0.98 * (FRAMES - 1)), len(scans)
    assert len(scans) <= FRAMES - 1
    assert len(scans[0].ranges) == 1081
    assert scans[0].header.frame_id == "lidar"
    assert len(truth) >= int(0.98 * (FRAMES - 1)) and len(odoms) > 0
    assert len(imus) > 0
    from racing_sim_autodrive import core, devkit_standin, replay

    cog, rear = [], []
    for _, raw in replay.load_frames(dl.FIXTURE)[1:41]:
        frame = core.frame_from_devkit(
            devkit_standin.devkit_sample(devkit_standin.devkit_values(raw))
        )
        pose = core.frame_pose(frame)
        moved = core.rear_axle_to_cog(pose, dl.VEHICLE["lr"])
        cog.append((moved.x, moved.y))
        rear.append((pose.x, pose.y))
    last = truth[-1].pose.pose.position
    assert last.z == pytest.approx(dl.VEHICLE["h"], abs=1e-6)
    near_cog = min(math.hypot(last.x - x, last.y - y) for x, y in cog)
    near_rear = min(math.hypot(last.x - x, last.y - y) for x, y in rear)
    assert near_cog < 2e-3, near_cog  # one of the loop's frames, at the CoG
    assert near_rear > 0.1, near_rear  # and not the rear axle (lr = 0.155)
    stamps = [
        m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in truth
    ]
    assert all(b > a for a, b in zip(stamps, stamps[1:], strict=False))


def test_simad_2210_drive_reaches_the_devkit_latch(run, observer):
    """SIMAD-2210: `/drive` 2.0 m/s, 0.1 rad reaches the stand-in's latched
    reply (throttle > 0, steering = 0.1 / max steering angle), and B's
    endpoints toward the devkit are RELIABLE and VOLATILE."""
    publisher = observer.create_publisher(
        AckermannDriveStamped, dl.topic("/drive"), STATE
    )
    message = AckermannDriveStamped()
    message.drive.speed = 2.0
    message.drive.steering_angle = 0.1
    _, _, standin, out, _ = run(
        *dl.STATIONARY, "--pace", "recorded", "--count", "150"
    )
    end = time.monotonic() + 4.0
    while time.monotonic() < end:
        publisher.publish(message)
        spin(observer, 0.05)
    got = dl.replies(out)
    assert got, "the stand-in wrote no reply"
    maximum = dl.VEHICLE["max_steering_angle"]
    assert float(got[-1]["V1 Throttle"]) > 0.0
    assert float(got[-1]["V1 Steering"]) == pytest.approx(
        0.1 / maximum, abs=2e-3
    )
    for name in ("throttle_command", "steering_command"):
        infos = observer.get_publishers_info_by_topic(f"{dl.NAMESPACE}/{name}")
        assert infos, name
        for info in infos:
            assert info.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert info.qos_profile.durability == DurabilityPolicy.VOLATILE
    for name in ("lidar", "lap_time"):
        infos = observer.get_subscriptions_info_by_topic(
            f"{dl.NAMESPACE}/{name}"
        )
        assert infos, name
        for info in infos:
            assert info.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert info.qos_profile.durability == DurabilityPolicy.VOLATILE


def test_simad_2220_reset_service_gives_one_true_run_then_false(run, observer):
    """SIMAD-2220: `~/reset` makes B publish `reset_command` True in one
    contiguous run and then False (ruling E7 b: True is held until the
    session sees the reset's effect), and the stand-in's reply stream, the
    only place a True overwritten by a later False would show, carries
    `Reset True` in one contiguous run (at least one reply) and then False."""
    seen = []
    observer.create_subscription(
        Bool,
        dl.RESET_TOPIC,
        lambda m: seen.append(bool(m.data)),
        QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    _, _, standin, out, _ = run(
        *dl.STATIONARY, "--pace", "recorded", "--count", "400"
    )
    client = observer.create_client(Reset, f"{dl.PREFIX}/racing_sim/reset")
    assert client.wait_for_service(timeout_sec=10.0)
    spin(observer, 1.5)
    future = client.call_async(Reset.Request(seed=0, scenario_id=""))
    spin_until(observer, future.done, 5.0, "reset response")
    assert future.result().success
    spin_until(
        observer, lambda: standin.poll() is not None, 60.0, "stand-in done"
    )
    spin(observer, 0.5)
    assert seen[:1] == [True], seen
    assert seen.count(True) == 1 and seen[-1] is False, seen
    flags = [r["V1 Reset"] for r in dl.replies(out)]
    assert "True" in flags, "the devkit never replied Reset True"
    first = flags.index("True")
    last = len(flags) - 1 - flags[::-1].index("True")
    assert set(flags[first : last + 1]) == {"True"}, "True run not contiguous"
    assert flags[last + 1 :] and set(flags[last + 1 :]) == {"False"}


def test_simad_2240_every_name_is_under_a_test_prefix(run, observer):
    """SIMAD-2240: B and the stand-in publish, subscribe and serve only
    names under `/test_simad_devkit/` (stack side) and
    `/test_simad_devkit_dk<S>/` (devkit side), matched with the trailing `/`
    (gotcha 39); `/rosout` and `/parameter_events` are on every node."""
    _, _, standin, _, _ = run(
        *dl.STATIONARY, "--pace", "recorded", "--count", "200"
    )
    allowed = {"/rosout", "/parameter_events"}
    prefixes = (f"{dl.PREFIX}/", f"{dl.DEVKIT_PREFIX}/")
    nodes = [
        ("racing_sim", dl.PREFIX),
        ("racing_sim_devkit_link", dl.PREFIX),
        ("autodrive_bridge", "/"),
    ]
    outside = set()
    found = set()
    end = time.monotonic() + 15.0
    while time.monotonic() < end and len(found) < len(nodes):
        spin(observer, 0.2)
        for name, namespace in nodes:
            try:
                names = [
                    n
                    for getter in (
                        observer.get_publisher_names_and_types_by_node,
                        observer.get_subscriber_names_and_types_by_node,
                    )
                    for n, _ in getter(name, namespace)
                ]
            except Exception:
                continue
            found.add(name)
            outside |= {
                n
                for n in names
                if n not in allowed and not n.startswith(prefixes)
            }
    assert found == {n for n, _ in nodes}, found
    assert not outside, sorted(outside)


def test_simad_2260_a_withheld_topic_is_named_in_one_stall(run, observer):
    """SIMAD-2260: the stand-in withholds `collision_count`: no frame ever
    completes, and B logs one stall warning that names that topic and no
    other."""
    _, blog, standin, _, _ = run(
        *dl.STATIONARY,
        "--pace",
        "recorded",
        "--count",
        "150",
        "--withhold",
        "collision_count",
    )
    spin_until(
        observer, lambda: standin.poll() is not None, 60.0, "stand-in done"
    )
    spin(observer, 1.5)
    stalls = re.findall(
        r"stall: .*silent devkit topics: ([^\n]*)",
        blog.read_text(errors="replace"),
    )
    assert len(stalls) == 1, blog.read_text(errors="replace")[-800:]
    assert stalls[0].strip() == "collision_count", stalls
