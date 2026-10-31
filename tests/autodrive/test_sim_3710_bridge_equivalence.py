"""SIM-3710: B and A publish the same contract from the same recorded frames
(#44 part 9, spec "same contract, same commands").

The recorded stream goes through the REAL devkit container (replay -> devkit
bridge -> its 13 ROS topics -> B) and, separately, straight into A. The
contract topics are compared pairwise by order: equal counts, values within
1e-4 (stamps excluded: A stamps on arrival, B takes the devkit's). Frames
1..40 warm both up (B has published before recording starts), 41..448 are
compared (the fixture's reset is at 449).

Commands (plan P8 / E5, provisional): `/drive` holds speed 0 and steers 0.3
from the odometry message that corresponds to frame 200, on both. Both
bridges must reply throttle 0 and the same steering within 5e-4 (the devkit
rounds to 3 dp, A formats 4 dp); B's steering is A's delayed by a lag of at
most 3 frames, and the lag distribution is printed. Full throttle equality is
tier 1 (SIMAD-1470): two wall-clock runs never share a receipt dt.

The stack domain must carry no `roboracer_1` frame (E1-A): the devkit's /tf
stays on its own domain.

Run: see conftest.py.
"""

import time
from collections import defaultdict

import _compare
import conftest as c
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racing_interfaces.msg import TrackRelativeState
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5000,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
SENSOR = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5000,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
TOPICS = {
    "gt_odom": (Odometry, "/ground_truth/odom"),
    "gt_scan": (LaserScan, "/ground_truth/scan"),
    "imu": (Imu, "/imu"),
    "encoders": (JointState, "/racing_sim/wheel_encoders"),
    "track": (TrackRelativeState, "/ground_truth/track_relative_state"),
}
STEP_AT = 200 - 40  # odometry index in the compared window
TOLERANCE = 1e-4


def _steering(replies):
    return [float(r["V1 Steering"]) for r in replies]


def test_sim_3710_b_equals_a_on_contract_and_commands(devkit, tmp_path):
    rclpy.init()
    node = rclpy.create_node("sim_3710_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    recording = {c.PREFIX_A: False, c.PREFIX_B: False}
    got = defaultdict(list)
    drives = {}
    odoms = {c.PREFIX_A: 0, c.PREFIX_B: 0}
    tf_seen = []

    def drive(prefix, steering):
        message = AckermannDriveStamped()
        message.drive.speed = 0.0
        message.drive.steering_angle = steering
        drives[prefix].publish(message)

    def make(prefix, key):
        def callback(message):
            if not recording[prefix]:
                return
            got[(prefix, key)].append(message)
            if key == "gt_odom":
                odoms[prefix] += 1
                if odoms[prefix] >= STEP_AT:
                    drive(prefix, 0.3)

        return callback

    for prefix in (c.PREFIX_A, c.PREFIX_B):
        drives[prefix] = node.create_publisher(
            AckermannDriveStamped, f"{prefix}/drive", QOS
        )
        for key, (kind, name) in TOPICS.items():
            node.create_subscription(
                kind,
                f"{prefix}{name}",
                make(prefix, key),
                SENSOR if key in ("gt_scan", "imu", "encoders") else QOS,
            )
        node.create_subscription(
            TFMessage, f"{prefix}/tf", lambda m: tf_seen.append(m), QOS
        )

    node.create_subscription(TFMessage, "/tf", lambda m: tf_seen.append(m), QOS)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    port = c.free_port()
    b, b_log = c.start_node(
        tmp_path,
        "racing_sim_autodrive.devkit_node",
        c.PREFIX_B,
        "b",
        devkit_domain_id=devkit["domain"],
    )
    a, a_log = c.start_node(
        tmp_path,
        "racing_sim_autodrive.server_node",
        c.PREFIX_A,
        "a",
        port=port,
        host="127.0.0.1",
    )
    procs = [a, b]
    try:
        c.wait_for_log(b, b_log, "devkit bridge on")
        c.wait_for_log(a, a_log, "listening on")
        spin(1.5)
        replies = {}
        for prefix, url, tag in (
            (c.PREFIX_B, devkit["url"], "b"),
            (c.PREFIX_A, f"http://127.0.0.1:{port}", "a"),
        ):
            drive(prefix, 0.0)
            warm, _, _ = c.replay(
                url,
                tmp_path,
                f"{tag}_warm",
                "--start-frame",
                "1",
                "--end-frame",
                "40",
                replies=False,
            )
            procs.append(warm)
            while warm.poll() is None:
                spin(0.1)
            assert warm.returncode == 0, "warm-up replay failed"
            spin(1.5)
            recording[prefix] = True
            main, out, mlog = c.replay(
                url,
                tmp_path,
                tag,
                "--start-frame",
                "41",
                "--end-frame",
                "448",
            )
            procs.append(main)
            while main.poll() is None:
                spin(0.1)
            spin(2.0)
            recording[prefix] = False
            assert main.returncode == 0, mlog.read_text()
            replies[tag] = c.read_replies(out)
        # E1-A: the devkit's frames never reach the stack domain.
        frames = {t.child_frame_id for m in tf_seen for t in m.transforms} | {
            t.header.frame_id for m in tf_seen for t in m.transforms
        }
        assert "roboracer_1" not in frames, frames
    finally:
        for process in reversed(procs):
            c.stop(process, timeout_s=10.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    resyncs = [
        line for line in b_log.read_text().splitlines() if "resyncs=" in line
    ]
    for key in TOPICS:
        a_msgs, b_msgs = got[(c.PREFIX_A, key)], got[(c.PREFIX_B, key)]
        assert len(a_msgs) == len(b_msgs), (
            f"{key}: A {len(a_msgs)} vs B {len(b_msgs)}; B: {resyncs}"
        )
        assert a_msgs, f"{key}: nothing recorded"
        bad = _compare.first_difference(a_msgs, b_msgs, TOLERANCE)
        assert bad is None, f"{key}: first difference at {bad}; B: {resyncs}"

    # E2: one frame carries one receipt stamp (the devkit's lidar header
    # stamp) on every topic B publishes from it.
    stamps = {
        key: [
            m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            for m in got[(c.PREFIX_B, key)]
        ]
        for key in ("gt_odom", "gt_scan", "imu", "encoders")
    }
    reference = stamps["gt_scan"]
    assert len(set(reference)) == len(reference), "B repeated a stamp"
    for key, values in stamps.items():
        assert values == reference, (key, values[:3], reference[:3])

    a_steer, b_steer = _steering(replies["a"]), _steering(replies["b"])
    assert all(float(r["V1 Throttle"]) == 0.0 for r in replies["a"])
    assert all(float(r["V1 Throttle"]) == 0.0 for r in replies["b"])
    assert len(a_steer) == len(b_steer), (len(a_steer), len(b_steer))
    first_a = next(i for i, s in enumerate(a_steer) if s > 0.15)
    first_b = next(i for i, s in enumerate(b_steer) if s > 0.15)
    lag = first_b - first_a
    print(f"SIM-3710 command lag: {lag} frame(s); A steps at {first_a}")
    assert 0 <= lag <= 3, lag
    for i in range(len(a_steer) - lag):
        assert b_steer[i + lag] == pytest.approx(a_steer[i], abs=5e-4), i
    assert not any(s != 0.0 for s in b_steer[:first_b]), "B steered early"
