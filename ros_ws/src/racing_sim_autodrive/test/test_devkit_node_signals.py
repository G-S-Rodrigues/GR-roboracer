"""Tier 2: SIMAD-2230, B zeroes the devkit's commands on SIGINT and exits 0.

The devkit replays the last command it was given forever, so a B that dies
mid-drive leaves the car driving. B owns SIGINT: it publishes throttle 0 and
steering 0 and waits for the acks before it shuts any context down. Needs
ROS_DOMAIN_ID set by the caller.
"""

import sys
import time
from pathlib import Path

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))

import _devkit_launch as dl  # noqa: E402

STATE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


@pytest.mark.parametrize("second_sigint", [False, True])
def test_simad_2230_sigint_mid_drive_zeroes_the_latch_and_exits_0(
    tmp_path, second_sigint
):
    """SIMAD-2230: with B driving (the stand-in's latched throttle and
    steering both non-zero first, so a B that never drove cannot pass), a
    SIGINT (and a second 0.1 s later, as the launch's group signal sends)
    exits 0 and the stand-in's next replies carry throttle 0.0 and
    steering 0.0."""
    rclpy.init()
    node = rclpy.create_node("simad_2230_driver")
    publisher = node.create_publisher(
        AckermannDriveStamped, dl.topic("/drive"), STATE
    )
    message = AckermannDriveStamped()
    message.drive.speed = 2.0
    message.drive.steering_angle = 0.1
    bridge, blog = dl.start_bridge(tmp_path)
    standin = None
    try:
        dl.wait_for_log(bridge, blog, "devkit bridge on")
        standin, out, _ = dl.start_standin(
            tmp_path, *dl.STATIONARY, "--pace", "recorded", "--count", "900"
        )

        def spin(seconds, drive=True):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if drive:
                    publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.02)

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            spin(0.2)
            rows = dl.replies(out)
            if (
                rows
                and float(rows[-1]["V1 Throttle"]) != 0.0
                and float(rows[-1]["V1 Steering"]) != 0.0
            ):
                break
        rows = dl.replies(out)
        assert float(rows[-1]["V1 Throttle"]) != 0.0, rows[-1]
        assert float(rows[-1]["V1 Steering"]) != 0.0, rows[-1]
        before = len(rows)
        bridge.send_signal(2)
        if second_sigint:
            time.sleep(0.1)
            if bridge.poll() is None:
                bridge.send_signal(2)
        code = bridge.wait(timeout=20.0)
        assert code == 0, (code, blog.read_text(errors="replace")[-1500:])
        spin(1.5, drive=False)
        after = dl.replies(out)
        assert len(after) > before + 5, "the stand-in stopped replying"
        assert float(after[-1]["V1 Throttle"]) == 0.0, after[-1]
        assert float(after[-1]["V1 Steering"]) == 0.0, after[-1]
    finally:
        dl.stop(bridge, 9, 5.0)
        if standin is not None:
            dl.stop(standin, 2, 10.0)
        node.destroy_node()
        rclpy.shutdown()
