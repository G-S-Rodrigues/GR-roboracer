"""Tier 2: BRINGUP-2001, the teleop node turns /key_vel into a limited /drive.

The node runs as its installed executable in a namespace with `/key_vel` and
`/drive` remapped under PREFIX (gotcha #39: colcon runs other packages' tier-2
tests in parallel on one domain, and a global /drive at 50 Hz would reach
them). Plain pytest and not a launch test: the SIGINT case needs the
process's exit code, once with and once without a second SIGINT, which is a
fresh process each (the shape of racing_sim_autodrive's SIMAD-2230). Needs
ROS_DOMAIN_ID set by the caller.
"""

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TwistStamped
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
CONFIG = REPOSITORY_ROOT / "config" / "teleop" / "autodrive_keyboard.yaml"
PREFIX = "/test_teleop"
KEY_VEL = f"{PREFIX}/key_vel"
DRIVE = f"{PREFIX}/drive"
ACCELERATION = 0.8
# The server's /drive QoS (contract_node.STATE_QOS): a subscriber with it
# hears the node only if the node's publisher is compatible with it.
STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _stamp(message) -> float:
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


class _Harness:
    """The node under test plus a publisher and a listener on its names."""

    def __init__(self, tmp_path: Path) -> None:
        # A parameter file keyed by the bare node name matches only a node
        # in the root namespace; the committed values go under the test's.
        document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        params = tmp_path / "teleop.yaml"
        params.write_text(
            yaml.safe_dump(
                {
                    f"{PREFIX}/racing_bringup_teleop": document[
                        "racing_bringup_teleop"
                    ]
                }
            ),
            encoding="utf-8",
        )
        executable = (
            Path(get_package_prefix("racing_bringup"))
            / "lib"
            / "racing_bringup"
            / "racing_bringup_teleop"
        )
        assert executable.is_file(), f"no executable {executable}"
        rclpy.init()
        self.node = rclpy.create_node("bringup_2001_probe")
        self.drives: list[AckermannDriveStamped] = []
        self.node.create_subscription(
            AckermannDriveStamped, DRIVE, self.drives.append, STATE_QOS
        )
        self.publisher = self.node.create_publisher(TwistStamped, KEY_VEL, 10)
        self.process = subprocess.Popen(
            [
                str(executable),
                "--ros-args",
                "-r",
                f"__ns:={PREFIX}",
                "-r",
                f"/key_vel:={KEY_VEL}",
                "-r",
                f"/drive:={DRIVE}",
                "--params-file",
                str(params),
            ],
            start_new_session=True,
            env={**os.environ},
        )

    def spin(self, seconds: float, speed=None, angle=0.0) -> None:
        """Spin for `seconds`, publishing the key at 10 Hz when `speed` is
        given (key_teleop's rate), nothing when it is None."""
        end = time.monotonic() + seconds
        next_key = 0.0
        while time.monotonic() < end:
            if speed is not None and time.monotonic() >= next_key:
                message = TwistStamped()
                message.twist.linear.x = speed
                message.twist.angular.z = angle
                self.publisher.publish(message)
                next_key = time.monotonic() + 0.1
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for_first_drive(self) -> None:
        end = time.monotonic() + 20.0
        while not self.drives and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert self.drives, "the node published no /drive"

    def close(self) -> None:
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGKILL)
            self.process.wait(timeout=5.0)
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def harness(tmp_path):
    probe = _Harness(tmp_path)
    try:
        yield probe
    finally:
        probe.close()


def test_bringup_2001_key_vel_ramps_the_drive_and_silence_stops_it(
    harness,
) -> None:
    """BRINGUP-2001: /key_vel at speed 1.5 gives /drive at about 50 Hz whose
    speed rises by at most acceleration x dt (dt the /drive stamps'
    difference) and never passes 1.5; with the input gone the speed reaches
    0 within input_timeout + 1.5/deceleration + 0.3 s."""
    harness.wait_for_first_drive()
    first = len(harness.drives)
    started = time.monotonic()
    harness.spin(1.0, speed=1.5)
    elapsed = time.monotonic() - started
    driven = harness.drives[first:]
    rate = len(driven) / elapsed
    assert 40.0 <= rate <= 60.0, rate
    speeds = [m.drive.speed for m in driven]
    assert max(speeds) > 0.3, "the car never moved"
    assert max(speeds) <= 1.5
    stamps = [_stamp(m) for m in driven]
    for i in range(1, len(driven)):
        dt = max(stamps[i] - stamps[i - 1], 0.0)
        assert speeds[i] - speeds[i - 1] <= ACCELERATION * dt + 1e-6, (i, dt)

    stopped_at = time.monotonic()
    harness.spin(0.0)
    limit = 0.5 + 1.5 / 1.5 + 0.3
    while time.monotonic() - stopped_at < limit + 1.0:
        harness.spin(0.05)
        if harness.drives[-1].drive.speed == 0.0:
            break
    assert harness.drives[-1].drive.speed == 0.0
    assert time.monotonic() - stopped_at <= limit


@pytest.mark.parametrize("second_sigint", [False, True])
def test_bringup_2001_sigint_zeroes_the_drive_and_exits_0(
    harness, second_sigint
) -> None:
    """BRINGUP-2001 (gotcha #41, SIMAD-2230): with the car driven to a
    non-zero speed and steering, a SIGINT (and a second 0.1 s later, as the
    launch's group signal sends) exits 0 and the last /drive is speed 0 and
    steering 0."""
    harness.wait_for_first_drive()
    harness.spin(1.0, speed=1.5, angle=0.3)
    last = harness.drives[-1].drive
    assert last.speed > 0.3 and last.steering_angle > 0.1
    harness.process.send_signal(signal.SIGINT)
    if second_sigint:
        harness.spin(0.1)
        if harness.process.poll() is None:
            harness.process.send_signal(signal.SIGINT)
    deadline = time.monotonic() + 10.0
    while harness.process.poll() is None and time.monotonic() < deadline:
        harness.spin(0.05)
    assert harness.process.returncode == 0
    harness.spin(0.5)
    final = harness.drives[-1].drive
    assert final.speed == 0.0
    assert final.steering_angle == 0.0
