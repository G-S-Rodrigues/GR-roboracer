"""PP-2010: the Pure Pursuit node's wiring, through real topics.

The gtest (PP1010-1150) covers the core; this covers the node around it, the
two seams a core test cannot see:

* `/trajectory`'s per-point `v` reaches `/controller/drive` only under
  `speed_source: trajectory`; the default ignores it.
* `/odom`'s twist reaches the lookahead only under `lookahead_gain > 0`.

Every node here is renamed and every topic remapped to a `/test_pp2010/` name:
colcon runs other packages' tier-2 tests in parallel on one domain, and
racing_sim_adapter and racing_evaluation publish `/odom`.
"""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racing_interfaces.msg import Trajectory, TrajectoryPoint
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

PREFIX = "/test_pp2010"
TRAJECTORY_V = 2.2
TIMEOUT_SECONDS = 10.0

RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# node.cpp subscribes /trajectory reliable + transient_local, depth 1.
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _controller(name, odom, trajectory, drive, **parameters):
    return launch_ros.actions.Node(
        package="racing_controller_baseline",
        executable="racing_controller_baseline_node",
        name=name,
        parameters=[{"closed_trajectory": False, **parameters}],
        remappings=[
            ("/odom", odom),
            ("/trajectory", trajectory),
            ("/controller/drive", drive),
        ],
        output="screen",
    )


@pytest.mark.launch_test
def generate_test_description():
    straight = (
        f"{PREFIX}/odom",
        f"{PREFIX}/trajectory",
    )
    bend = (f"{PREFIX}/bend_odom", f"{PREFIX}/bend_trajectory")
    return launch.LaunchDescription(
        [
            _controller("pp_default", *straight, f"{PREFIX}/drive_default"),
            _controller(
                "pp_trajectory",
                *straight,
                f"{PREFIX}/drive_trajectory",
                speed_source="trajectory",
                maximum_speed=3.0,
            ),
            _controller("pp_fixed", *bend, f"{PREFIX}/drive_fixed"),
            _controller(
                "pp_gain", *bend, f"{PREFIX}/drive_gain", lookahead_gain=1.0
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _trajectory(points) -> Trajectory:
    message = Trajectory()
    message.header.frame_id = "map"
    message.source = "pp2010"
    for index, (x, y) in enumerate(points):
        point = TrajectoryPoint()
        point.x = float(x)
        point.y = float(y)
        point.v = TRAJECTORY_V
        point.s = float(index)
        message.points.append(point)
    return message


def _straight_points():
    # 10 m along x, 0.5 m apart.
    return [(0.5 * i, 0.0) for i in range(21)]


def _bend_points():
    # Straight along x to 2.2 m, then a quarter circle of radius 1.0 m to
    # the left about (2.2, 1.0), 0.1 m apart.
    line = [(0.1 * i, 0.0) for i in range(22)]
    arc = [
        (2.2 + math.sin(0.1 * i), 1.0 - math.cos(0.1 * i)) for i in range(1, 17)
    ]
    return line + arc


def _odometry(x: float, speed: float) -> Odometry:
    message = Odometry()
    message.header.frame_id = "map"
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    message.twist.twist.linear.x = speed
    return message


class TestControllerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("pp2010_test")
        cls.drives = {}
        for name in ("default", "trajectory", "fixed", "gain"):
            cls.node.create_subscription(
                AckermannDriveStamped,
                f"{PREFIX}/drive_{name}",
                lambda message, name=name: cls.drives.setdefault(name, message),
                RELIABLE_QOS,
            )
        cls.trajectories = [
            cls.node.create_publisher(
                Trajectory, f"{PREFIX}/{topic}", LATCHED_QOS
            )
            for topic in ("trajectory", "bend_trajectory")
        ]
        cls.odometries = [
            cls.node.create_publisher(
                Odometry, f"{PREFIX}/{topic}", RELIABLE_QOS
            )
            for topic in ("odom", "bend_odom")
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _drive(self):
        """Publish both scenarios until all four controllers answered."""
        self.trajectories[0].publish(_trajectory(_straight_points()))
        self.trajectories[1].publish(_trajectory(_bend_points()))
        deadline = time.monotonic() + TIMEOUT_SECONDS
        while len(self.drives) < 4 and time.monotonic() < deadline:
            # Straight: x = 2, no twist. Bend: x = 1.0, twist 2.5 m/s (R3:
            # lookahead_gain * speed = 2.5 exceeds lookahead_distance 1.0).
            self.odometries[0].publish(_odometry(2.0, 0.0))
            self.odometries[1].publish(_odometry(1.0, 2.5))
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertEqual(
            sorted(self.drives), ["default", "fixed", "gain", "trajectory"]
        )

    def test_pp_2010_trajectory_speed_reaches_the_drive_only_by_choice(self):
        self._drive()
        self.assertAlmostEqual(
            self.drives["trajectory"].drive.speed, TRAJECTORY_V, delta=1e-6
        )
        # The node default maximum_speed on a straight; v is ignored.
        self.assertAlmostEqual(
            self.drives["default"].drive.speed, 2.0, delta=1e-6
        )

    def test_pp_2010_odom_twist_reaches_the_lookahead_only_by_gain(self):
        self._drive()
        # Gain 0: target s = 1.0 + 1.0 = 2.0, still on the straight.
        self.assertAlmostEqual(
            self.drives["fixed"].drive.steering_angle, 0.0, delta=1e-6
        )
        # Gain 1: lookahead = max(1.0, 1.0 * 2.5) = 2.5, target s = 3.5,
        # 1.3 m into the arc at (3.164, 0.733) from (1, 0):
        # delta = atan(0.33 * 2 * 0.733 / (2.164^2 + 0.733^2)) = 0.0924 rad,
        # left. The two differ only if the twist reached the core.
        self.assertAlmostEqual(
            self.drives["gain"].drive.steering_angle, 0.0924, delta=0.01
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15]
        )
