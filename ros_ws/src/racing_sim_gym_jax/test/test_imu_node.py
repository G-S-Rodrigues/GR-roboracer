"""Tier-2: the live sim node's /imu carries body acceleration (SIMJAX-1540)."""

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
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu

# A namespace and remaps keep this node's absolute names off the shared
# graph (ADAPT-2170): other packages' tier-2 tests run in parallel on one
# domain, and /clock in particular is followed by every use_sim_time node.
NAMESPACE = "/test_simjax_imu"
ABSOLUTE_NAMES = (
    "/scan",
    "/ground_truth/scan",
    "/imu",
    "/odom",
    "/ground_truth/odom",
    "/ground_truth/track_relative_state",
    "/tf",
    "/drive",
    "/clock",
)
GRAVITY = 9.81

BEST_EFFORT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
RELIABLE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _name(name: str) -> str:
    return f"{NAMESPACE}{name}"


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="racing_sim_gym_jax",
                executable="racing_sim_gym_jax_node",
                namespace=NAMESPACE,
                remappings=[(n, _name(n)) for n in ABSOLUTE_NAMES],
                output="screen",
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class TestImuAcceleration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("simjax_imu_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_simjax_1540_imu_carries_body_acceleration(self):
        """SIMJAX-1540: on a constant-steer drive /imu (best-effort,
        base_link, sim-time stamp) reads a_z = g and a_y = v * omega of the
        same step's ground truth, and a_x is nonzero while the car speeds
        up; orientation and angular_velocity.z are what main publishes
        (ground truth's)."""
        imu: list[Imu] = []
        odometry: dict[int, Odometry] = {}
        self.node.create_subscription(
            Imu, _name("/imu"), imu.append, BEST_EFFORT
        )
        self.node.create_subscription(
            Odometry,
            _name("/ground_truth/odom"),
            lambda m: odometry.__setitem__(_ns(m.header.stamp), m),
            RELIABLE,
        )
        drive = self.node.create_publisher(
            AckermannDriveStamped, _name("/drive"), RELIABLE
        )
        deadline = time.monotonic() + 60.0
        # The JAX backend compiles before it publishes anything.
        while not imu and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(imu, "no /imu")
        start = time.monotonic()
        while time.monotonic() - start < 4.0:
            message = AckermannDriveStamped()
            message.drive.speed = 3.0
            message.drive.steering_angle = 0.2
            drive.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)

        paired = [m for m in imu if _ns(m.header.stamp) in odometry]
        self.assertGreater(len(paired), 100)
        peak_ax = 0.0
        turning = 0
        for message in paired:
            truth = odometry[_ns(message.header.stamp)]
            speed = truth.twist.twist.linear.x
            yaw_rate = truth.twist.twist.angular.z
            self.assertEqual(message.header.frame_id, "base_link")
            self.assertEqual(message.linear_acceleration.z, GRAVITY)
            self.assertAlmostEqual(
                message.linear_acceleration.y, speed * yaw_rate, places=9
            )
            self.assertEqual(message.angular_velocity.z, yaw_rate)
            self.assertEqual(message.orientation, truth.pose.pose.orientation)
            peak_ax = max(peak_ax, abs(message.linear_acceleration.x))
            turning += abs(message.linear_acceleration.y) > 0.5
        self.assertGreater(peak_ax, 0.5, "a_x never saw the speed change")
        self.assertGreater(turning, 10, "a_y never saw the turn")
        # Sim time, not wall time: the first stamps are near zero.
        self.assertLess(_ns(imu[0].header.stamp), 60 * 1_000_000_000)
