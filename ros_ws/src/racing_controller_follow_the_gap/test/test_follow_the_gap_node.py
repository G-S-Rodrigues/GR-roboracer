"""FTG-2010 and FTG-2020: the follow-the-gap node over real topics.

FTG-2010: one command per scan, on /controller/drive, stamped with the scan's
stamp, frame ``base_link``, nothing before the first scan, and an opening on
the left steers left.

FTG-2020: launched with ``hold: true`` the node publishes zero commands (so a
supervisor never stales) however open the scan is; setting ``hold`` false at
runtime releases it, and the first command is capped by the launch ramp.

Every topic is renamed under ``/test_ftg2010/`` because colcon runs tier-2
tests in parallel on one domain.
"""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PREFIX = "/test_ftg2010"
NODE = "ftg_2010"
SCAN = f"{PREFIX}/scan"
DRIVE = f"{PREFIX}/drive"
ANGLE_MIN = -2.35619
INCREMENT = 0.004363323
BEAMS = 1081
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package="racing_controller_follow_the_gap",
        executable="racing_controller_follow_the_gap_node",
        name=NODE,
        namespace=PREFIX,
        remappings=[("/scan", SCAN), ("/controller/drive", DRIVE)],
        parameters=[{"hold": True}],
        output="screen",
    )
    return launch.LaunchDescription(
        [node, launch_testing.actions.ReadyToTest()]
    )


def _scan(stamp_ns: int, opening_deg: float) -> LaserScan:
    a = ANGLE_MIN + INCREMENT * np.arange(BEAMS)
    r = np.full(BEAMS, 0.5)
    r[int(np.argmin(np.abs(a - math.copysign(1.5, -opening_deg))))] = 0.4
    r[np.abs(a - math.radians(opening_deg)) <= math.radians(6)] = 5.0
    m = LaserScan()
    m.header.frame_id = "lidar"
    m.header.stamp.sec = stamp_ns // 10**9
    m.header.stamp.nanosec = stamp_ns % 10**9
    m.angle_min = ANGLE_MIN
    m.angle_max = ANGLE_MIN + INCREMENT * (BEAMS - 1)
    m.angle_increment = INCREMENT
    m.range_min = 0.06
    m.range_max = 10.0
    m.ranges = [float(x) for x in r]
    return m


class TestFollowTheGapNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("ftg_2010_test")
        cls.received = []
        cls.node.create_subscription(
            AckermannDriveStamped, DRIVE, cls.received.append, RELIABLE
        )
        cls.scans = cls.node.create_publisher(
            LaserScan, SCAN, qos_profile_sensor_data
        )
        cls.params = cls.node.create_client(
            SetParameters, f"{PREFIX}/{NODE}/set_parameters"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _wait_connected(self) -> None:
        end = time.monotonic() + 10.0
        while time.monotonic() < end and not (
            self.scans.get_subscription_count() and self.params.service_is_ready()
        ):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.scans.get_subscription_count(), "node not found")
        self._spin(1.0)

    def _set_hold(self, value: bool) -> None:
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="hold",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=value
                ),
            )
        ]
        future = self.params.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.result().results[0].successful)

    def _send(self, n: int, opening_deg: float, start_ns: int) -> None:
        for k in range(n):
            self.scans.publish(_scan(start_ns + k * 10_000_000, opening_deg))
            self._spin(0.03)
        self._spin(0.3)

    def test_ftg_2020_then_2010_hold_release_and_contract(self):
        self._wait_connected()
        # FTG-2010: nothing before the first scan.
        self.assertEqual(len(self.received), 0)
        # FTG-2020: held, so zero commands, one per scan, however open.
        self._send(5, 10.0, 1_000_000_000_000)
        self.assertEqual(len(self.received), 5)
        for m in self.received:
            self.assertEqual(m.drive.speed, 0.0)
            self.assertEqual(m.drive.steering_angle, 0.0)
        # Released: the first command is capped by the launch ramp.
        self.received.clear()
        self._set_hold(False)
        base_ns = 2_000_000_000_000
        self._send(5, 10.0, base_ns)
        # FTG-2010: one command per scan, the scan's stamp, frame, sign.
        self.assertEqual(len(self.received), 5)
        for k, m in enumerate(self.received):
            ns = base_ns + k * 10_000_000
            self.assertEqual(m.header.stamp.sec, ns // 10**9)
            self.assertEqual(m.header.stamp.nanosec, ns % 10**9)
            self.assertEqual(m.header.frame_id, "base_link")
            self.assertGreater(m.drive.steering_angle, 0.1)  # opening left
        first = self.received[0].drive.speed
        self.assertGreater(first, 0.0)
        self.assertLessEqual(first, 0.8 + 1e-6)  # minimum_speed at t = 0
        self.assertGreater(self.received[-1].drive.speed, first)  # ramping
        # Opening on the right steers right.
        self.received.clear()
        self._send(2, -10.0, 3_000_000_000_000)
        self.assertEqual(len(self.received), 2)
        self.assertLess(self.received[-1].drive.steering_angle, -0.1)
        # Held again: zeros again.
        self._set_hold(True)
        self.received.clear()
        self._send(2, 10.0, 4_000_000_000_000)
        self.assertEqual([m.drive.speed for m in self.received], [0.0, 0.0])


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
