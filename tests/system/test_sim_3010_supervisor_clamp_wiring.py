"""SIM-3010: the safety supervisor clamps an excessive controller command
before it reaches ``/drive``, in a live (not unit-tested) ROS graph.

Launches only ``racing_safety_supervisor_node`` — SAFE-1010..1040
(`racing_safety_supervisor/test/supervisor_test.cpp`) already exhaustively
unit-test the clamp math itself. This test only proves that math is actually
wired into a running node: a command published on ``/controller/drive``
really does get clamped on the way to ``/drive`` and reported on
``/safety/status``.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from racing_interfaces.msg import SafetyStatus
from racing_test_keywords.ros_helpers import wait_for_node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

VEHICLE_CONFIG = "config/vehicles/f1tenth_default.yaml"
SUPERVISOR_NODE = "racing_safety_supervisor"
COMMAND_TOPIC = "/controller/drive"
DRIVE_TOPIC = "/drive"
STATUS_TOPIC = "/safety/status"
MAXIMUM_SPEED = 3.0  # config/vehicles/f1tenth_default.yaml: maximum_speed

RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
def generate_test_description():
    supervisor = launch_ros.actions.Node(
        package="racing_safety_supervisor",
        executable="racing_safety_supervisor_node",
        parameters=[VEHICLE_CONFIG],
    )
    return launch.LaunchDescription(
        [supervisor, launch_testing.actions.ReadyToTest()]
    )


class TestSim3010SupervisorClampWiring(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sim_3010_supervisor_clamp_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_sim_3010_supervisor_clamps_an_excessive_command_on_drive(self):
        wait_for_node(
            self.node,
            SUPERVISOR_NODE,
            {COMMAND_TOPIC, DRIVE_TOPIC, STATUS_TOPIC},
            timeout=15.0,
        )

        publisher = self.node.create_publisher(
            AckermannDriveStamped, COMMAND_TOPIC, RELIABLE_QOS
        )
        command = AckermannDriveStamped()
        command.drive.speed = 100.0  # far past maximum_speed: 3.0

        status_messages: list[SafetyStatus] = []
        drive_messages: list[AckermannDriveStamped] = []
        status_sub = self.node.create_subscription(
            SafetyStatus, STATUS_TOPIC, status_messages.append, RELIABLE_QOS
        )
        drive_sub = self.node.create_subscription(
            AckermannDriveStamped,
            DRIVE_TOPIC,
            drive_messages.append,
            RELIABLE_QOS,
        )
        try:
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                publisher.publish(command)
                rclpy.spin_once(self.node, timeout_sec=0.02)
                if (
                    status_messages
                    and status_messages[-1].active_clamps
                    & SafetyStatus.CLAMP_SPEED
                    and drive_messages
                    and drive_messages[-1].drive.speed == MAXIMUM_SPEED
                ):
                    break
        finally:
            self.node.destroy_subscription(status_sub)
            self.node.destroy_subscription(drive_sub)

        self.assertTrue(status_messages, "no message on /safety/status")
        self.assertTrue(drive_messages, "no message on /drive")
        self.assertTrue(
            status_messages[-1].active_clamps & SafetyStatus.CLAMP_SPEED
        )
        self.assertEqual(
            status_messages[-1].reason, SafetyStatus.REASON_COMMAND_LIMIT
        )
        self.assertEqual(drive_messages[-1].drive.speed, MAXIMUM_SPEED)
