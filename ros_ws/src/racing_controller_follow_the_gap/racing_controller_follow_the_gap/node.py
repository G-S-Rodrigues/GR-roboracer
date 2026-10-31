"""Thin ROS shell over ``gap``: /scan in, /controller/drive out.

Every decision lives in ``racing_controller_follow_the_gap.gap``. This node
owns the topics, the ``hold`` switch and the clock of the launch ramp.
"""

from __future__ import annotations

import dataclasses
import signal

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from racing_controller_follow_the_gap import gap

SCAN_TOPIC = "/scan"
DRIVE_TOPIC = "/controller/drive"
# As racing_controller_baseline: the supervisor subscribes reliable.
DRIVE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class FollowTheGapNode(Node):
    def __init__(self) -> None:
        super().__init__("racing_controller_follow_the_gap")
        defaults = gap.Parameters()
        values = {
            f.name: self.declare_parameter(
                f.name, getattr(defaults, f.name)
            ).value
            for f in dataclasses.fields(defaults)
        }
        self._gap = gap.Parameters(**values)
        self._gap.validate()
        # `hold` publishes zero commands (the supervisor never stales) until
        # it is set false, e.g. while a simulator is reset.
        self._hold = bool(self.declare_parameter("hold", False).value)
        self._released_at: float | None = None
        self.add_on_set_parameters_callback(self._on_parameters)
        self._publisher = self.create_publisher(
            AckermannDriveStamped, DRIVE_TOPIC, DRIVE_QOS
        )
        self.create_subscription(
            LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data
        )

    def _on_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "hold":
                self._hold = bool(parameter.value)
                self._released_at = None
            elif parameter.name != "use_sim_time":
                return SetParametersResult(
                    successful=False, reason="only `hold` is settable at runtime"
                )
        return SetParametersResult(successful=True)

    def _on_scan(self, scan: LaserScan) -> None:
        stamp = _seconds(scan.header.stamp)
        steering, speed = 0.0, 0.0
        if not self._hold:
            if self._released_at is None:
                self._released_at = stamp
            steering, speed = gap.command(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                self._gap,
            )
            speed = gap.limit_speed(
                speed, stamp - self._released_at, self._gap
            )
        message = AckermannDriveStamped()
        message.header.stamp = scan.header.stamp
        message.header.frame_id = "base_link"
        message.drive.steering_angle = steering
        message.drive.speed = speed
        self._publisher.publish(message)


def let_rclpy_own_sigint() -> None:
    """Let rclpy's handler be the only thing SIGINT does (EVAL-1050)."""
    signal.signal(signal.SIGINT, lambda signum, frame: None)


def spin_until_shutdown(node) -> None:
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if node.context.ok():
            raise


def main() -> None:
    let_rclpy_own_sigint()
    rclpy.init()
    node = FollowTheGapNode()
    try:
        spin_until_shutdown(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
