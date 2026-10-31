"""The simulator contract both AutoDRIVE front ends publish.

`ContractNode` is node `racing_sim`: the capability and vehicle parameters,
the contract topics (`racing_sim_adapter/contract.hpp`), `/drive`,
`~/reset` and `~/step_mode`, and `_publish` of one `bridge.Step`. What
differs between the front ends is where a frame comes from (A: our
Socket.IO server, `server_node`; B: the official devkit's ROS topics,
`devkit_node`) and how the session is built (`_build_session`).
"""

from __future__ import annotations

import math
import signal
import threading
from pathlib import Path

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from racing_common import CartesianPose, Track
from racing_interfaces.msg import TrackRelativeState
from racing_interfaces.srv import Reset, SetStepMode
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster

from racing_sim_autodrive import bridge
from racing_sim_autodrive.capabilities import CAPABILITIES

SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

SCAN_FRAME_ID = bridge.SCAN_FRAME_ID  # both scans, the contract name

VEHICLE_PARAMETERS = (
    "lr",
    "h",
    "wheel_radius",
    "throttle_gain",
    "max_steering_angle",
    "kp",
    "ki",
)


def _quaternion_z(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ContractNode(Node):
    """Node `racing_sim`; subclasses add a frame source and `_build_session`."""

    def __init__(self) -> None:
        super().__init__("racing_sim")
        for name, value in CAPABILITIES.items():
            self.declare_parameter(
                name,
                value,
                ParameterDescriptor(
                    read_only=True,
                    description="simulator capability (ADAPT-2140)",
                ),
                ignore_override=True,
            )
        # No defaults: a missing value fails at start, never runs on a guess
        # (CLAUDE.md of this package; part 8 passes the vehicle file).
        for name in VEHICLE_PARAMETERS:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        for name in ("world_to_map.x", "world_to_map.y", "world_to_map.yaw"):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter("track_path", Parameter.Type.STRING)
        self.declare_parameter("scenario_id", Parameter.Type.STRING)

        vehicle = bridge.VehicleValues(
            **{name: self._number(name) for name in VEHICLE_PARAMETERS}
        )
        self._scenario_id = self._text("scenario_id")
        track = Track.from_yaml(str(Path(self._text("track_path"))))
        self._session = self._build_session(
            vehicle,
            CartesianPose(
                self._number("world_to_map.x"),
                self._number("world_to_map.y"),
                self._number("world_to_map.yaw"),
            ),
            track,
        )
        self._vehicle = vehicle
        self._drive: bridge.DriveRequest | None = None
        self._drive_lock = threading.Lock()
        self._warned_unsupported = False

        self._scan_publisher = self.create_publisher(
            LaserScan, "/scan", SENSOR_QOS
        )
        self._truth_scan_publisher = self.create_publisher(
            LaserScan, "/ground_truth/scan", SENSOR_QOS
        )
        self._imu_publisher = self.create_publisher(Imu, "/imu", SENSOR_QOS)
        self._odometry_publisher = self.create_publisher(
            Odometry, "/odom", STATE_QOS
        )
        self._truth_odometry_publisher = self.create_publisher(
            Odometry, "/ground_truth/odom", STATE_QOS
        )
        self._track_publisher = self.create_publisher(
            TrackRelativeState, "/ground_truth/track_relative_state", STATE_QOS
        )
        self._encoder_publisher = self.create_publisher(
            JointState, "~/wheel_encoders", SENSOR_QOS
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            AckermannDriveStamped, "/drive", self._on_drive, STATE_QOS
        )
        self.create_service(Reset, "~/reset", self._on_reset)
        self.create_service(SetStepMode, "~/step_mode", self._on_step_mode)

    def _number(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _text(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _build_session(
        self, vehicle: bridge.VehicleValues, world_to_map, track
    ) -> bridge.BridgeSession:
        raise NotImplementedError

    # ---- ROS side (executor thread) ----

    def _on_drive(self, message: AckermannDriveStamped) -> None:
        drive = message.drive
        if (
            drive.acceleration or drive.steering_angle_velocity
        ) and not self._warned_unsupported:
            self._warned_unsupported = True
            self.get_logger().warning(
                "/drive acceleration and steering_angle_velocity are ignored; "
                "only speed and steering_angle are used"
            )
        with self._drive_lock:
            self._drive = bridge.DriveRequest(
                float(drive.speed), float(drive.steering_angle)
            )

    def _on_reset(self, request: Reset.Request, response: Reset.Response):
        response.success, response.message = bridge.handle_reset(
            request.scenario_id, self._scenario_id
        )
        if response.success:
            self._session.request_reset()
        return response

    def _on_step_mode(
        self, request: SetStepMode.Request, response: SetStepMode.Response
    ):
        response.success, response.message = bridge.handle_step_mode(
            request.mode
        )
        return response

    def _publish(self, stamp, step: bridge.Step) -> None:
        truth, odom = step.ground_truth, step.odometry
        ranges = step.frame.ranges
        if ranges is not None:
            geometry = bridge.scan_geometry(len(ranges))
            for publisher, frame_id in (
                (self._scan_publisher, SCAN_FRAME_ID),
                (self._truth_scan_publisher, SCAN_FRAME_ID),
            ):
                scan = LaserScan()
                scan.header.stamp = stamp
                scan.header.frame_id = frame_id
                scan.angle_min = geometry.angle_min
                scan.angle_max = geometry.angle_max
                scan.angle_increment = geometry.angle_increment
                scan.range_min = geometry.range_min
                scan.range_max = geometry.range_max
                scan.scan_time = 0.0
                scan.ranges = np.asarray(ranges, dtype=float).tolist()
                publisher.publish(scan)

        truth_message = self._odometry_message(stamp, "map", truth)
        odometry = self._odometry_message(stamp, "odom", odom)
        self._truth_odometry_publisher.publish(truth_message)
        self._odometry_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = odometry.child_frame_id
        transform.transform.translation.x = odom.x
        transform.transform.translation.y = odom.y
        transform.transform.translation.z = odom.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        imu.orientation = truth_message.pose.pose.orientation
        imu.angular_velocity.z = truth.wz
        self._imu_publisher.publish(imu)

        encoders = JointState()
        encoders.header.stamp = stamp
        encoders.name = ["left_rear_wheel", "right_rear_wheel"]
        encoders.position = list(step.frame.encoder_angles)
        self._encoder_publisher.publish(encoders)

        if step.track_state is not None:
            state = TrackRelativeState()
            state.header.stamp = stamp
            state.header.frame_id = "map"
            state.source = "autodrive_ground_truth"
            state.valid_until = stamp
            state.s = step.track_state.s
            state.d = step.track_state.d
            state.heading_error = step.track_state.heading_error
            state.v_s = step.track_state.v_s
            state.v_d = step.track_state.v_d
            state.covariance = [0.0] * 25
            self._track_publisher.publish(state)

    @staticmethod
    def _odometry_message(stamp, frame_id: str, state: bridge.CogState):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = state.x
        message.pose.pose.position.y = state.y
        message.pose.pose.position.z = state.z
        z, w = _quaternion_z(state.yaw)
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        message.twist.twist.linear.x = state.vx
        message.twist.twist.linear.y = state.vy
        message.twist.twist.angular.z = state.wz
        return message


def _ignore_signals() -> None:
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, signal.SIG_IGN)
