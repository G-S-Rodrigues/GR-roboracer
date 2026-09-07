"""ROS shell for the gym_jax simulator backend."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from racing_interfaces.msg import TrackRelativeState
from racing_interfaces.srv import Reset, SetStepMode
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, LaserScan

from .backend import DriveCommand, GymBackend, Snapshot
from .scenario import load_scenario

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


class RacingSimNode(Node):
    """Translate between ROS messages and a ROS-free warmed backend."""

    def __init__(self) -> None:
        super().__init__("racing_sim")
        default_scenario = (
            Path(get_package_share_directory("racing_sim_gym_jax"))
            / "config"
            / "scenarios"
            / "contract_test.yaml"
        )
        self.declare_parameter("scenario_path", str(default_scenario))
        self.declare_parameter("seed", 0)
        scenario_path = Path(
            self.get_parameter("scenario_path")
            .get_parameter_value()
            .string_value
        )
        self._scenario = load_scenario(scenario_path)
        seed = self.get_parameter("seed").get_parameter_value().integer_value
        self._backend = GymBackend(self._scenario, seed)
        self._step_mode = False
        self._simulation_time_ns = 0

        self._scan_publisher = self.create_publisher(
            LaserScan, "/scan", SENSOR_QOS
        )
        self._odometry_publisher = self.create_publisher(
            Odometry, "/odom", STATE_QOS
        )
        self._imu_publisher = self.create_publisher(Imu, "/imu", SENSOR_QOS)
        self._track_state_publisher = self.create_publisher(
            TrackRelativeState,
            "/ground_truth/track_relative_state",
            STATE_QOS,
        )
        self.create_subscription(
            AckermannDriveStamped, "/drive", self._on_drive, STATE_QOS
        )
        self.create_service(Reset, "~/reset", self._on_reset)
        self.create_service(SetStepMode, "~/step_mode", self._on_step_mode)
        self._timer = self.create_timer(
            self._scenario.control_period, self._on_timer
        )
        self.get_logger().info(
            "gym_jax warmed in "
            f"{self._backend.compile_latency_seconds:.3f}s; "
            f"env={self._scenario.env_id}"
        )

    def _on_drive(self, message: AckermannDriveStamped) -> None:
        drive = message.drive
        self._backend.set_command(
            DriveCommand(
                steering_angle=float(drive.steering_angle),
                steering_angle_velocity=float(drive.steering_angle_velocity),
                speed=float(drive.speed),
                acceleration=float(drive.acceleration),
            )
        )

    def _on_reset(self, request: Reset.Request, response: Reset.Response):
        if request.scenario_id not in ("", self._scenario.scenario_id):
            response.success = False
            response.message = f"unknown scenario: {request.scenario_id}"
            return response
        snapshot = self._backend.reset(int(request.seed))
        self._simulation_time_ns = 0
        self._publish(snapshot)
        response.success = True
        response.message = "reset"
        return response

    def _on_step_mode(
        self,
        request: SetStepMode.Request,
        response: SetStepMode.Response,
    ):
        if request.mode not in (
            SetStepMode.Request.REAL_TIME,
            SetStepMode.Request.STEPPED,
        ):
            response.success = False
            response.message = "unsupported step mode"
            return response
        self._step_mode = request.mode == SetStepMode.Request.STEPPED
        response.success = True
        response.message = "stepped" if self._step_mode else "real_time"
        return response

    def _on_timer(self) -> None:
        snapshot = (
            self._backend.snapshot()
            if self._step_mode
            else self._backend.step()
        )
        self._publish(snapshot)

    def _publish(self, snapshot: Snapshot) -> None:
        # Message time follows deterministic simulator time, not how quickly a
        # CPU host happens to execute the JAX step. This keeps recorded rates
        # and seeded runs comparable across CPU/GPU hardware.
        stamp = Time(
            sec=self._simulation_time_ns // 1_000_000_000,
            nanosec=self._simulation_time_ns % 1_000_000_000,
        )
        if not self._step_mode:
            self._simulation_time_ns += round(
                self._scenario.control_period * 1_000_000_000
            )
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        beam_count = len(snapshot.scan)
        field_of_view = float(self._scenario.parameters.get("fov", 4.7))
        scan.angle_min = -field_of_view / 2.0
        scan.angle_max = field_of_view / 2.0
        scan.angle_increment = (
            field_of_view / (beam_count - 1) if beam_count > 1 else 0.0
        )
        scan.time_increment = 0.0
        scan.scan_time = self._scenario.control_period
        scan.range_min = 0.0
        scan.range_max = float(self._scenario.parameters.get("max_range", 10.0))
        scan.ranges = snapshot.scan.astype(float).tolist()
        self._scan_publisher.publish(scan)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "map"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = snapshot.x
        odometry.pose.pose.position.y = snapshot.y
        odometry.pose.pose.orientation.z = math.sin(snapshot.yaw / 2.0)
        odometry.pose.pose.orientation.w = math.cos(snapshot.yaw / 2.0)
        odometry.twist.twist.linear.x = snapshot.speed
        odometry.twist.twist.angular.z = snapshot.yaw_rate
        self._odometry_publisher.publish(odometry)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        imu.orientation = odometry.pose.pose.orientation
        imu.angular_velocity.z = snapshot.yaw_rate
        self._imu_publisher.publish(imu)

        relative = TrackRelativeState()
        relative.header.stamp = stamp
        relative.header.frame_id = "map"
        relative.source = "gym_jax_ground_truth"
        relative.valid_until = stamp
        relative.s = snapshot.s
        relative.d = snapshot.d
        relative.heading_error = snapshot.heading_error
        relative.v_s = snapshot.v_s
        relative.v_d = snapshot.v_d
        relative.covariance = [0.0] * 25
        self._track_state_publisher.publish(relative)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RacingSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
