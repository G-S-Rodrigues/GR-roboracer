"""ROS shell of front end A: our Socket.IO server for the AutoDRIVE sim.

The Unity sim connects here as a client, once, and sends `Bridge` frames; this
node answers each with throttle, steering and reset, and publishes the
simulator contract's topics (racing_sim_adapter/contract.hpp) under the node
name `racing_sim`. Every conversion is `bridge.BridgeSession` (ROS-free).

Threading (validated in plan step 1, no monkey-patching): the gevent
WSGIServer runs on the main thread and calls the Bridge handler; the rclpy
executor spins in a daemon thread for `/drive` and the services. Publishing
from the handler is safe; the only shared state is the latest `/drive`
(under a lock) and the session's reset flag (one bool).

Stamps are the node clock at frame receipt (ADR 0006, `free_running`),
exactly as the official devkit stamps; the sim sends none.
"""

from __future__ import annotations

import faulthandler
import math
import signal
import sys
import threading
import time
from pathlib import Path

import gevent
import numpy as np
import rclpy
import socketio
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
from nav_msgs.msg import Odometry
from racing_common import CartesianPose, Track
from racing_interfaces.msg import TrackRelativeState
from racing_interfaces.srv import Reset, SetStepMode
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor
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

from racing_sim_autodrive import bridge, core
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


# A connected sim silent for this long is logged once (gotcha 29, 30).
STALL_TIMEOUT_S = 0.5
# The executor thread's loop: wait this long for work, then yield the GIL.
SPIN_TIMEOUT_S = 0.05
SPIN_YIELD_S = 0.001


class AutoDriveServer(Node):
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
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 4567)  # config/autodrive/simulator.yaml
        self.declare_parameter("dt_source", "lap_time")
        # No defaults: a missing value fails at start, never runs on a guess
        # (CLAUDE.md of this package; part 8 passes the vehicle file).
        for name in VEHICLE_PARAMETERS:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        for name in ("world_to_map.x", "world_to_map.y", "world_to_map.yaw"):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter("track_path", Parameter.Type.STRING)
        self.declare_parameter("scenario_id", Parameter.Type.STRING)

        def number(name: str) -> float:
            return self.get_parameter(name).get_parameter_value().double_value

        def text(name: str) -> str:
            return self.get_parameter(name).get_parameter_value().string_value

        vehicle = bridge.VehicleValues(
            **{name: number(name) for name in VEHICLE_PARAMETERS}
        )
        self._scenario_id = text("scenario_id")
        self._host = text("host")
        self._port = (
            self.get_parameter("port").get_parameter_value().integer_value
        )
        track = Track.from_yaml(str(Path(text("track_path"))))
        self._session = bridge.BridgeSession(
            vehicle,
            CartesianPose(
                number("world_to_map.x"),
                number("world_to_map.y"),
                number("world_to_map.yaw"),
            ),
            track,
            dt_source=text("dt_source"),
        )
        self._vehicle = vehicle
        self._drive: bridge.DriveRequest | None = None
        self._drive_lock = threading.Lock()
        self._warned_unsupported = False
        self._sid: str | None = None
        self._stall = bridge.StallWatch(STALL_TIMEOUT_S)

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

        self.sio = socketio.Server(async_mode="gevent")
        self.sio.on("connect", self._on_connect)
        self.sio.on("disconnect", self._on_disconnect)
        self.sio.on("Bridge", self._on_bridge)

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

    # ---- Socket.IO side (gevent greenlet, main thread) ----

    def _on_connect(self, sid, environ) -> None:
        self._sid = sid
        self._stall.connected(time.monotonic())
        self.get_logger().info(f"sim connected: {sid}")

    def _on_disconnect(self, sid) -> None:
        self._stall.disconnected()
        self.get_logger().error(
            f"sim disconnected: {sid}. It never reconnects (gotcha 30): "
            "restart the sim."
        )

    def _on_bridge(self, sid, data) -> None:
        self._stall.entered()
        try:
            self._handle_bridge(sid, data)
        finally:
            self._stall.exited(time.monotonic())

    def _handle_bridge(self, sid, data) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        with self._drive_lock:
            drive = self._drive
        try:
            step = self._session.on_frame(
                data, drive, received_at=now.nanoseconds * 1e-9
            )
        except core.FrameError as error:
            # the sim is never left waiting (gotcha 29)
            self.get_logger().error(f"bad Bridge frame: {error}")
            self.sio.emit("Bridge", bridge.fallback_reply(), to=sid)
            self._stall.emitted()
            return
        self._publish(stamp, step)
        self.sio.emit("Bridge", step.reply, to=sid)
        self._stall.emitted()

    def _watch_stalls(self) -> None:
        """Log one WARNING and one all-thread stack dump per stall."""
        while True:
            gevent.sleep(STALL_TIMEOUT_S / 5)
            report = self._stall.check(time.monotonic())
            if report is not None:
                self.get_logger().warning(f"stall: {report}")
                faulthandler.dump_traceback(file=sys.stderr, all_threads=True)

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

    def serve(self) -> None:
        """Run the Socket.IO server until SIGINT/SIGTERM (main thread)."""
        server = pywsgi.WSGIServer(
            (self._host, self._port),
            socketio.WSGIApp(self.sio),
            handler_class=WebSocketHandler,
        )
        # Bound before the handlers and the log line: a signal arriving ahead
        # of start() would stop an unstarted server and crash serve_forever.
        server.start()
        gevent.spawn(self._watch_stalls)
        gevent.signal_handler(signal.SIGINT, server.stop)
        gevent.signal_handler(signal.SIGTERM, server.stop)
        self.get_logger().info(f"listening on {self._host}:{self._port}")
        server.serve_forever()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoDriveServer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stopping = threading.Event()

    def spin() -> None:
        # Not executor.spin(): under a /drive flood that loop never blocks,
        # holds the GIL, and the gevent hub on the main thread gets it once
        # per switch interval, so replies drain at ~20/s (SIMAD-2080). One
        # callback, then sleep: the sleep releases the GIL for the hub.
        try:
            while rclpy.ok() and not stopping.is_set():
                executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)
                time.sleep(SPIN_YIELD_S)
        except Exception:
            if rclpy.ok():
                raise

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        node.serve()
    finally:
        stopping.set()
        thread.join(timeout=5.0)  # a spin left running aborts at exit
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
