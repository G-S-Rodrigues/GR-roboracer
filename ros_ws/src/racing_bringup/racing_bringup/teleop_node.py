"""The hand-driving node: /key_vel (TwistStamped) in, a limited /drive out.

`linear.x` is the target speed in m/s and `angular.z` the target steering
angle in rad (racing_bringup.teleop). The node owns /drive: nothing else
runs under `slam.launch.py drive:=teleop`. The AutoDRIVE server keeps the last
/drive forever, so a dead node leaves the car driving (gotcha #41); the node
therefore publishes zero on shutdown. That covers this node stopping (a
crash, or a kill of the node alone), not a whole-launch Ctrl+C, which races
the server: `ros2 launch` signals every process at once and the server may
close before this zero reaches it. A SIGKILL cannot be covered either.
"""

import contextlib
import dataclasses
import signal
import threading

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions

from racing_bringup import teleop

SPIN_TIMEOUT_S = 0.05
ACK_TIMEOUT_S = 1.0
ZEROS_AT_SHUTDOWN = 3
# The server's /drive QoS (racing_sim_autodrive contract_node.STATE_QOS):
# another QoS and the server hears nothing (gotcha #6).
DRIVE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("racing_bringup_teleop")
        names = [f.name for f in dataclasses.fields(teleop.Limits)]
        for name in names:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.limiter = teleop.Limiter(
            teleop.Limits(
                **{
                    name: self.get_parameter(name).value  # unset raises
                    for name in names
                }
            )
        )
        self._linear = 0.0
        self._angular = 0.0
        self._received: float | None = None
        self._last: float | None = None
        self.create_subscription(TwistStamped, "/key_vel", self._on_key, 10)
        self._publisher = self.create_publisher(
            AckermannDriveStamped, "/drive", DRIVE_QOS
        )
        self.create_timer(1.0 / self.limiter.limits.rate_hz, self._tick)

    def _seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_key(self, message: TwistStamped) -> None:
        self._linear = message.twist.linear.x
        self._angular = message.twist.angular.z
        self._received = self._seconds()

    def _tick(self) -> None:
        # One clock read serves the dt, the age and the stamp.
        now = self.get_clock().now()
        seconds = now.nanoseconds * 1e-9
        dt = 0.0 if self._last is None else seconds - self._last
        self._last = seconds
        age = None if self._received is None else seconds - self._received
        speed, steering = self.limiter.step(
            *teleop.targets(
                self._linear,
                self._angular,
                age,
                self.limiter.limits.input_timeout,
            ),
            dt,
        )
        self._publish(speed, steering, now.to_msg())

    def _publish(self, speed: float, steering: float, stamp=None) -> None:
        message = AckermannDriveStamped()
        message.header.stamp = (
            stamp if stamp is not None else self.get_clock().now().to_msg()
        )
        message.drive.speed = float(speed)
        message.drive.steering_angle = float(steering)
        self._publisher.publish(message)

    def shutdown_commands(self) -> None:
        """Zero speed and steering, sent while the context is alive."""
        for _ in range(ZEROS_AT_SHUTDOWN):
            self._publish(0.0, 0.0)
        # an unreachable server must not hang exit
        with contextlib.suppress(Exception):
            self._publisher.wait_for_all_acked(Duration(seconds=ACK_TIMEOUT_S))


def _ignore_signals() -> None:
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, signal.SIG_IGN)


def main(args=None) -> None:
    # The node owns the signals, as racing_sim_autodrive's devkit_node does
    # (SIMAD-2230): rclpy's own handler shuts the context down before the
    # zero can leave, and the server then drives on the last /drive.
    stop = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stop.set())
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = TeleopNode()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
    finally:
        _ignore_signals()
        if node is not None:
            node.shutdown_commands()  # the context is still alive
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _ignore_signals()


if __name__ == "__main__":
    main()
