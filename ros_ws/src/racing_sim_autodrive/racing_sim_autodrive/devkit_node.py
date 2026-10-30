"""ROS shell of front end B: the official AutoDRIVE devkit bridge.

The devkit (`autodrive_roboracer`) is the process that talks to the Unity
sim; this node never does. It subscribes the devkit's 13 per-frame topics,
assembles them into frames (`devkit.DevkitAssembler`), runs the same
`bridge.BridgeSession` front end A runs, publishes the simulator contract
(`contract_node.ContractNode`) and sends the session's throttle, steering and
reset back as the devkit's three command topics.

Every contract message carries the devkit's `lidar` header stamp (ruling E2:
the stamp of the event the devkit received, as A stamps its own receipt), and
dt is the difference of those stamps. `dt_source=lap_time` does not exist
here (owner D4).

`DevkitLink` is the part that talks to the devkit's graph: its own node, so
a second rclpy context can hold it (step 7).
"""

from __future__ import annotations

import contextlib
import signal
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, Float32, Int32

from racing_sim_autodrive import bridge, core, devkit
from racing_sim_autodrive.contract_node import ContractNode, _ignore_signals

# A topic silent this long while others flow is named in a stall warning.
STALL_TIMEOUT_S = 0.5
STALL_PERIOD_S = 0.1
ACK_TIMEOUT_S = 1.0
SPIN_TIMEOUT_S = 0.05

DEVKIT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_TYPES = {
    "throttle": Float32,
    "steering": Float32,
    "left_encoder": JointState,
    "right_encoder": JointState,
    "ips": Point,
    "imu": Imu,
    "odom": Odometry,
    "lidar": LaserScan,
    "lap_count": Int32,
    "lap_time": Float32,
    "last_lap_time": Float32,
    "best_lap_time": Float32,
    "collision_count": Int32,
}


def _xyz(v):
    return (float(v.x), float(v.y), float(v.z))


def _stamp_ns(header) -> int:
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def _extract(topic: str, msg):
    """The plain value the assembler keeps of one devkit message."""
    if topic in ("throttle", "steering", "lap_time", "last_lap_time"):
        return float(msg.data)
    if topic == "best_lap_time":
        return float(msg.data)
    if topic in ("lap_count", "collision_count"):
        return int(msg.data)
    if topic in ("left_encoder", "right_encoder"):
        return float(msg.position[0])
    if topic == "ips":
        return _xyz(msg)
    if topic == "imu":
        q = msg.orientation
        return (
            (float(q.x), float(q.y), float(q.z), float(q.w)),
            _xyz(msg.angular_velocity),
            _xyz(msg.linear_acceleration),
        )
    if topic == "odom":
        return (_xyz(msg.pose.pose.position), _xyz(msg.twist.twist.linear))
    return (list(msg.ranges), float(msg.scan_time))  # lidar


class DevkitLink:
    """Subscriptions and command publishers on the devkit's graph."""

    def __init__(
        self, namespace, reset_topic, on_frame, context=None, node_namespace=""
    ):
        self.node = Node(
            "racing_sim_devkit_link",
            context=context,
            namespace=node_namespace or None,
            # not the process's `--ros-args`: its `__node:=racing_sim` would
            # rename this node too, and every name here is already absolute
            use_global_arguments=False,
        )
        self._on_frame = on_frame
        self._assembler = devkit.DevkitAssembler()
        self._lock = threading.Lock()
        self._seen: dict[str, float] = {}
        base = namespace.rstrip("/")
        self._subscriptions = [
            self.node.create_subscription(
                _TYPES[topic],
                f"{base}/{topic}",
                self._callback(topic),
                DEVKIT_QOS,
            )
            for topic in devkit.REQUIRED
        ]
        self._throttle = self.node.create_publisher(
            Float32, f"{base}/throttle_command", COMMAND_QOS
        )
        self._steering = self.node.create_publisher(
            Float32, f"{base}/steering_command", COMMAND_QOS
        )
        self._reset = self.node.create_publisher(Bool, reset_topic, COMMAND_QOS)

    @property
    def assembler(self) -> devkit.DevkitAssembler:
        return self._assembler

    def _callback(self, topic):
        def callback(msg, info):
            header = _stamp_ns(msg.header) if topic in devkit.HEADERED else None
            with self._lock:
                self._seen[topic] = time.monotonic()
                frames = self._assembler.offer(
                    topic,
                    _extract(topic, msg),
                    int(info["source_timestamp"]),
                    int(info["publication_sequence_number"]),
                    header,
                )
            for frame in frames:
                self._on_frame(frame)

        return callback

    def any_seen(self) -> bool:
        with self._lock:
            return bool(self._seen)

    def silent_topics(self, now: float, timeout_s: float) -> list[str]:
        with self._lock:
            return [
                t
                for t in devkit.REQUIRED
                if now - self._seen.get(t, -1e9) > timeout_s
            ]

    def publish_commands(self, throttle, steering, reset) -> None:
        self._throttle.publish(Float32(data=float(throttle)))
        self._steering.publish(Float32(data=float(steering)))
        if reset is not None:
            self._reset.publish(Bool(data=bool(reset)))

    def shutdown_commands(self, pending_reset: bool) -> None:
        """Zeros (and False if a reset True is held), then wait for acks."""
        self.publish_commands(*devkit.shutdown_commands(pending_reset))
        for publisher in (self._throttle, self._steering, self._reset):
            # an unreachable devkit must not hang exit
            with contextlib.suppress(Exception):
                publisher.wait_for_all_acked(Duration(seconds=ACK_TIMEOUT_S))


class DevkitNode(ContractNode):
    def __init__(self) -> None:
        super().__init__()
        namespace = self._text("devkit_namespace")
        reset_topic = self._text("devkit_reset_topic")
        self._commands = devkit.DevkitCommands()
        self._stall = bridge.StallWatch(STALL_TIMEOUT_S)
        self._stall_connected = False
        self.link = DevkitLink(
            namespace,
            reset_topic,
            self._on_assembled,
            node_namespace=self.get_namespace(),
        )
        self.create_timer(STALL_PERIOD_S, self._watch_stalls)
        self.get_logger().info(f"devkit bridge on {namespace}")

    def _build_session(self, vehicle, world_to_map, track):
        self.declare_parameter("devkit_namespace", "/autodrive/roboracer_1")
        self.declare_parameter("devkit_reset_topic", "/autodrive/reset_command")
        return devkit.make_session(vehicle, world_to_map, track)

    def _on_assembled(self, assembled: devkit.AssembledFrame) -> None:
        ns = assembled.stamp_ns
        stamp = Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
        with self._drive_lock:
            drive = self._drive
        self._stall.entered()
        try:
            try:
                frame = core.frame_from_devkit(assembled.sample)
            except core.FrameError as error:
                self.get_logger().error(f"bad devkit frame: {error}")
                return
            step = self._session.on_sim_frame(
                frame, drive, received_at=ns * 1e-9
            )
            self._publish(stamp, step)
            throttle, steering, reset = self._commands.from_step(
                step, self._session.awaiting_reset
            )
            self.link.publish_commands(throttle, steering, reset)
            self._stall.emitted()
        finally:
            self._stall.exited(time.monotonic())

    def _watch_stalls(self) -> None:
        now = time.monotonic()
        if not self._stall_connected and self.link.any_seen():
            self._stall_connected = True
            self._stall.connected(now)
        report = self._stall.check(now)
        if report is not None:
            silent = self.link.silent_topics(now, STALL_TIMEOUT_S)
            self.get_logger().warning(
                f"stall: {report}; silent devkit topics: "
                f"{', '.join(silent) or 'none'}"
            )

    def shutdown_commands(self) -> None:
        a = self.link.assembler
        self.get_logger().info(
            f"assembler: resyncs={a.resyncs} dropped={a.dropped_frames} "
            f"lost={ {t: n for t, n in a.lost.items() if n} }"
        )
        self.link.shutdown_commands(self._commands.pending_reset)


def main(args=None) -> None:
    # B owns the signals: rclpy's own handler would shut the context down
    # before the zero commands could leave (plan P4).
    stop = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stop.set())
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = DevkitNode()
        executor.add_node(node)
        executor.add_node(node.link.node)
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)
    finally:
        _ignore_signals()
        if node is not None:
            node.shutdown_commands()
        executor.shutdown()
        if node is not None:
            node.link.node.destroy_node()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _ignore_signals()


if __name__ == "__main__":
    main()
