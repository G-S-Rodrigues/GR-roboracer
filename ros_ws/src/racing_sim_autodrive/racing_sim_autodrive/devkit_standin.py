"""A stand-in for the official AutoDRIVE devkit's ROS side (front end B's CI).

The real devkit (`autodrive_bridge.py` in the Humble image) cannot run in
CI, so B is tested against this. Two layers:

- Pure: `devkit_values` copies the devkit's `Bridge` handler
  (`autodrive_bridge.py:309-342`): the same keys, read unguarded, so a
  missing one (the headless first frame's lidar, gotcha 29) means no publish
  and no reply, as the devkit's `KeyError` does. `devkit_reply` copies its
  reply strings. The devkit is third-party code in a Humble image, and
  importing it pulls in socketio and cv_bridge, so this is a deliberate
  duplicate; SIM-3720 holds it to the pinned image.
- ROS shell (`main`): replays the recorded frames as the sim would, and per
  frame publishes the 13 per-frame topics in the devkit's order, with its QoS
  (RELIABLE, VOLATILE, KEEP_LAST 1), types and frame ids, then its 11 `/tf`
  transforms. It latches the three command topics as the devkit does and
  writes one reply per frame.
"""

from __future__ import annotations

import argparse
import base64
import gzip
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from racing_sim_autodrive import core, replay

__all__ = [
    "DevkitValues",
    "build_parser",
    "devkit_reply",
    "devkit_sample",
    "devkit_values",
    "latch_command",
    "main",
]

DEFAULT_NAMESPACE = "/autodrive/roboracer_1"
DEFAULT_RESET_TOPIC = "/autodrive/reset_command"
LIDAR_KEY = "V1 LIDAR Range Array"

# The per-frame topic suffixes, in the order the devkit publishes them
# (autodrive_bridge.py:343-370): feedbacks, encoders, ips, imu, odom, (tf),
# lidar, (camera), then the lap data.
PUBLISH_ORDER = (
    "throttle",
    "steering",
    "left_encoder",
    "right_encoder",
    "ips",
    "imu",
    "odom",
    "lidar",
    "lap_count",
    "lap_time",
    "last_lap_time",
    "best_lap_time",
    "collision_count",
)


def _f32(value: float) -> float:
    return float(np.float32(value))


def _vector(raw, key: str) -> tuple[float, ...]:
    return tuple(float(t) for t in raw[key].split())


@dataclass(frozen=True)
class DevkitValues:
    """What the devkit publishes for one `Bridge` frame, at its precisions.

    The two echoes, the three lap times and the ranges are Float32 on the
    wire; the rest are doubles.
    """

    throttle: float
    steering: float
    encoder_angles: tuple[float, float]
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]
    linear_velocity: tuple[float, float, float]
    lidar_scan_rate: float
    ranges: np.ndarray  # float32
    lap_count: int
    lap_time: float
    last_lap_time: float
    best_lap_time: float
    collision_count: int


def devkit_values(raw) -> DevkitValues | None:
    """One socket frame -> what the devkit would publish, or None.

    None is the devkit's `KeyError`: any missing key, the lidar-less first
    frame above all, means it publishes nothing and sends no reply.
    """
    try:
        text = gzip.decompress(base64.b64decode(raw[LIDAR_KEY])).decode()
        ranges = np.array(text.split(), dtype=np.float64).astype(np.float32)
        _ = raw["V1 Front Camera Image"]  # read, never decoded here
        return DevkitValues(
            throttle=_f32(float(raw["V1 Throttle"])),
            steering=_f32(float(raw["V1 Steering"])),
            encoder_angles=_vector(raw, "V1 Encoder Angles")[:2],
            position=_vector(raw, "V1 Position"),
            orientation=_vector(raw, "V1 Orientation Quaternion"),
            angular_velocity=_vector(raw, "V1 Angular Velocity"),
            linear_acceleration=_vector(raw, "V1 Linear Acceleration"),
            linear_velocity=_vector(raw, "V1 Linear Velocity"),
            lidar_scan_rate=float(raw["V1 LIDAR Scan Rate"]),
            ranges=ranges,
            lap_count=int(float(raw["V1 Lap Count"])),
            lap_time=_f32(float(raw["V1 Lap Time"])),
            last_lap_time=_f32(float(raw["V1 Last Lap Time"])),
            best_lap_time=_f32(float(raw["V1 Best Lap Time"])),
            collision_count=int(float(raw["V1 Collisions"])),
        )
    except KeyError:
        return None


def devkit_sample(values: DevkitValues) -> core.DevkitSample:
    """The values front end B reads back off the devkit's messages."""
    return core.DevkitSample(
        throttle=values.throttle,
        steering=values.steering,
        left_encoder=values.encoder_angles[0],
        right_encoder=values.encoder_angles[1],
        ips=values.position,
        imu_orientation=values.orientation,
        imu_angular_velocity=values.angular_velocity,
        imu_linear_acceleration=values.linear_acceleration,
        linear_velocity=values.linear_velocity,
        ranges=values.ranges,
        scan_time=1.0 / values.lidar_scan_rate,
        lap_count=values.lap_count,
        lap_time=values.lap_time,
        last_lap_time=values.last_lap_time,
        best_lap_time=values.best_lap_time,
        collisions=values.collision_count,
    )


def latch_command(data: float) -> float:
    """The devkit's command callbacks: `float(np.round(msg.data, 3))`."""
    return float(np.round(data, 3))


def devkit_reply(
    throttle_command: float, steering_command: float, reset_command: bool
) -> dict[str, str]:
    """The devkit's `emit('Bridge', ...)` payload: `str()` of each latch."""
    return {
        "V1 Throttle": str(throttle_command),
        "V1 Steering": str(steering_command),
        "V1 Reset": str(reset_command),
    }


def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _transforms(values: DevkitValues):
    """(child, parent, translation, quaternion) as in `broadcast_transforms`
    (autodrive_bridge.py:206-219): 11 transforms under `world`."""
    angles = values.encoder_angles
    steer = values.steering
    track, base = 2 * 0.141537, 2 * 0.0765
    tan = math.tan(steer)
    left = math.atan(track * tan / (track - base * tan))
    right = math.atan(track * tan / (track + base * tan))
    identity = (0.0, 0.0, 0.0, 1.0)
    return [
        ("roboracer_1", "world", values.position, values.orientation),
        (
            "left_encoder",
            "roboracer_1",
            (0.0, 0.12, 0.0),
            _quaternion_from_euler(0.0, 120 * angles[0] % 6.283, 0.0),
        ),
        (
            "right_encoder",
            "roboracer_1",
            (0.0, -0.12, 0.0),
            _quaternion_from_euler(0.0, 120 * angles[1] % 6.283, 0.0),
        ),
        ("ips", "roboracer_1", (0.08, 0.0, 0.055), identity),
        ("imu", "roboracer_1", (0.08, 0.0, 0.055), identity),
        ("lidar", "roboracer_1", (0.2733, 0.0, 0.096), identity),
        (
            "front_camera",
            "roboracer_1",
            (-0.015, 0.0, 0.15),
            (0.0, 0.0871557, 0.0, 0.9961947),
        ),
        (
            "front_left_wheel",
            "roboracer_1",
            (0.33, 0.118, 0.0),
            _quaternion_from_euler(0.0, 0.0, left),
        ),
        (
            "front_right_wheel",
            "roboracer_1",
            (0.33, -0.118, 0.0),
            _quaternion_from_euler(0.0, 0.0, right),
        ),
        (
            "rear_left_wheel",
            "roboracer_1",
            (0.0, 0.118, 0.0),
            _quaternion_from_euler(0.0, angles[0] % 6.283, 0.0),
        ),
        (
            "rear_right_wheel",
            "roboracer_1",
            (0.0, -0.118, 0.0),
            _quaternion_from_euler(0.0, angles[1] % 6.283, 0.0),
        ),
    ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--namespace", default=DEFAULT_NAMESPACE)
    parser.add_argument("--reset-topic", default=DEFAULT_RESET_TOPIC)
    parser.add_argument(
        "--tf-topic",
        default="/tf",
        help="where the 11 transforms go (the devkit: the global /tf)",
    )
    parser.add_argument(
        "--no-tf", action="store_true", help="publish no transforms"
    )
    parser.add_argument("--pace", choices=replay.PACES, default="recorded")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--count", type=int, default=None)
    parser.add_argument("--replies-out", type=Path, default=None)
    parser.add_argument(
        "--withhold",
        default=None,
        choices=PUBLISH_ORDER,
        help="publish everything but this topic (a stall)",
    )
    parser.add_argument(
        "--frame-id-lidar",
        default="lidar",
        help="the scan's frame id; only SIM-3720's negative control sets it",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    import rclpy
    from geometry_msgs.msg import Point, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from std_msgs.msg import Bool, Float32, Int32
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("autodrive_bridge")
    qos = QoSProfile(
        durability=DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    ns = args.namespace.rstrip("/")
    types = {
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
    pubs = {
        name: node.create_publisher(kind, f"{ns}/{name}", qos)
        for name, kind in types.items()
    }
    tf_pub = (
        None
        if args.no_tf
        else node.create_publisher(
            TFMessage,
            args.tf_topic,
            QoSProfile(
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
            ),
        )
    )
    latch = {"throttle": 0.0, "steering": 0.0, "reset": False}
    lock = threading.Lock()

    def on_throttle(msg):
        with lock:
            latch["throttle"] = latch_command(msg.data)

    def on_steering(msg):
        with lock:
            latch["steering"] = latch_command(msg.data)

    def on_reset(msg):
        with lock:
            latch["reset"] = bool(msg.data)

    node.create_subscription(
        Float32, f"{ns}/throttle_command", on_throttle, qos
    )
    node.create_subscription(
        Float32, f"{ns}/steering_command", on_steering, qos
    )
    node.create_subscription(Bool, args.reset_topic, on_reset, qos)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def stamp():
        return node.get_clock().now().to_msg()

    def joint(angle, name):
        msg = JointState()
        msg.header.stamp = stamp()
        msg.header.frame_id = name
        msg.name = [name]
        msg.position = [angle]
        return msg

    def message(name, v: DevkitValues):
        if name in ("throttle", "steering"):
            return Float32(data=getattr(v, name))
        if name == "left_encoder":
            return joint(v.encoder_angles[0], name)
        if name == "right_encoder":
            return joint(v.encoder_angles[1], name)
        if name == "ips":
            return Point(x=v.position[0], y=v.position[1], z=v.position[2])
        if name == "imu":
            msg = Imu()
            msg.header.stamp = stamp()
            msg.header.frame_id = "imu"
            msg.orientation.x, msg.orientation.y = v.orientation[:2]
            msg.orientation.z, msg.orientation.w = v.orientation[2:]
            msg.orientation_covariance = [
                0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025,
            ]  # fmt: skip
            msg.angular_velocity.x = v.angular_velocity[0]
            msg.angular_velocity.y = v.angular_velocity[1]
            msg.angular_velocity.z = v.angular_velocity[2]
            msg.angular_velocity_covariance = msg.orientation_covariance
            msg.linear_acceleration.x = v.linear_acceleration[0]
            msg.linear_acceleration.y = v.linear_acceleration[1]
            msg.linear_acceleration.z = v.linear_acceleration[2]
            msg.linear_acceleration_covariance = msg.orientation_covariance
            return msg
        if name == "odom":
            msg = Odometry()
            msg.header.stamp = stamp()
            msg.header.frame_id = "world"
            msg.child_frame_id = "roboracer_1"
            pose = msg.pose.pose
            pose.position.x, pose.position.y, pose.position.z = v.position
            pose.orientation.x, pose.orientation.y = v.orientation[:2]
            pose.orientation.z, pose.orientation.w = v.orientation[2:]
            twist = msg.twist.twist
            twist.linear.x, twist.linear.y, twist.linear.z = v.linear_velocity
            twist.angular.x = v.angular_velocity[0]
            twist.angular.y = v.angular_velocity[1]
            twist.angular.z = v.angular_velocity[2]
            diagonal = [0.0] * 36
            for i in range(6):
                diagonal[7 * i] = 0.0025
            msg.pose.covariance = diagonal
            msg.twist.covariance = diagonal
            return msg
        if name == "lidar":
            msg = LaserScan()
            msg.header.stamp = stamp()
            msg.header.frame_id = args.frame_id_lidar
            msg.angle_min = -2.35619
            msg.angle_max = 2.35619
            msg.angle_increment = 0.004363323
            msg.time_increment = (1 / v.lidar_scan_rate) / 1080
            msg.scan_time = 1 / v.lidar_scan_rate
            msg.range_min = 0.06
            msg.range_max = 10.0
            msg.ranges = v.ranges.tolist()
            return msg
        if name in ("lap_count", "collision_count"):
            return Int32(data=getattr(v, name))
        return Float32(data=getattr(v, name))

    def transforms(v: DevkitValues):
        msg = TFMessage()
        for child, parent, trans, quat in _transforms(v):
            tf = TransformStamped()
            tf.header.stamp = stamp()
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.translation.x = float(trans[0])
            tf.transform.translation.y = float(trans[1])
            tf.transform.translation.z = float(trans[2])
            tf.transform.rotation.x = float(quat[0])
            tf.transform.rotation.y = float(quat[1])
            tf.transform.rotation.z = float(quat[2])
            tf.transform.rotation.w = float(quat[3])
            msg.transforms.append(tf)
        return msg

    frames = replay.load_frames(args.fixture)
    out = args.replies_out.open("w") if args.replies_out else None
    sent = 0
    try:
        for index, delay, raw in replay.schedule(
            frames,
            args.pace,
            args.start_frame,
            args.loop,
            args.count,
            end_frame=args.end_frame,
        ):
            if delay:
                time.sleep(delay)
            values = devkit_values(raw)
            if values is None:
                continue  # the devkit's KeyError: nothing out, no reply
            for name in PUBLISH_ORDER:
                if name == "lidar" and tf_pub is not None:
                    tf_pub.publish(transforms(values))
                if name != args.withhold:
                    pubs[name].publish(message(name, values))
            with lock:
                reply = devkit_reply(
                    latch["throttle"], latch["steering"], latch["reset"]
                )
            sent += 1
            if out is not None:
                out.write(json.dumps({"frame": index, "reply": reply}) + "\n")
                out.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if out is not None:
            out.close()
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"devkit_standin: {sent} frames published", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
