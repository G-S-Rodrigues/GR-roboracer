"""Support node: publishes the trajectory, track-boundary and TF outputs no
other node in the vertical slice owns.

``racing_sim_gym_jax`` publishes ``/odom`` but never a TF, so RViz has no way
to place the robot in ``map``. Nothing publishes the raceline the controller
needs on ``/trajectory``, and nothing publishes the track geometry RViz needs
to draw. This node is the thin, ROS-only home for those three gaps.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Point, Point32, TransformStamped
from nav_msgs.msg import Odometry
from racing_interfaces.msg import TrackBoundaries, Trajectory, TrajectoryPoint
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _load_centerline(
    track_path: Path,
) -> list[tuple[float, float, float, float]]:
    document = yaml.safe_load(track_path.read_text(encoding="utf-8"))
    return [(row[0], row[1], row[3], row[4]) for row in document["centerline"]]


def _boundaries_from_centerline(
    centerline: list[tuple[float, float, float, float]],
) -> tuple[list[Point32], list[Point32]]:
    """Offset each centerline vertex by its recorded left/right width."""
    left: list[Point32] = []
    right: list[Point32] = []
    count = len(centerline)
    for index, (x, y, left_width, right_width) in enumerate(centerline):
        prev_x, prev_y, _, _ = centerline[index - 1]
        next_x, next_y, _, _ = centerline[(index + 1) % count]
        tangent_x = next_x - prev_x
        tangent_y = next_y - prev_y
        length = math.hypot(tangent_x, tangent_y)
        normal_x = -tangent_y / length if length else 0.0
        normal_y = tangent_x / length if length else 0.0
        left.append(
            Point32(
                x=x + normal_x * left_width,
                y=y + normal_y * left_width,
                z=0.0,
            )
        )
        right.append(
            Point32(
                x=x - normal_x * right_width,
                y=y - normal_y * right_width,
                z=0.0,
            )
        )
    return left, right


def _load_raceline(raceline_path: Path) -> list[TrajectoryPoint]:
    points: list[TrajectoryPoint] = []
    with raceline_path.open(encoding="utf-8", newline="") as handle:
        for row in csv.reader(handle, delimiter=";"):
            s, x, y, psi, kappa, v, a = (float(value) for value in row)
            point = TrajectoryPoint()
            point.s = s
            point.x = x
            point.y = y
            point.psi = psi
            point.kappa = kappa
            point.v = v
            point.a = a
            points.append(point)
    return points


def _boundary_marker(
    marker_id: int, name: str, points: list[Point32], frame_id: str
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = f"track_boundary_{name}"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.color.a = 1.0
    marker.color.r = 1.0 if name == "right" else 0.0
    marker.color.g = 1.0 if name == "left" else 0.0
    closed = [*points, points[0]] if points else points
    marker.points = [Point(x=p.x, y=p.y, z=p.z) for p in closed]
    return marker


def _trajectory_marker(points: list[TrajectoryPoint], frame_id: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "trajectory"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.03
    marker.color.a = 1.0
    marker.color.b = 1.0
    closed = [*points, points[0]] if points else points
    marker.points = [Point(x=p.x, y=p.y, z=0.0) for p in closed]
    return marker


class SupportNode(Node):
    """Publishes /trajectory and /track/boundaries once, latched, and
    broadcasts the map -> base_link TF that follows /odom."""

    def __init__(self) -> None:
        super().__init__("racing_bringup_support")
        self.declare_parameter(
            "track_path", "config/tracks/analytic_circle.yaml"
        )
        self.declare_parameter(
            "raceline_path",
            "config/scenarios/maps/analytic_circle/analytic_circle_raceline.csv",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        track_path = Path(
            self.get_parameter("track_path").get_parameter_value().string_value
        )
        raceline_path = Path(
            self.get_parameter("raceline_path")
            .get_parameter_value()
            .string_value
        )
        self._map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )
        self._base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )

        self._trajectory_publisher = self.create_publisher(
            Trajectory, "/trajectory", LATCHED_QOS
        )
        self._boundaries_publisher = self.create_publisher(
            TrackBoundaries, "/track/boundaries", LATCHED_QOS
        )
        self._track_marker_publisher = self.create_publisher(
            MarkerArray, "/visualization/track", LATCHED_QOS
        )
        self._trajectory_marker_publisher = self.create_publisher(
            MarkerArray, "/visualization/trajectory", LATCHED_QOS
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, "/odom", self._on_odometry, LATCHED_QOS.depth
        )

        self._publish_static_artifacts(track_path, raceline_path)

    def _publish_static_artifacts(
        self, track_path: Path, raceline_path: Path
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        centerline = _load_centerline(track_path)
        left, right = _boundaries_from_centerline(centerline)
        boundaries = TrackBoundaries()
        boundaries.header.stamp = stamp
        boundaries.header.frame_id = self._map_frame
        boundaries.source = track_path.stem
        boundaries.left = left
        boundaries.right = right
        self._boundaries_publisher.publish(boundaries)

        track_markers = MarkerArray()
        track_markers.markers.append(
            _boundary_marker(0, "left", left, self._map_frame)
        )
        track_markers.markers.append(
            _boundary_marker(1, "right", right, self._map_frame)
        )
        self._track_marker_publisher.publish(track_markers)

        raceline = _load_raceline(raceline_path)
        trajectory = Trajectory()
        trajectory.header.stamp = stamp
        trajectory.header.frame_id = self._map_frame
        trajectory.source = raceline_path.stem
        trajectory.valid_until = stamp
        trajectory.points = raceline
        self._trajectory_publisher.publish(trajectory)

        trajectory_markers = MarkerArray()
        trajectory_markers.markers.append(
            _trajectory_marker(raceline, self._map_frame)
        )
        self._trajectory_marker_publisher.publish(trajectory_markers)

    def _on_odometry(self, message: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
