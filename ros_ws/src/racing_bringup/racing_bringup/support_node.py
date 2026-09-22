"""Support node: publishes the trajectory, track-boundary and TF outputs no
other node in the vertical slice owns.

``racing_sim_gym_jax`` broadcasts ``odom -> base_link`` from its dead
reckoning, but ``map -> odom`` belongs to whichever pose source is active;
with ground truth as that source (the only one so far) nobody else owns
it. Nothing publishes the raceline the controller needs on ``/trajectory``,
and nothing publishes the track geometry RViz needs to draw. This node is
the thin, ROS-only home for those three gaps.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import (
    Point,
    Point32,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from nav_msgs.msg import Odometry
from racing_interfaces.msg import TrackBoundaries, Trajectory, TrajectoryPoint
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)
from visualization_msgs.msg import Marker, MarkerArray

LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# The covariance published with the initial pose: 0.5 m in x and y, 15 deg
# in yaw (nav2's own defaults for a pose set by hand). It sizes nav2_amcl's
# opening particle cloud, and it is deliberately not zero - this stands in
# for RViz's "2D Pose Estimate", a human putting the car roughly where it
# is, not ground truth handed to the estimator exactly. A zero covariance
# would start AMCL at the truth and make its first seconds unmeasurable.
INITIAL_POSE_VARIANCES = (0.25, 0.25, 0.06853891945200942)


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


def _planar_pose(message: Odometry) -> tuple[float, float, float]:
    pose = message.pose.pose
    q = pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2)
    )
    return pose.position.x, pose.position.y, yaw


def _planar_correction(
    truth: tuple[float, float, float],
    dead_reckoning: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return map -> odom such that (map -> odom) * dead_reckoning == truth."""
    yaw = truth[2] - dead_reckoning[2]
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    x = truth[0] - (cos_yaw * dead_reckoning[0] - sin_yaw * dead_reckoning[1])
    y = truth[1] - (sin_yaw * dead_reckoning[0] + cos_yaw * dead_reckoning[1])
    return x, y, yaw


def _planar_compose(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return the planar pose ``first * second`` (BRINGUP-1040)."""
    cos_yaw, sin_yaw = math.cos(first[2]), math.sin(first[2])
    return (
        first[0] + cos_yaw * second[0] - sin_yaw * second[1],
        first[1] + sin_yaw * second[0] + cos_yaw * second[1],
        first[2] + second[2],
    )


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
    broadcasts the ground-truth pose source's map -> odom TF."""

    def __init__(self, **kwargs) -> None:
        super().__init__("racing_bringup_support", **kwargs)
        self.declare_parameter(
            "track_path", "config/tracks/analytic_circle.yaml"
        )
        self.declare_parameter(
            "raceline_path",
            "config/scenarios/maps/analytic_circle/analytic_circle_raceline.csv",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        # Ground truth is the only pose source that does not own map -> odom
        # itself. An estimator (slam_toolbox, nav2_amcl) does, so this node
        # must stop broadcasting it: two owners of one TF edge (BRINGUP-2010).
        self.declare_parameter("publish_map_to_odom", True)
        # Non-empty under an estimator: publish, per /odom, the latest
        # map -> odom composed with that dead reckoning - an Odometry in
        # map at control rate, which is what the controller consumes.
        self.declare_parameter("composed_odometry_topic", "")
        # Non-empty for an estimator that has to be told where it starts
        # (nav2_amcl). The start pose is drawn from the run's seed
        # (f110_env.reset places the car at a random arc length), so it
        # cannot be a configured constant; the first ground-truth pose is
        # published here once, latched, and never again - re-seeding a
        # particle filter mid-lap would hide the divergence the run
        # measures. slam_toolbox needs none: it defines `map` as where it
        # starts.
        self.declare_parameter("initial_pose_topic", "")

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
        self._odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
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
        self._publish_map_to_odom = (
            self.get_parameter("publish_map_to_odom")
            .get_parameter_value()
            .bool_value
        )
        composed_topic = (
            self.get_parameter("composed_odometry_topic")
            .get_parameter_value()
            .string_value
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._composed_publisher = None
        if composed_topic:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._composed_publisher = self.create_publisher(
                Odometry, composed_topic, LATCHED_QOS.depth
            )
        initial_pose_topic = (
            self.get_parameter("initial_pose_topic")
            .get_parameter_value()
            .string_value
        )
        self._initial_pose_publisher = (
            self.create_publisher(
                PoseWithCovarianceStamped, initial_pose_topic, LATCHED_QOS
            )
            if initial_pose_topic
            else None
        )
        self._initial_pose_sent = False
        self._latest_truth: Odometry | None = None
        self._latest_dead_reckoning: Odometry | None = None
        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self._on_ground_truth,
            LATCHED_QOS.depth,
        )
        self.create_subscription(
            Odometry, "/odom", self._on_dead_reckoning, LATCHED_QOS.depth
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

    def _on_ground_truth(self, message: Odometry) -> None:
        self._latest_truth = message
        self._publish_initial_pose(message)
        self._publish_correction()

    def _publish_initial_pose(self, truth: Odometry) -> None:
        """Publish the start pose once, latched, for an estimator that needs
        one. Latched, so it reaches nav2_amcl however late it activates."""
        if self._initial_pose_publisher is None or self._initial_pose_sent:
            return
        initial = PoseWithCovarianceStamped()
        initial.header.stamp = truth.header.stamp
        initial.header.frame_id = self._map_frame
        initial.pose.pose = truth.pose.pose
        x_var, y_var, yaw_var = INITIAL_POSE_VARIANCES
        initial.pose.covariance[0] = x_var
        initial.pose.covariance[7] = y_var
        initial.pose.covariance[35] = yaw_var
        # The publisher itself stays alive - a transient-local message is
        # only delivered to a late subscriber while its publisher exists.
        self._initial_pose_publisher.publish(initial)
        self._initial_pose_sent = True

    def _on_dead_reckoning(self, message: Odometry) -> None:
        self._latest_dead_reckoning = message
        self._publish_correction()
        self._publish_composed(message)

    def _publish_composed(self, dead_reckoning: Odometry) -> None:
        """Publish (latest map -> odom) * dead reckoning, in map.

        The same chain as map -> odom -> base_link in TF, so the pose the
        controller drives on cannot disagree with what RViz shows. Nothing
        is published until the estimator has published map -> odom: no pose
        is better than one in the wrong frame.
        """
        if self._composed_publisher is None:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._odom_frame, Time()
            )
        except TransformException:
            return
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        translation = transform.transform.translation
        x, y, heading = _planar_compose(
            (translation.x, translation.y, yaw),
            _planar_pose(dead_reckoning),
        )
        composed = Odometry()
        composed.header.stamp = dead_reckoning.header.stamp
        composed.header.frame_id = self._map_frame
        composed.child_frame_id = dead_reckoning.child_frame_id
        composed.pose.pose.position.x = x
        composed.pose.pose.position.y = y
        composed.pose.pose.orientation.z = math.sin(heading / 2.0)
        composed.pose.pose.orientation.w = math.cos(heading / 2.0)
        # The twist is in the body frame, which map -> odom does not move.
        composed.twist = dead_reckoning.twist
        self._composed_publisher.publish(composed)

    def _publish_correction(self) -> None:
        """Broadcast map -> odom = truth * dead_reckoning^-1, per stamp.

        The correction, not the ground-truth pose: the sim node already
        broadcasts odom -> base_link from its drifting dead reckoning, so
        publishing the pose here would double-count it and the vehicle would
        sit at roughly twice its displacement in RViz; identity would leave
        it on the drifting estimate. Both are invisible to every numeric
        test (repo-gotchas #16). The two poses are paired by stamp, so the
        composed map -> base_link is exactly the ground-truth pose.
        """
        if not self._publish_map_to_odom:
            return
        truth = self._latest_truth
        dead_reckoning = self._latest_dead_reckoning
        if truth is None or dead_reckoning is None:
            return
        if truth.header.stamp != dead_reckoning.header.stamp:
            return
        x, y, yaw = _planar_correction(
            _planar_pose(truth), _planar_pose(dead_reckoning)
        )
        transform = TransformStamped()
        transform.header.stamp = truth.header.stamp
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
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
