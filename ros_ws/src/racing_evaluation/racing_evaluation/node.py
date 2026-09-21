"""Thin ROS shell: score a live pose estimate against ground truth.

Publishes one ``racing_interfaces/LocalizationError`` per ground-truth sample
on ``/evaluation/localization_error``. Every decision - alignment, staleness,
the error measures - lives in ``racing_evaluation.localization``.
"""

from __future__ import annotations

import math

import racing_common
import rclpy
from nav_msgs.msg import Odometry
from racing_interfaces.msg import LocalizationError
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from racing_evaluation.localization import (
    DEFAULT_WINDOW,
    AlignedPair,
    Aligner,
    Pose,
    RunningError,
    lateral_longitudinal_heading_error,
    position_error,
)

ERROR_TOPIC = "/evaluation/localization_error"
GROUND_TRUTH_TOPIC = "/ground_truth/odom"

# Ground truth comes from the simulator's reliable state stream.
TRUTH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# Best effort matches a reliable *or* a best-effort publisher (repo-gotchas
# #6): the estimate is whatever a pose source publishes, and a QoS mismatch
# would score a working estimator as permanently unavailable.
ESTIMATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
ERROR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def pose_from_odometry(message: Odometry) -> Pose:
    stamp = message.header.stamp
    position = message.pose.pose.position
    q = message.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    return Pose(
        stamp=stamp.sec + stamp.nanosec * 1e-9,
        x=position.x,
        y=position.y,
        yaw=yaw,
    )


class EvaluationNode(Node):
    def __init__(self) -> None:
        super().__init__("racing_evaluation")
        self._estimate_topic = self.declare_parameter(
            "estimate_topic", GROUND_TRUTH_TOPIC
        ).value
        truth_topic = self.declare_parameter(
            "ground_truth_topic", GROUND_TRUTH_TOPIC
        ).value
        track_path = self.declare_parameter("track_path", "").value
        window = self.declare_parameter("window", DEFAULT_WINDOW).value
        if not track_path:
            raise ValueError("track_path is required")
        self._track = racing_common.Track.from_yaml(track_path)
        self._aligner = Aligner(window)
        self._truth_frame = ""
        self._running = RunningError()

        self._publisher = self.create_publisher(
            LocalizationError, ERROR_TOPIC, ERROR_QOS
        )
        self._truth_subscription = self.create_subscription(
            Odometry, truth_topic, self._on_truth, TRUTH_QOS
        )
        self._estimate_subscription = self.create_subscription(
            Odometry, self._estimate_topic, self._on_estimate, ESTIMATE_QOS
        )

    def _on_truth(self, message: Odometry) -> None:
        self._truth_frame = message.header.frame_id
        self._publish(self._aligner.add_truth(pose_from_odometry(message)))

    def _on_estimate(self, message: Odometry) -> None:
        self._publish(self._aligner.add_estimate(pose_from_odometry(message)))

    def _publish(self, pairs: list[AlignedPair]) -> None:
        for pair in pairs:
            self._publisher.publish(self._score(pair))

    def _score(self, pair: AlignedPair) -> LocalizationError:
        message = LocalizationError()
        seconds = math.floor(pair.truth.stamp)
        message.header.stamp.sec = int(seconds)
        message.header.stamp.nanosec = min(
            round((pair.truth.stamp - seconds) * 1e9), 999_999_999
        )
        message.header.frame_id = self._truth_frame
        message.source = self.get_name()
        message.estimate_topic = self._estimate_topic
        message.available = pair.estimate is not None
        if pair.estimate is None:
            message.position_error = math.nan
            message.lateral_error = math.nan
            message.longitudinal_error = math.nan
            message.heading_error = math.nan
            self._running.add(None)
        else:
            error = position_error(pair.estimate, pair.truth)
            component = lateral_longitudinal_heading_error(
                pair.estimate, pair.truth, self._track
            )
            message.position_error = error
            message.lateral_error = component.lateral
            message.longitudinal_error = component.longitudinal
            message.heading_error = component.heading
            self._running.add(error)
        summary = self._running.summary()
        message.sample_count = summary.sample_count
        message.available_count = summary.scored_count
        message.position_rmse = summary.rmse
        message.position_maximum = summary.maximum
        return message


def main() -> None:
    rclpy.init()
    node = EvaluationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
