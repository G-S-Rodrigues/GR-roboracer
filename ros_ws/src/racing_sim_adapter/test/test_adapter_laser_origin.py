"""Tier-2: /scan is ray-marched from the laser, /ground_truth/scan is not.

A separate launch from `test_adapter_contract.py`: it needs the backend
launched with the vehicle file, which carries the laser mount, and held, so
every sample is taken at one known pose.
"""

import math
import statistics
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan

ADAPTER_NODE = "racing_sim"
VEHICLE_CONFIG = (
    Path(__file__).resolve().parents[4]
    / "config"
    / "vehicles"
    / "f1tenth_default.yaml"
)
SAMPLE_COUNT = 20

SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _mount_x() -> float:
    document = yaml.safe_load(VEHICLE_CONFIG.read_text(encoding="utf-8"))
    return float(document[ADAPTER_NODE]["ros__parameters"]["laser_x"])


@pytest.mark.launch_test
def generate_test_description():
    backend = launch_ros.actions.Node(
        package="racing_sim_gym_jax",
        executable="racing_sim_gym_jax_node",
        parameters=[str(VEHICLE_CONFIG), {"start_held": True}],
        output="screen",
    )
    return launch.LaunchDescription(
        [backend, launch_testing.actions.ReadyToTest()]
    )


def _forward_range(scan: LaserScan) -> float:
    """The range at angle 0, interpolated between the two beams either side
    of it at the angles the message declares. Not the two middle indices:
    /scan and /ground_truth/scan are cast at different spacings (plan D6),
    so the same index is not the same direction in both."""
    position = -scan.angle_min / scan.angle_increment
    below = int(position)
    weight = position - below
    return (1.0 - weight) * scan.ranges[below] + weight * scan.ranges[below + 1]


class TestAdapterLaserOrigin(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("racing_sim_adapter_laser_origin_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _collect(self, topic: str) -> list[LaserScan]:
        messages: list[LaserScan] = []
        subscription = self.node.create_subscription(
            LaserScan, topic, messages.append, SENSOR_QOS
        )
        deadline = time.monotonic() + 20.0
        try:
            while len(messages) < SAMPLE_COUNT and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        finally:
            self.node.destroy_subscription(subscription)
        self.assertGreaterEqual(len(messages), SAMPLE_COUNT, f"no {topic}")
        return messages

    def test_adapt_2120_scan_is_cast_from_the_laser_mount(self):
        """ADAPT-2120: straight ahead, /scan reads the laser mount's
        forward offset less than /ground_truth/scan.

        Held, the vehicle sits at one pose, and the laser lies on the
        forward ray 0.275 m ahead of base_link, so both rays hit the same
        wall point and the ranges differ by exactly the mount offset. A scan
        cast from base_link but stamped `laser` would read the same range as
        ground truth - and TF would place every point 0.275 m too far
        forward. /ground_truth/scan stays cast from base_link: it is what
        racing_metrics' clearance and collisions read, and the goldens
        depend on it. The median over held samples removes /scan's noise.

        Measured at the held contract_test pose, each scan read at angle 0
        by its own declared angles: 0.2713 m (the map is a raster); cast
        from base_link, /scan read 0.0052 m. Before plan D6 the two middle
        beams were compared by index and read 0.2876 m, but the gym's beams
        sit at -0.0004/+0.075 rad, not the declared +/-0.037. The 0.05 m
        delta is ~14x the error and far from 0.0052.
        """
        truth = self._collect("/ground_truth/scan")
        noisy = self._collect("/scan")
        self.assertEqual(noisy[0].header.frame_id, "laser")
        # Labelled where it was cast from, so the label is honest.
        self.assertEqual(truth[0].header.frame_id, "base_link")

        truth_range = statistics.median(_forward_range(m) for m in truth)
        laser_range = statistics.median(_forward_range(m) for m in noisy)

        self.assertTrue(math.isfinite(truth_range))
        self.assertAlmostEqual(
            truth_range - laser_range, _mount_x(), delta=0.05
        )
