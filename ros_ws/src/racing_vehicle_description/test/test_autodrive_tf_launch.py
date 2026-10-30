"""DESC-2010: robot_state_publisher takes the AutoDRIVE URDF; TF is right.

Reads the URDF through the package's own urdf/descriptions.yaml (not
racing_bringup: that would close a colcon dependency cycle).
"""

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
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener

SHARE = Path(get_package_share_directory("racing_vehicle_description"))
MAPPING = yaml.safe_load((SHARE / "urdf" / "descriptions.yaml").read_text())
URDF = SHARE / "urdf" / MAPPING["autodrive_roboracer"]

# base_link -> frame, from lr = 0.15532, h = 0.07354, axle centre 0.0592,
# lidar 0.2733 / 0.096 above the axle centre.
EXPECTED = {
    "lidar": (0.11798, 0.0, 0.08166),
    "left_rear_wheel": (-0.15532, 0.118, -0.01434),
    "right_rear_wheel": (-0.15532, -0.118, -0.01434),
    "left_front_wheel": (0.17468, 0.118, -0.00434),
    "right_front_wheel": (0.17468, -0.118, -0.00434),
}


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": URDF.read_text()}],
    )
    return launch.LaunchDescription(
        [node, launch_testing.actions.ReadyToTest()]
    )


class TestAutodriveTf(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("desc_2010")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_desc_2010_tf_matches_the_autodrive_car(self):
        buffer = Buffer()
        TransformListener(
            buffer,
            self.node,
            spin_thread=False,
            static_qos=QoSProfile(
                depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if buffer.can_transform("base_link", "lidar", rclpy.time.Time()):
                break
        for frame, want in EXPECTED.items():
            tf = buffer.lookup_transform("base_link", frame, rclpy.time.Time())
            got = tf.transform.translation
            for g, w, axis in zip(
                (got.x, got.y, got.z), want, "xyz", strict=True
            ):
                self.assertAlmostEqual(g, w, delta=1e-6, msg=f"{frame} {axis}")
