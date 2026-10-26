"""Tier 1: the ROS-free AutoDRIVE core (SIMAD-*), against the part-1 fixture."""

import subprocess
import sys

from racing_sim_autodrive import control, core, lidar  # noqa: F401


def test_simad_1010_core_imports_without_rclpy():
    """SIMAD-1010: the modules import with no rclpy; CartesianPose is there."""
    code = (
        "import sys, racing_sim_autodrive.core, racing_sim_autodrive.lidar, "
        "racing_sim_autodrive.control; "
        "assert not any(m.startswith('rclpy') for m in sys.modules)"
    )
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr
    pose = core.Pose2D(1.0, 2.0, 0.5)
    assert (pose.x, pose.y, pose.psi) == (1.0, 2.0, 0.5)
