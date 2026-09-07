"""Structural tests for the RViz-only F1TENTH description."""

import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE_ROOT = Path(__file__).parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf" / "f1tenth.urdf"
RVIZ_PATH = PACKAGE_ROOT / "rviz" / "sim_pure_pursuit.rviz"


def test_urdf_has_canonical_vehicle_and_laser_frames() -> None:
    """The description connects laser to base_link with a fixed joint."""
    root = ET.parse(URDF_PATH).getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    assert {"base_link", "laser"}.issubset(links)

    laser_joint = root.find("./joint[@name='base_link_to_laser']")
    assert laser_joint is not None
    assert laser_joint.attrib["type"] == "fixed"
    assert laser_joint.find("parent").attrib["link"] == "base_link"
    assert laser_joint.find("child").attrib["link"] == "laser"


def test_urdf_is_visualization_only() -> None:
    """The description does not smuggle Gazebo or sensor plugins into scope."""
    source = URDF_PATH.read_text(encoding="utf-8").lower()
    assert "<gazebo" not in source
    assert "<plugin" not in source
    assert "<sensor" not in source
    assert "transmission" not in source


def test_rviz_shows_each_required_live_artifact() -> None:
    """RViz is configured for robot, scan, track, and planned trajectory."""
    source = RVIZ_PATH.read_text(encoding="utf-8")
    assert "Fixed Frame: map" in source
    assert "rviz_default_plugins/RobotModel" in source
    assert "rviz_default_plugins/LaserScan" in source
    assert "Topic: /scan" in source
    assert "rviz_default_plugins/MarkerArray" in source
    assert "Topic: /visualization/track" in source
    assert "Topic: /visualization/trajectory" in source


def test_manual_tf_sign_check_is_explicitly_documented() -> None:
    """The numeric-test blind spot is retained as an explicit manual gate."""
    instructions = (PACKAGE_ROOT / "CLAUDE.md").read_text(encoding="utf-8")
    assert "Manual TF-sign check" in instructions
    assert "left turn" in instructions
    assert "LiDAR" in instructions
