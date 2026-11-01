"""Structural tests for the RViz-only F1TENTH description."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PACKAGE_ROOT = Path(__file__).parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf" / "f1tenth.urdf"
DESCRIPTIONS_PATH = PACKAGE_ROOT / "urdf" / "descriptions.yaml"
RVIZ_PATH = PACKAGE_ROOT / "rviz" / "sim_pure_pursuit.rviz"


def _mapped() -> dict[str, Path]:
    mapping = yaml.safe_load(DESCRIPTIONS_PATH.read_text(encoding="utf-8"))
    return {
        name: PACKAGE_ROOT / "urdf" / name
        for name in sorted(set(mapping.values()))
    }


MAPPED = _mapped()
REQUIRED_LINKS = {
    "base_link",
    "rear_axle",
    "lidar",
    "left_rear_wheel",
    "right_rear_wheel",
    "left_front_wheel",
    "right_front_wheel",
}


@pytest.fixture(params=sorted(MAPPED), ids=sorted(MAPPED))
def urdf_path(request: pytest.FixtureRequest) -> Path:
    path = MAPPED[request.param]
    assert path.is_file(), f"descriptions.yaml names {path}, which is missing"
    return path


def test_urdf_has_canonical_vehicle_and_laser_frames(urdf_path: Path) -> None:
    """DESC-1030: each URDF in urdf/descriptions.yaml connects lidar to
    rear_axle with a fixed joint and has the canonical frames."""
    root = ET.parse(urdf_path).getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    assert REQUIRED_LINKS.issubset(links), sorted(REQUIRED_LINKS - links)
    laser_joint = root.find("./joint[@name='rear_axle_to_lidar']")
    assert laser_joint is not None
    assert laser_joint.attrib["type"] == "fixed"
    assert laser_joint.find("parent").attrib["link"] == "rear_axle"
    assert laser_joint.find("child").attrib["link"] == "lidar"
    assert root.find("./joint[@name='base_link_to_rear_axle']") is not None
    for joint in root.findall("joint"):
        assert joint.attrib["type"] == "fixed", joint.attrib["name"]


def test_urdf_is_visualization_only(urdf_path: Path) -> None:
    """DESC-1030: no URDF smuggles Gazebo or sensor plugins into scope."""
    source = urdf_path.read_text(encoding="utf-8").lower()
    assert "<gazebo" not in source
    assert "<plugin" not in source
    assert "<sensor" not in source
    assert "transmission" not in source


def test_f1tenth_lidar_mount_is_unchanged() -> None:
    """The rename must not move the f1tenth mount (rear_axle -> lidar)."""
    root = ET.parse(URDF_PATH).getroot()
    laser_joint = root.find("./joint[@name='rear_axle_to_lidar']")
    assert laser_joint.attrib["type"] == "fixed"
    origin = laser_joint.find("origin")
    assert origin.attrib["xyz"] == "0.275 0.0 0.105"
    assert origin.attrib["rpy"] == "0 0 0"


def test_every_referenced_mesh_exists(urdf_path: Path) -> None:
    """A missing mesh is invisible: RViz draws nothing and logs nothing loud.

    The URDF once referenced 21 meshes while 4 were on disk, and every other
    test here still passed — robot_state_publisher does not load geometry, so
    only a human looking at RViz would have noticed. DESC-1030 runs this over
    every URDF in the map.
    """
    root = ET.parse(urdf_path).getroot()
    references = {
        mesh.attrib["filename"]
        for mesh in root.iter("mesh")
        if "filename" in mesh.attrib
    }
    assert references, "the URDF references no meshes at all"

    missing = sorted(
        reference
        for reference in references
        if not (
            PACKAGE_ROOT / reference.split("racing_vehicle_description/")[-1]
        ).is_file()
    )
    assert missing == [], f"URDF references meshes that do not exist: {missing}"


def test_rviz_shows_each_required_live_artifact() -> None:
    """RViz is configured for robot, scan, track, and planned trajectory."""
    source = RVIZ_PATH.read_text(encoding="utf-8")
    assert "Fixed Frame: map" in source
    assert "rviz_default_plugins/RobotModel" in source
    assert "rviz_default_plugins/LaserScan" in source
    assert "Reliability Policy: Best Effort" in source
    assert "Value: /scan" in source
    assert "rviz_default_plugins/MarkerArray" in source
    assert "Value: /visualization/track" in source
    assert "Value: /visualization/trajectory" in source


def test_rviz_receives_every_latched_topic() -> None:
    """/map and both marker topics are published latched, once or rarely.

    A Volatile display never receives a message published before it
    subscribed, so each needs Transient Local or it stays empty with no
    error (repo-gotchas #6).
    """
    source = RVIZ_PATH.read_text(encoding="utf-8")
    assert "rviz_default_plugins/Map" in source
    for topic in ("/map", "/visualization/track", "/visualization/trajectory"):
        block = source.split(f"Value: {topic}\n")[0].rsplit("Topic:", 1)[-1]
        assert "Durability Policy: Transient Local" in block, topic


def test_rviz_camera_follows_the_vehicle() -> None:
    """A league track does not start at the origin: Spielberg's car starts
    ~50 m from it, outside a camera orbiting (0, 0)."""
    source = RVIZ_PATH.read_text(encoding="utf-8")
    assert "Target Frame: base_link" in source


def test_manual_tf_sign_check_is_explicitly_documented() -> None:
    """The numeric-test blind spot is retained as an explicit manual gate."""
    instructions = (PACKAGE_ROOT / "CLAUDE.md").read_text(encoding="utf-8")
    assert "Manual TF-sign check" in instructions
    assert "left turn" in instructions
    assert "LiDAR" in instructions
