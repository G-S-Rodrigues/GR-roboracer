"""The RViz-only URDF agrees with the vehicle parameter file."""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf" / "f1tenth.urdf"
VEHICLE_CONFIG = (
    Path(__file__).resolve().parents[4]
    / "config"
    / "vehicles"
    / "f1tenth_default.yaml"
)


def _rotation(rpy: list[float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _origin_in(root: ET.Element, link: str, frame: str) -> np.ndarray:
    """Compose fixed joints from `link` up to `frame`; fail if broken."""
    position = np.zeros(3)
    current = link
    for _ in range(len(root.findall("joint")) + 1):
        if current == frame:
            return position
        joints = [
            j
            for j in root.findall("joint")
            if j.find("child").attrib["link"] == current
        ]
        assert len(joints) == 1, f"no single joint has child {current}"
        joint = joints[0]
        assert joint.attrib["type"] == "fixed", joint.attrib["name"]
        origin = joint.find("origin")
        attributes = origin.attrib if origin is not None else {}
        xyz = [float(v) for v in attributes.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in attributes.get("rpy", "0 0 0").split()]
        position = np.array(xyz) + _rotation(rpy) @ position
        current = joint.find("parent").attrib["link"]
    raise AssertionError(f"{link} does not reach {frame}")


def test_desc_1010_wheel_axles_are_wheelbase_apart() -> None:
    """DESC-1010: RViz must show the car the sim drives.

    The controller's wheelbase comes from the vehicle file; the URDF's
    front wheel origin must be that far ahead of the rear one, per side.
    """
    config = yaml.safe_load(VEHICLE_CONFIG.read_text())
    wheelbase = config["racing_controller_baseline"]["ros__parameters"][
        "wheelbase"
    ]
    root = ET.parse(URDF_PATH).getroot()
    for side in ("left", "right"):
        rear = _origin_in(root, f"{side}_rear_wheel", "base_link")
        front = _origin_in(root, f"{side}_front_wheel", "base_link")
        offset = front - rear
        assert offset == pytest.approx([wheelbase, 0.0, 0.0], abs=1e-6), (
            f"{side}: front axle is {offset.round(4).tolist()} m from the "
            f"rear axle, vehicle file says [{wheelbase}, 0.0, 0.0] "
            "(forward by the wheelbase, level and on the same side)"
        )


def test_desc_1020_the_drawing_is_authored_in_the_rear_axle_frame() -> None:
    """DESC-1020: base_link is the CoG; the meshes sit in `rear_axle`.

    Every mesh was generated in rear-axle coordinates with an identity
    joint, so moving base_link to the centre of gravity must not move one
    of them: base_link carries no visual, and each mesh-bearing link except
    the laser and the wheels composes to rear_axle as the identity (the
    steering hinges carry no visual). The rear wheels' midpoint is the axle,
    one wheel radius above the ground point rear_axle names.
    """
    root = ET.parse(URDF_PATH).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    assert links["base_link"].find("visual") is None, "base_link has a visual"
    not_identity = {
        "lidar",
        "left_rear_wheel",
        "right_rear_wheel",
        "left_front_wheel",
        "right_front_wheel",
    }
    drawn = [
        name
        for name, link in links.items()
        if link.find("visual") is not None and name not in not_identity
    ]
    assert "rear_axle" in drawn, "the chassis visual is not on rear_axle"
    for name in drawn:
        offset = _origin_in(root, name, "rear_axle")
        assert offset == pytest.approx([0.0, 0.0, 0.0], abs=1e-9), (
            f"{name} is {offset.round(4).tolist()} m from rear_axle; its mesh "
            "is authored there"
        )
    left = _origin_in(root, "left_rear_wheel", "rear_axle")
    right = _origin_in(root, "right_rear_wheel", "rear_axle")
    assert (left + right) / 2 == pytest.approx([0.0, 0.0, 0.058], abs=1e-9)
