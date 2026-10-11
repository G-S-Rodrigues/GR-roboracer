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


def _origin_in_base_link(root: ET.Element, link: str) -> np.ndarray:
    """Compose fixed joints from `link` up to base_link; fail if broken."""
    position = np.zeros(3)
    current = link
    for _ in range(len(root.findall("joint")) + 1):
        if current == "base_link":
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
        xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
        position = np.array(xyz) + _rotation(rpy) @ position
        current = joint.find("parent").attrib["link"]
    raise AssertionError(f"{link} does not reach base_link")


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
        rear = _origin_in_base_link(root, f"{side}_rear_wheel")
        front = _origin_in_base_link(root, f"{side}_front_wheel")
        offset = front - rear
        assert offset == pytest.approx([wheelbase, 0.0, 0.0], abs=1e-6), (
            f"{side}: front axle is {offset.round(4).tolist()} m from the "
            f"rear axle, vehicle file says [{wheelbase}, 0.0, 0.0] "
            "(forward by the wheelbase, level and on the same side)"
        )
