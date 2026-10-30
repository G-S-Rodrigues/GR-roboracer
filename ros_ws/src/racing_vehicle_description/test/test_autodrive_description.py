"""The AutoDRIVE RoboRacer description matches AutoDRIVE's car.

Numbers come from config/vehicles/autodrive_roboracer/{source,identified}.yaml
and the vehicle file, never copied into the tests. The meshes are measured
from the committed STLs composed through the URDF's joints.

Tolerances (gotcha 14 spirit: written once, never widened to green a run).
Each is: measured on the committed files / the tolerance / the smallest real
error it must still catch. The errors to catch: the prefab axle height 0.05
instead of the rest height 0.0592 (9.2 mm in z); a dropped `Vehicle` node
offset (40 mm in y, 83 mm in x); a left/right flip (>= 80 mm at a named
wheel); mm read as m. Every tolerance sits below 9.2 mm.

Re-measured 2026-10-30 on the committed STLs (measured / tolerance):
- wheel centre x, y: 5.17 mm (right_rear y) / 6 mm; catches 9.2 mm and up.
- wheel centre z: 0.66 mm / 2 mm; catches 9.2 mm.
- tyre radius: 2.0 mm under the collider's 0.059 / 2.5 mm.
- front overhang: 4.76 mm past 0.09; rear edge (0.05 % quantile of x) 2.14
  mm inside 0.08; length 2.61 mm over 0.500 / 5 mm each.
- chassis lateral centre: 1.75 mm / 3 mm.
- lidar horizontal centre: 6.12 mm (x), 1.75 mm (y) off the frame origin /
  8 mm.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from test_urdf_matches_vehicle import _origin_in

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
URDF_PATH = PACKAGE_ROOT / "urdf" / "autodrive_roboracer.urdf"
VEHICLES = REPO_ROOT / "config" / "vehicles"
VEHICLE_DIR = VEHICLES / "autodrive_roboracer"

WHEELS = {
    "left_rear_wheel": "wheel_rl.stl",
    "right_rear_wheel": "wheel_rr.stl",
    "left_front_wheel": "wheel_fl.stl",
    "right_front_wheel": "wheel_fr.stl",
}


def _source(key: str) -> float:
    data = yaml.safe_load((VEHICLE_DIR / "source.yaml").read_text())
    return float(_find(data, key)["value"])


def _find(node, key):
    if isinstance(node, dict):
        if key in node:
            return node[key]
        for value in node.values():
            found = _find(value, key)
            if found is not None:
                return found
    return None


def _rest_height() -> float:
    data = yaml.safe_load((VEHICLE_DIR / "identified.yaml").read_text())
    return float(_find(data, "rest_height")["value"])


def _axle_rest_z() -> float:
    """The rear-axle centre above the ground point `rear_axle`."""
    return _rest_height() - _source("com_z_above_rear_axle_m")


def _root() -> ET.Element:
    assert URDF_PATH.is_file(), f"missing {URDF_PATH}"
    return ET.parse(URDF_PATH).getroot()


def read_stl(path: Path) -> np.ndarray:
    """Vertices (3n, 3) of a binary STL; the size must match its header."""
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    assert len(data) == 84 + 50 * count, f"{path.name} is not a binary STL"
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return (
        np.frombuffer(data, dtype=record, count=count, offset=84)["v"]
        .reshape(-1, 3)
        .astype(float)
    )


def _mesh_of(root: ET.Element, link: str) -> list[Path]:
    element = root.find(f"./link[@name='{link}']")
    return [
        PACKAGE_ROOT
        / m.attrib["filename"].split("racing_vehicle_description/")[-1]
        for m in element.iter("mesh")
    ]


def _in_rear_axle(root: ET.Element, link: str) -> np.ndarray:
    """All of `link`'s mesh vertices, in the rear_axle frame."""
    offset = _origin_in(root, link, "rear_axle")
    return np.vstack([read_stl(p) for p in _mesh_of(root, link)]) + offset


def test_desc_1040_base_link_to_rear_axle_is_minus_lr_minus_h() -> None:
    """DESC-1040: the CoG is lr ahead of the rear axle at height h."""
    joint = _root().find("./joint[@name='base_link_to_rear_axle']")
    assert joint is not None
    xyz = [float(v) for v in joint.find("origin").attrib["xyz"].split()]
    lr = _source("com_x_lr_m")
    h = _rest_height()
    assert xyz == pytest.approx([-lr, 0.0, -h], abs=1e-9)


def test_desc_1050_rear_axle_to_lidar_is_the_vehicle_file_mount() -> None:
    """DESC-1050: laser x/y/yaw from the vehicle file; z is the guide's
    0.096 above the axle centre plus the axle centre's rest height."""
    vehicle = yaml.safe_load(
        (VEHICLES / "autodrive_roboracer.yaml").read_text()
    )
    sim = vehicle["racing_sim"]["ros__parameters"]
    joint = _root().find("./joint[@name='rear_axle_to_lidar']")
    origin = joint.find("origin")
    xyz = [float(v) for v in origin.attrib["xyz"].split()]
    rpy = [float(v) for v in origin.attrib["rpy"].split()]
    want_z = _source("lidar_z_m") + _axle_rest_z()
    assert xyz == pytest.approx(
        [sim["laser_x"], sim["laser_y"], want_z], abs=1e-9
    )
    assert rpy == pytest.approx([0.0, 0.0, sim["laser_yaw"]], abs=1e-9)


def test_desc_1060_mesh_wheels_sit_where_the_car_has_them() -> None:
    """DESC-1060: each tyre centre (by name, so left/right is checked) is at
    (0 | wheelbase, +-track/2, axle rest z) and the radius is the collider's."""
    root = _root()
    wheelbase = _source("wheelbase_m")
    track = _source("track_width_m")
    radius = _source("wheel_radius_m")
    rest = _axle_rest_z()
    for link in WHEELS:
        front = "front" in link
        left = link.startswith("left")
        want = np.array(
            [
                wheelbase if front else 0.0,
                track / 2 if left else -track / 2,
                rest + (0.010 if front else 0.0),
            ]
        )
        points = _in_rear_axle(root, link)
        lo, hi = points.min(axis=0), points.max(axis=0)
        centre = (lo + hi) / 2
        assert centre[:2] == pytest.approx(want[:2], abs=0.006), link
        assert centre[2] == pytest.approx(want[2], abs=0.002), link
        measured = (hi[2] - lo[2]) / 2
        assert measured == pytest.approx(radius, abs=0.0025), link


def _chassis(root: ET.Element) -> np.ndarray:
    return _in_rear_axle(root, "rear_axle")


def test_desc_1070_mesh_body_has_the_cars_length_and_sits_on_the_ground() -> (
    None
):
    """DESC-1070: overhangs 0.09 front / 0.08 rear, length 0.50, lowest
    point on the ground, body centred.

    The rear edge is the rear crash member's. The antennas and the rear
    shock tower stick out further (to -0.104) and are the only things that
    do, so the edge is taken at the 0.05 % quantile of the chassis
    vertices' x (the committed mesh puts it 2.1 mm inside 0.08; the
    1 % quantile gives 3.0 mm), which ignores the few hundred vertices of those two parts.
    """
    root = _root()
    chassis = _chassis(root)
    wheelbase = _source("wheelbase_m")
    length = _source("body_length_m")
    front_overhang = length - wheelbase - 0.08
    front = chassis[:, 0].max() - wheelbase
    rear = -np.quantile(chassis[:, 0], 0.0005)
    assert front == pytest.approx(front_overhang, abs=0.005)
    assert rear == pytest.approx(0.08, abs=0.005)
    assert front + wheelbase + rear == pytest.approx(length, abs=0.005)
    lowest = min(_in_rear_axle(root, link)[:, 2].min() for link in WHEELS)
    assert 0.0 <= lowest <= 0.004, f"lowest point {lowest}"
    lateral = (chassis[:, 1].min() + chassis[:, 1].max()) / 2
    assert lateral == pytest.approx(0.0, abs=0.003)


def test_desc_1080_the_lidar_frame_is_in_the_lidar_mesh() -> None:
    """DESC-1080: the lidar frame origin lies inside the lidar mesh's
    bounding box and near its horizontal centre."""
    root = _root()
    points = _in_rear_axle(root, "lidar")
    origin = _origin_in(root, "lidar", "rear_axle")
    lo, hi = points.min(axis=0), points.max(axis=0)
    assert np.all(lo <= origin) and np.all(origin <= hi), (lo, origin, hi)
    centre = (lo + hi) / 2
    assert centre[:2] == pytest.approx(origin[:2], abs=0.008)
