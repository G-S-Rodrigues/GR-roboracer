import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from f1tenth_gym_jax.envs.utils import Param

URDF = (
    Path(__file__).resolve().parents[2]
    / "racing_vehicle_description"
    / "urdf"
    / "f1tenth.urdf"
)


def test_simjax_1140_urdf_rear_axle_is_behind_and_below_the_gyms_cog():
    """SIMJAX-1140: base_link is the gym's CoG; rear_axle hangs off it.

    The gym's `st` state is the centre of gravity, `lr` ahead of the rear
    axle and `h` up, and the sim publishes it as base_link. The URDF's
    drawing is authored at the rear axle's ground point, so `rear_axle` is a
    fixed child of base_link at (-lr, 0, -h). The URDF is a second copy of
    `lr` and `h` and `Param` is the backend's: this test pins the two equal
    (repo-gotchas #15).
    """
    joints = {
        joint.get("name"): joint for joint in ET.parse(URDF).findall("joint")
    }
    joint = joints.get("base_link_to_rear_axle")
    assert joint is not None, "URDF has no joint base_link_to_rear_axle"
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "base_link"
    assert joint.find("child").get("link") == "rear_axle"
    origin = joint.find("origin")
    xyz = [float(value) for value in origin.get("xyz").split()]
    rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
    param = Param()
    assert xyz == pytest.approx([-param.lr, 0.0, -param.h], abs=1e-9)
    assert rpy == [0.0, 0.0, 0.0]
