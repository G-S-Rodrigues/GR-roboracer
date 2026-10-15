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


def test_simjax_1150_urdf_cog_link_is_the_gyms_centre_of_gravity():
    """SIMJAX-1150: the URDF's cog_link is where the gym's state lives.

    The gym's `st` state is the centre of gravity, `lr` ahead of the rear
    axle and `h` up. base_link is the rear axle, so cog_link is a fixed
    child at (lr, 0, h). The URDF is the launch's copy of `lr` (it computes
    Pure Pursuit's reference offset from it) and `Param` is the backend's:
    this test pins the two equal (repo-gotchas #15).
    """
    joints = {
        joint.get("name"): joint for joint in ET.parse(URDF).findall("joint")
    }
    joint = joints["base_link_to_cog_link"]
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "base_link"
    assert joint.find("child").get("link") == "cog_link"
    origin = joint.find("origin")
    xyz = [float(value) for value in origin.get("xyz").split()]
    rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
    param = Param()
    assert xyz == pytest.approx([param.lr, 0.0, param.h], abs=1e-9)
    assert rpy == [0.0, 0.0, 0.0]
