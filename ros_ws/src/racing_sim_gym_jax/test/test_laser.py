import math

import numpy as np
import pytest
from racing_sim_gym_jax.laser import LaserMount, laser_poses


def test_simjax_1080_laser_pose_is_the_vehicle_pose_moved_by_the_mount():
    """SIMJAX-1080: /scan is cast from the laser, not from base_link.

    The mount is base_link -> laser, so it is rotated by the vehicle's yaw
    before it is added: 0.275 m ahead is +y for a car facing +y. Casting from
    base_link while stamping frame `laser` puts every beam 0.275 m ahead of
    where the sim cast it, once TF places it - a bias every estimator reads
    as a localization error.
    """
    mount = LaserMount(x=0.275, y=0.1, yaw=0.3)
    poses = np.array(
        [
            [0.0, 0.0, 0.0],
            [2.0, -1.0, math.pi / 2.0],
            [-3.0, 4.0, -3.0 * math.pi / 4.0],
        ],
        dtype=np.float32,
    )

    shifted = np.asarray(laser_poses(poses, mount))

    for (x, y, yaw), (lx, ly, lyaw) in zip(poses, shifted, strict=True):
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        assert lx == pytest.approx(
            x + cos_yaw * 0.275 - sin_yaw * 0.1, abs=1e-6
        )
        assert ly == pytest.approx(
            y + sin_yaw * 0.275 + cos_yaw * 0.1, abs=1e-6
        )
        assert lyaw == pytest.approx(yaw + 0.3, abs=1e-6)
    assert shifted[1, :2] == pytest.approx([2.0 - 0.1, -1.0 + 0.275], abs=1e-6)


def test_simjax_1080_the_zero_mount_is_the_vehicle_pose():
    """SIMJAX-1080: a mount at base_link casts from base_link, unchanged."""
    poses = np.array([[1.5, -2.5, 0.7]], dtype=np.float32)

    assert np.asarray(laser_poses(poses, LaserMount())) == pytest.approx(poses)
    assert LaserMount().is_origin
    assert not LaserMount(x=0.275).is_origin
