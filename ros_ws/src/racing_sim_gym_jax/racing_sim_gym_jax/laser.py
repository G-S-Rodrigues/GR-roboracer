"""ROS-free placement of the laser on the vehicle.

The gym ray-marches its scan from the vehicle reference point (base_link),
but a real car's lidar sits on a mount ahead of it, and /scan is stamped in
frame `laser`. Every estimator places each beam through TF base_link ->
laser, so a scan cast from base_link but labelled `laser` lands the whole
scan one mount offset away from where it was cast. The sim therefore casts
/scan from the laser's pose, which this module computes.
"""

from __future__ import annotations

from dataclasses import dataclass

import jax.numpy as jnp


@dataclass(frozen=True)
class LaserMount:
    """The planar base_link -> laser transform: the URDF joint
    `base_link_to_laser`, as the vehicle file states it (BRINGUP-1050)."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    @property
    def is_origin(self) -> bool:
        return self.x == 0.0 and self.y == 0.0 and self.yaw == 0.0


def laser_poses(poses, mount: LaserMount):
    """Map vehicle poses `(..., 3)` of x, y, yaw to the laser's poses."""
    poses = jnp.asarray(poses)
    x, y, yaw = poses[..., 0], poses[..., 1], poses[..., 2]
    cos_yaw, sin_yaw = jnp.cos(yaw), jnp.sin(yaw)
    return jnp.stack(
        (
            x + cos_yaw * mount.x - sin_yaw * mount.y,
            y + sin_yaw * mount.x + cos_yaw * mount.y,
            yaw + mount.yaw,
        ),
        axis=-1,
    )
