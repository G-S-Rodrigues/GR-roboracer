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

import jax
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


@dataclass(frozen=True, eq=False)
class DistanceGrid:
    """The gym env's map, as jax_pf ray-marches it: a Euclidean distance
    transform (metres) with the map origin's pose and the raster's shape."""

    distance: jax.Array
    origin_x: float
    origin_y: float
    origin_cos: float
    origin_sin: float
    height: int
    width: int
    resolution: float
    eps: float
    max_range: float

    @classmethod
    def of_env(cls, env) -> DistanceGrid:
        return cls(
            jnp.asarray(env.distance_transform),
            float(env.orig_x),
            float(env.orig_y),
            float(env.orig_c),
            float(env.orig_s),
            int(env.height),
            int(env.width),
            float(env.resolution),
            float(env.eps),
            float(env.max_range),
        )

    def nearest_obstacle(self, x, y):
        """jax_pf's `distance_transform` lookup (truncated cell, clamped)."""
        dx, dy = x - self.origin_x, y - self.origin_y
        row = jnp.clip(
            (-dx * self.origin_sin + dy * self.origin_cos) / self.resolution,
            0,
            self.height - 1,
        ).astype(int)
        col = jnp.clip(
            (dx * self.origin_cos + dy * self.origin_sin) / self.resolution,
            0,
            self.width - 1,
        ).astype(int)
        return self.distance[row, col]


def declared_beam_angles(field_of_view: float, beam_count: int):
    """The beam angles a LaserScan with angle_min = -fov/2 and
    angle_increment = fov/(N-1) declares, relative to the laser."""
    return -field_of_view / 2.0 + jnp.arange(beam_count) * (
        field_of_view / (beam_count - 1)
    )


def declared_angle_increment(field_of_view: float, beam_count: int) -> float:
    """/scan's angle_increment: fov/(N-1), the cast's own spacing."""
    return field_of_view / (beam_count - 1) if beam_count > 1 else 0.0


def gym_angle_increment(field_of_view: float, beam_count: int) -> float:
    """/ground_truth/scan's angle_increment: the spacing the gym casts at.

    f1tenth_gym_jax passes jax_pf an increment of fov/(N-1), and jax_pf's
    `get_scan` spreads N beams over `linspace(start, start + inc * N, N)`,
    i.e. inc * N/(N-1) apart (plan D6, repo-gotchas #22).
    """
    return declared_angle_increment(field_of_view, beam_count) * (
        beam_count / (beam_count - 1) if beam_count > 1 else 0.0
    )


def cast_scan(grid: DistanceGrid, pose, angles):
    """Ray-march one scan from `pose` (x, y, yaw) at exactly `angles`.

    jax_pf's own march (sphere tracing on the distance transform, the same
    eps and max range), but at the declared angles: jax_pf's `get_scan`
    spaces its beams fov/(N-1) * N/(N-1) apart and reads each from a
    `theta_dis` table at a truncated index (plan D6, repo-gotchas #22), so
    its beams are not where a LaserScan declaring fov/(N-1) says they are.
    """
    x, y, yaw = pose[0], pose[1], pose[2]

    def march(angle):
        cos_a, sin_a = jnp.cos(yaw + angle), jnp.sin(yaw + angle)
        first = grid.nearest_obstacle(x, y)

        def advance(carry):
            step, total, px, py = carry
            px, py = px + step * cos_a, py + step * sin_a
            step = grid.nearest_obstacle(px, py)
            return step, total + step, px, py

        def marching(carry):
            step, total, _, _ = carry
            return (step > grid.eps) & (total <= grid.max_range)

        _, total, _, _ = jax.lax.while_loop(
            marching, advance, (first, first, x, y)
        )
        return jnp.minimum(total, grid.max_range)

    return jax.vmap(march)(angles)
