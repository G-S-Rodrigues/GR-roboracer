"""Fit the rigid correction between a published map frame and the true one.

ROS-free (numpy only, on `map_accuracy`'s wall-face helpers). Given the poses a
run published (base_link, the CoG, in whatever map frame the stack was told)
and the lidar scans taken at them, find the transform `C` that, applied to
every scan endpoint, makes the endpoints lie on the map image's walls. `C`
composes onto a `world_to_map` (`compose`), which is how part 10 measures
`config/autodrive/porto_frame.yaml` instead of deriving it (#44).

The cost is the mean over endpoints of ``min(distance to the nearest wall
face, CAP)``: the cap keeps a few stray returns (an opponent, a door) from
dragging the fit. The search is a coarse-to-fine pattern search over
(dx, dy, dyaw) on a distance field sampled once at the map's resolution.
`Fit.spread` is for the caller to fill by fitting subsets (the tool fits lap
thirds): along-track translation is weakly observable on a straight.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from racing_evaluation.map_accuracy import (
    Grid,
    _nearest_distance,
    wall_edges,
)

__all__ = ["Correction", "Fit", "apply_correction", "compose", "fit_correction"]

CAP = 0.5  # metres: a return further than this from any wall is not fitted
MIN_POINTS = 200
MAX_POINTS = 30000
SEARCH_LEVELS = 9
FIRST_STEP_XY = 0.2
FIRST_STEP_YAW = 0.04
PAD = 1.0  # the distance field extends this far past the endpoints


@dataclass(frozen=True)
class Correction:
    """p' = R(dyaw) p + (dx, dy): rotation about the map origin, then shift."""

    dx: float
    dy: float
    dyaw: float


@dataclass(frozen=True)
class Fit:
    correction: Correction
    residual: float  # mean min(distance, CAP) after the correction, metres
    points: int  # scan endpoints used


def apply_correction(poses: np.ndarray, c: Correction) -> np.ndarray:
    """Move (n, 3) x, y, yaw poses by `c` (yaw wrapped to [-pi, pi])."""
    poses = np.asarray(poses, dtype=float)
    cos, sin = math.cos(c.dyaw), math.sin(c.dyaw)
    out = np.empty_like(poses)
    out[:, 0] = cos * poses[:, 0] - sin * poses[:, 1] + c.dx
    out[:, 1] = sin * poses[:, 0] + cos * poses[:, 1] + c.dy
    out[:, 2] = np.remainder(poses[:, 2] + c.dyaw + math.pi, math.tau) - math.pi
    return out


def compose(c: Correction, world_to_map: tuple[float, float, float]):
    """The `world_to_map` (x, y, yaw) after applying `c` on top of it."""
    moved = apply_correction(np.array([world_to_map], dtype=float), c)[0]
    return (float(moved[0]), float(moved[1]), float(moved[2]))


def _endpoints(poses, scans, angle_min, angle_increment, laser_x, range_max):
    poses = np.asarray(poses, dtype=float)
    scans = np.asarray(scans, dtype=float)
    if scans.ndim != 2 or scans.shape[0] != len(poses):
        raise ValueError("scans must be (n_poses, n_beams)")
    angles = angle_min + angle_increment * np.arange(scans.shape[1])
    heading = poses[:, 2:3] + angles[None, :]
    ox = poses[:, 0:1] + laser_x * np.cos(poses[:, 2:3])
    oy = poses[:, 1:2] + laser_x * np.sin(poses[:, 2:3])
    valid = np.isfinite(scans) & (scans > 0.05) & (scans < range_max)
    x = ox + scans * np.cos(heading)
    y = oy + scans * np.sin(heading)
    return np.column_stack([x[valid], y[valid]])


def _distance_field(points: np.ndarray, grid: Grid):
    low = points.min(axis=0) - PAD
    high = points.max(axis=0) + PAD
    res = grid.resolution
    xs = np.arange(low[0], high[0] + res, res)
    ys = np.arange(low[1], high[1] + res, res)
    gx, gy = np.meshgrid(xs, ys)
    lattice = np.column_stack([gx.ravel(), gy.ravel()])
    field = np.minimum(_nearest_distance(lattice, wall_edges(grid)), CAP)
    return field.reshape(gy.shape), low, res


def _sample(field, low, res, points: np.ndarray) -> np.ndarray:
    """Bilinear lookup of the distance field; CAP outside it."""
    u = (points[:, 0] - low[0]) / res
    v = (points[:, 1] - low[1]) / res
    i0, j0 = np.floor(v).astype(int), np.floor(u).astype(int)
    inside = (
        (i0 >= 0) & (i0 < field.shape[0] - 1) & (j0 >= 0)
        & (j0 < field.shape[1] - 1)
    )  # fmt: skip
    out = np.full(len(points), CAP)
    i, j = i0[inside], j0[inside]
    fu, fv = (u - j0)[inside], (v - i0)[inside]
    out[inside] = (
        field[i, j] * (1 - fu) * (1 - fv)
        + field[i, j + 1] * fu * (1 - fv)
        + field[i + 1, j] * (1 - fu) * fv
        + field[i + 1, j + 1] * fu * fv
    )
    return out


def fit_correction(
    poses,
    scans,
    grid: Grid,
    *,
    angle_min: float,
    angle_increment: float,
    laser_x: float,
    range_max: float = 30.0,
) -> Fit:
    """The correction that puts the scans' endpoints on `grid`'s walls.

    Raises ValueError for fewer than 200 finite endpoints."""
    points = _endpoints(
        poses, scans, angle_min, angle_increment, laser_x, range_max
    )
    if len(points) < MIN_POINTS:
        raise ValueError(
            f"{len(points)} finite scan endpoints; the fit needs at least "
            f"{MIN_POINTS}"
        )
    if len(points) > MAX_POINTS:
        points = points[:: math.ceil(len(points) / MAX_POINTS)]
    field, low, res = _distance_field(points, grid)

    def cost(dx: float, dy: float, dyaw: float) -> float:
        cos, sin = math.cos(dyaw), math.sin(dyaw)
        moved = np.column_stack(
            [
                cos * points[:, 0] - sin * points[:, 1] + dx,
                sin * points[:, 0] + cos * points[:, 1] + dy,
            ]
        )
        return float(_sample(field, low, res, moved).mean())

    best = (0.0, 0.0, 0.0)
    best_cost = cost(*best)
    step_xy, step_yaw = FIRST_STEP_XY, FIRST_STEP_YAW
    for _ in range(SEARCH_LEVELS):
        centre = best
        for ix in range(-3, 4):
            for iy in range(-3, 4):
                for ia in range(-3, 4):
                    trial = (
                        centre[0] + ix * step_xy,
                        centre[1] + iy * step_xy,
                        centre[2] + ia * step_yaw,
                    )
                    value = cost(*trial)
                    if value < best_cost:
                        best, best_cost = trial, value
        step_xy *= 0.5
        step_yaw *= 0.5
    return Fit(Correction(*best), best_cost, len(points))
