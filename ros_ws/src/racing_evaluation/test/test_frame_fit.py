"""EVAL-1610..1630: fit a map-frame correction from lidar scans (#44 part 10).

Synthetic scans are ray-marched on Porto's own map from poses along the
centerline, then the poses are perturbed by a known rigid transform, which is
what a wrong `world_to_map` does to every published pose. The fit must recover
the correction that undoes it.
"""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from racing_evaluation.frame_fit import (
    Correction,
    apply_correction,
    compose,
    fit_correction,
)
from racing_evaluation.map_accuracy import load_grid

REPO = Path(__file__).resolve().parents[4]
PORTO_MAP = REPO / "config/scenarios/maps/Porto/Porto.yaml"
PORTO_TRACK = REPO / "config/tracks/porto.yaml"
ANGLE_MIN = -2.356
BEAMS = 1081
ANGLE_INC = 2 * 2.356 / (BEAMS - 1)
LASER_X = 0.2733  # autodrive_roboracer.yaml: the lidar sits ahead of the CoG
RANGE_MAX = 10.0
PERTURBATION = Correction(0.12, -0.03, 0.01)


@pytest.fixture(scope="module")
def grid():
    return load_grid(PORTO_MAP)


def _poses(count=24) -> np.ndarray:
    """CoG poses along the centerline, heading along it."""
    rows = yaml.safe_load(PORTO_TRACK.read_text(encoding="utf-8"))["centerline"]
    xy = np.array([r[:2] for r in rows])
    index = np.linspace(0, len(xy), count, endpoint=False).astype(int)
    ahead = xy[(index + 3) % len(xy)] - xy[index]
    yaw = np.arctan2(ahead[:, 1], ahead[:, 0])
    return np.column_stack([xy[index], yaw])


def _cast(grid, poses) -> np.ndarray:
    """Ray-march every beam to the first occupied (or out-of-map) cell."""
    angles = ANGLE_MIN + ANGLE_INC * np.arange(BEAMS)
    steps = np.arange(0.0, RANGE_MAX, 0.02)
    scans = np.full((len(poses), BEAMS), np.inf)
    for k, (x, y, yaw) in enumerate(poses):
        ox = x + LASER_X * math.cos(yaw)
        oy = y + LASER_X * math.sin(yaw)
        direction = yaw + angles
        px = ox + steps[None, :] * np.cos(direction)[:, None]
        py = oy + steps[None, :] * np.sin(direction)[:, None]
        cols = np.floor((px - grid.origin[0]) / grid.resolution).astype(int)
        rows = np.floor((py - grid.origin[1]) / grid.resolution).astype(int)
        inside = (
            (rows >= 0)
            & (rows < grid.occupied.shape[0])
            & (cols >= 0)
            & (cols < grid.occupied.shape[1])
        )
        hit = ~inside
        hit[inside] = grid.occupied[rows[inside], cols[inside]]
        first = hit.argmax(axis=1)
        found = hit.any(axis=1) & (first > 0)
        scans[k, found] = steps[first[found]]
    return scans


def _perturbed(poses, error: Correction) -> np.ndarray:
    return apply_correction(poses, error)


def test_eval_1610_a_known_offset_is_recovered(grid):
    """EVAL-1610: poses displaced by (0.12, -0.03, 0.01 rad) are corrected
    back to within 0.03 m and 0.005 rad, the residual near the scan's own
    cell-size floor."""
    poses = _poses()
    scans = _cast(grid, poses)
    fit = fit_correction(
        _perturbed(poses, PERTURBATION),
        scans,
        grid,
        angle_min=ANGLE_MIN,
        angle_increment=ANGLE_INC,
        laser_x=LASER_X,
    )
    undone = apply_correction(_perturbed(poses, PERTURBATION), fit.correction)
    assert np.abs(undone[:, :2] - poses[:, :2]).max() <= 0.03
    assert abs(fit.correction.dyaw + PERTURBATION.dyaw) <= 0.005
    assert fit.points >= 200
    assert fit.residual < 0.05


def test_eval_1620_unperturbed_poses_fit_the_identity(grid):
    """EVAL-1620: poses that are already right need (almost) no correction:
    within 0.02 m (measured 0.0102, 0.0125 on the synthetic scans, whose
    ray-march steps 0.02 m and whose wall is the 0.05 m cell face) and
    0.005 rad (measured 0.0003)."""
    poses = _poses()
    fit = fit_correction(
        poses,
        _cast(grid, poses),
        grid,
        angle_min=ANGLE_MIN,
        angle_increment=ANGLE_INC,
        laser_x=LASER_X,
    )
    assert abs(fit.correction.dx) <= 0.02
    assert abs(fit.correction.dy) <= 0.02
    assert abs(fit.correction.dyaw) <= 0.005


def test_eval_1630_too_few_points_is_an_error_and_compose_chains(grid):
    """EVAL-1630: fewer than 200 finite returns refuse to fit (a fit of a
    handful of points is a guess); `compose` applies the correction on top of
    a world_to_map pose the way `apply_correction` moves a pose."""
    poses = _poses(count=2)
    blind = np.full((2, BEAMS), np.inf)
    with pytest.raises(ValueError, match="200"):
        fit_correction(
            poses,
            blind,
            grid,
            angle_min=ANGLE_MIN,
            angle_increment=ANGLE_INC,
            laser_x=LASER_X,
        )
    frame = (-2.9, 1.2, math.pi / 2)
    fixed = compose(PERTURBATION, frame)
    moved = apply_correction(np.array([frame]), PERTURBATION)[0]
    assert fixed == pytest.approx(tuple(moved))
