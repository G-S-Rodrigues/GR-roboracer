"""The pure parts of the live mapping run SIM-3740 (#44 part 11a).

No rclpy here: the settle wait, the no-ground-truth loop closure and the
dev-ground-truth shape score take plain arrays, so
`scripts/tests/test_autodrive_mapping_helpers.py` covers them without a sim.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from racing_evaluation.frame_fit import (
    Correction,
    _endpoints,
    apply_correction,
    fit_correction,
)
from racing_evaluation.map_accuracy import (
    Grid,
    MapAccuracy,
    _nearest_distance,
    map_to_wall,
    wall_edges,
)


def quiet_for(times, now, window_s, max_gap_s) -> bool:
    """True when `times` (arrival of /scan, one clock) hold at least two
    scans in the last `window_s` before `now` and no gap above `max_gap_s`,
    the gap from the last scan to `now` included."""
    recent = [t for t in times if now - window_s <= t <= now]
    if len(recent) < 2:
        return False
    if recent[0] - (now - window_s) > max_gap_s or now - recent[-1] > max_gap_s:
        return False
    return all(
        b - a <= max_gap_s for a, b in zip(recent, recent[1:], strict=False)
    )


def _travel(poses) -> np.ndarray:
    steps = np.hypot(*np.diff(np.asarray(poses)[:, :2], axis=0).T)
    return np.concatenate([[0.0], np.cumsum(steps)])


def _grid_of(points: np.ndarray, resolution: float, pad: float = 1.0) -> Grid:
    low = points.min(axis=0) - pad
    high = points.max(axis=0) + pad
    shape = np.ceil((high - low) / resolution).astype(int)[::-1]
    occupied = np.zeros(shape, dtype=bool)
    cols = ((points[:, 0] - low[0]) / resolution).astype(int)
    rows = ((points[:, 1] - low[1]) / resolution).astype(int)
    occupied[rows, cols] = True
    return Grid(occupied, ~occupied, resolution, (float(low[0]), float(low[1])))


def loop_closure(
    poses,
    scans,
    angle_min,
    angle_increment,
    laser_x,
    range_max,
    *,
    travel_m=3.0,
    resolution=0.05,
) -> Correction:
    """The correction that puts the last `travel_m` of scans (at their odom
    poses) onto a grid of the first `travel_m`: the lap's dead-reckoning
    closure error, measured with no ground truth. It is zero when the
    odometry returns to where it started."""
    poses = np.asarray(poses, dtype=float)
    scans = np.asarray(scans, dtype=float)
    travel = _travel(poses)
    first = travel <= travel_m
    last = travel >= travel[-1] - travel_m
    points = _endpoints(
        poses[first],
        scans[first],
        angle_min,
        angle_increment,
        laser_x,
        range_max,
    )
    reference = _grid_of(points, resolution)
    return fit_correction(
        poses[last],
        scans[last],
        reference,
        angle_min=angle_min,
        angle_increment=angle_increment,
        laser_x=laser_x,
        range_max=range_max,
    ).correction


@dataclass(frozen=True)
class ShapeFit:
    """`raw`: the map against truth as it is. `median`/`p95`: after the best
    rigid fit of the odom-pose scans onto truth is applied to the map's
    occupied cells. `correction`: that fit; `residual`: its mean distance."""

    raw: MapAccuracy
    median: float
    p95: float
    correction: Correction
    residual: float


def shape_after_fit(
    produced: Grid,
    truth: Grid,
    poses,
    scans,
    angle_min,
    angle_increment,
    laser_x,
    range_max,
) -> ShapeFit:
    """Ruling 2026-10-31: the odom anchor puts the map frame 0.28 m off the
    official one even with a gentle launch, so the map's shape is scored
    after a rigid fit, with the raw offset reported beside it."""
    fit = fit_correction(
        poses,
        scans,
        truth,
        angle_min=angle_min,
        angle_increment=angle_increment,
        laser_x=laser_x,
        range_max=range_max,
    )
    centres = produced.cell_centres(produced.occupied)
    placed = apply_correction(
        np.column_stack([centres, np.zeros(len(centres))]), fit.correction
    )[:, :2]
    distance = _nearest_distance(placed, wall_edges(truth))
    return ShapeFit(
        raw=map_to_wall(produced, truth),
        median=float(np.median(distance)),
        p95=float(np.percentile(distance, 95)),
        correction=fit.correction,
        residual=fit.residual,
    )
