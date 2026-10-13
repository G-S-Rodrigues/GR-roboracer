"""Map -> track extractor (part 2 of #12).

Ported from ForzaETH/race_stack @202450f, planner/global_planner/global_planner/
{global_planner_utils,global_planner_logic}.py, MIT; see raceline/NOTICE.md.
Not imported from upstream: upstream needs ROS, ``f110_msgs`` and TUM's LGPL
``interp_track``. What differs is listed in NOTICE.md.

Conventions: map frame, metres. ``Pose2D.yaw`` is 0 along +x (the gym / ROS
convention), NOT TUM's psi (0 = north). Grid cell ``(i, j)`` has its centre at
``origin + ((j + 0.5) * res, (i + 0.5) * res)`` (ROS map convention).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import cv2
import numpy as np
import yaml
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
from scipy.signal import savgol_filter
from scipy.spatial import cKDTree
from skimage.measure import find_contours
from skimage.morphology import skeletonize
from skimage.segmentation import watershed

from raceline.track import Track

STEP_M = 0.1  # output spacing; upstream's interp_track step
CURVATURE_SIGMA_M = 0.5
MAX_START_OFFSET = 1.0  # m; a loop must pass this close to start_pose
MIN_SIDE_AGREEMENT = 0.95  # share of points that must agree on a bound's side
BOUND_SIGMA_M = 0.2  # smoothing of a bound before its normals are taken
RECENTRE_SIGMA_M = 0.1  # smoothing of the re-centred line
CORNER_KAPPA = 0.1  # 1/m; the outer bound takes over at this curvature


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float  # rad, map frame, 0 = +x (gym/ROS convention; NOT TUM psi)


@dataclass(frozen=True)
class OccupancyGrid:
    data: np.ndarray  # (H, W) int8, -1/0/100, row 0 = bottom (ROS)
    resolution: float
    origin: Pose2D


def load_map(yaml_path: Path) -> OccupancyGrid:
    """Decode a map_server YAML + PNG into a ROS-convention grid.

    Trinary mode only: ``p = (255 - px) / 255`` (``px / 255`` when
    ``negate: 1``); ``p > occupied_thresh`` is 100, ``p < free_thresh`` is 0,
    anything between is -1. The PNG is flipped so that row 0 is the bottom.
    """
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    if meta.get("mode", "trinary") != "trinary":
        raise ValueError(f"only trinary maps are supported: {meta['mode']}")
    img_path = yaml_path.parent / meta["image"]
    img = cv2.imread(str(img_path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(img_path)
    if img.ndim != 2 or img.dtype != np.uint8:
        raise ValueError(f"{img_path} must be an 8-bit grayscale PNG")
    px = img.astype(float) / 255.0
    p = px if int(meta.get("negate", 0)) else 1.0 - px
    data = np.full(img.shape, -1, dtype=np.int8)
    data[p > float(meta["occupied_thresh"])] = 100
    data[p < float(meta["free_thresh"])] = 0
    ox, oy, oyaw = (float(v) for v in meta["origin"])
    return OccupancyGrid(
        data=np.ascontiguousarray(data[::-1]),
        resolution=float(meta["resolution"]),
        origin=Pose2D(ox, oy, oyaw),
    )


MIN_WALL_M2 = 0.5  # stray wall components smaller than this are filled
MAX_PATCH_M2 = 10.0  # unknown/occupied patches inside the corridor, filled


def _fill_small_walls(bw: np.ndarray, area_m2: float, res: float) -> None:
    n, labels, stats, _ = cv2.connectedComponentsWithStats(
        (bw == 0).astype(np.uint8), connectivity=4
    )
    small = np.flatnonzero(stats[1:, cv2.CC_STAT_AREA] < area_m2 / res**2) + 1
    bw[np.isin(labels, small)] = 255


def _fill_patches(bw: np.ndarray, area_m2: float, res: float) -> None:
    # A survey leaves patches of unknown inside the corridor, joined to the
    # outside unknown by thin speckle paths. Opening the wall mask with 5x5
    # cuts those paths; the patches that come out small are corridor, not wall.
    wall = cv2.morphologyEx(
        (bw == 0).astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)
    )
    n, labels, stats, _ = cv2.connectedComponentsWithStats(wall, connectivity=4)
    small = np.flatnonzero(stats[1:, cv2.CC_STAT_AREA] < area_m2 / res**2) + 1
    bw[np.isin(labels, small)] = 255


def _binarize(grid: OccupancyGrid) -> np.ndarray:
    # filter_map_occupancy_grid (:508-530): unknown counts as occupied,
    # then a 3x3 morphological opening, 2 iterations. Not upstream: the
    # clean-up before it. The opening would otherwise sever the thin free
    # strips around stray walls and survey patches and open the loop.
    bw = np.where(grid.data == 0, 255, 0).astype(np.uint8)
    _fill_small_walls(bw, MIN_WALL_M2, grid.resolution)
    _fill_patches(bw, MAX_PATCH_M2, grid.resolution)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel, iterations=2)
    _fill_small_walls(bw, MIN_WALL_M2, grid.resolution)
    return bw


def _prune_spurs(skel: np.ndarray) -> np.ndarray:
    # Not upstream: remove skeleton branches that end in a dead end by
    # peeling end points until none remain. A closed loop has no end points,
    # so it is untouched; every spur vanishes.
    skel = (skel > 0).astype(np.uint8)
    kernel = np.ones((3, 3), np.float32)
    while True:
        nbrs = cv2.filter2D(
            skel, cv2.CV_8U, kernel, borderType=cv2.BORDER_CONSTANT
        )
        ends = (skel == 1) & (nbrs <= 2)
        if not ends.any():
            return skel * 255
        skel[ends] = 0


def _cells_to_m(cells: np.ndarray, grid: OccupancyGrid) -> np.ndarray:
    # Cell-centre convention (upstream uses the cell corner).
    cells = np.asarray(cells, dtype=float)
    return np.column_stack(
        [
            grid.origin.x + (cells[:, 0] + 0.5) * grid.resolution,
            grid.origin.y + (cells[:, 1] + 0.5) * grid.resolution,
        ]
    )


def _closed_contours(binary: np.ndarray) -> list[np.ndarray]:
    # extract_centerline (global_planner_utils.py:25-105): RETR_CCOMP, keep the
    # closed contours (those with a parent or a child).
    contours, hierarchy = cv2.findContours(
        binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
    )
    if hierarchy is None:
        return []
    out = []
    for cont, h in zip(contours, hierarchy[0], strict=True):
        if h[2] >= 0 or h[3] >= 0:
            out.append(cont.reshape(-1, 2))
    return out


def _contour_length(cells: np.ndarray) -> float:
    d = cells - np.roll(cells, 1, axis=0)
    return float(np.hypot(d[:, 0], d[:, 1]).sum())


def _smooth_closed(pts: np.ndarray) -> np.ndarray:
    # smooth_centerline (:107-153): Savitzky-Golay order 3, window from the
    # point count, second pass rotated by half a lap to fix the wrap.
    n = len(pts)
    if n > 2000:
        win = int(n / 200) * 10 + 1
    elif n > 1000:
        win = 81
    elif n > 500:
        win = 41
    else:
        win = 21
    if win >= n // 2:
        raise ValueError(f"centerline of {n} points is too short to smooth")
    smooth = savgol_filter(pts, win, 3, axis=0)
    half = n // 2
    rolled = np.append(pts[half:], pts[:half], axis=0)
    smooth2 = savgol_filter(rolled, win, 3, axis=0)
    smooth[:win] = smooth2[half : half + win]
    smooth[-win:] = smooth2[half - win : half]
    return smooth


def _resample_closed(pts: np.ndarray, step: float = STEP_M) -> np.ndarray:
    closed = np.vstack([pts, pts[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(round(s[-1] / step)), 3)
    t = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.column_stack(
        [np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])]
    )


def _signed_area(xy: np.ndarray) -> float:
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def _tangents(xy: np.ndarray) -> np.ndarray:
    t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    return t / np.hypot(t[:, 0], t[:, 1])[:, None]


def _curvature(xy: np.ndarray) -> np.ndarray:
    # Not upstream's: computed here from the extracted path, never copied
    # (spec). The skeleton wobbles by about a cell, which a second derivative
    # turns into curvature of the wrong sign, so the path is smoothed by a
    # Gaussian of CURVATURE_SIGMA_M first, then differentiated as a periodic
    # cubic spline in arclength.
    sigma = CURVATURE_SIGMA_M / STEP_M
    xy = gaussian_filter1d(xy, sigma, axis=0, mode="wrap")
    closed = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    cx = CubicSpline(s, closed[:, 0], bc_type="periodic")
    cy = CubicSpline(s, closed[:, 1], bc_type="periodic")
    sn = s[:-1]
    dx, dy = cx(sn, 1), cy(sn, 1)
    ddx, ddy = cx(sn, 2), cy(sn, 2)
    return (dx * ddy - dy * ddx) / (dx * dx + dy * dy) ** 1.5


def _bound_contours(
    grid: OccupancyGrid, binary: np.ndarray, centerline_xy: np.ndarray
) -> list[np.ndarray]:
    # extract_track_bounds (:156-293): watershed seeded by the centerline.
    cells = (centerline_xy - [grid.origin.x, grid.origin.y]) / (
        grid.resolution
    ) - 0.5
    seed = np.zeros(binary.shape, dtype=np.uint8)
    cv2.polylines(
        seed, [np.round(cells).astype(np.int32).reshape(-1, 1, 2)], True, 255, 2
    )
    _, markers = cv2.connectedComponents(seed)
    dist = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
    labels = watershed(-dist, markers, mask=binary > 0)
    closed: list[np.ndarray] = []
    for label in np.unique(labels):
        if label == 0:
            continue
        mask = np.pad((labels == label).astype(float), 1)
        for c in find_contours(mask, 0.5):
            if len(c) > 3 and np.allclose(c[0], c[-1]):
                # (row, col) -> (col, row), undo the pad
                closed.append(c[:-1, ::-1] - 1.0)
    if len(closed) != 2:
        raise ValueError(
            f"expected exactly 2 track bounds, found {len(closed)}; "
            "check the map"
        )
    return [_cells_to_m(c, grid) for c in closed]


def _classify_bounds(
    xy: np.ndarray, bounds: list[np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    # Replaces upstream's traversal-direction heuristic (:265-280): the sign of
    # cross(tangent, offset to the bound) at every centreline point, majority
    # vote; positive = left of travel (track.cpp:143's left normal).
    tan = _tangents(xy)
    votes = []
    for b in bounds:
        _, idx = cKDTree(_resample_closed(b)).query(xy)
        near = _resample_closed(b)[idx]
        d = near - xy
        cross = tan[:, 0] * d[:, 1] - tan[:, 1] * d[:, 0]
        pos = float(np.mean(cross > 0))
        votes.append(pos if pos >= 0.5 else -(1.0 - pos))
    for v in votes:
        if abs(v) < MIN_SIDE_AGREEMENT:
            raise ValueError(
                f"a track bound is on one side of only {abs(v):.0%} of the "
                "centerline; need >= 95%"
            )
    if (votes[0] > 0) == (votes[1] > 0):
        raise ValueError("both track bounds are on the same side")
    left, right = (
        (bounds[0], bounds[1]) if votes[0] > 0 else (bounds[1], bounds[0])
    )
    return right, left


def _widths(xy: np.ndarray, bound: np.ndarray) -> np.ndarray:
    # dist_to_bounds (:296-405): minimum distance to the bound at 0.1 m.
    d, _ = cKDTree(_resample_closed(bound)).query(xy)
    return d


def _offset_bound(
    bound: np.ndarray, xy: np.ndarray, half_width: float
) -> np.ndarray:
    # The bound moved half_width towards the centreline along its own normal.
    b = gaussian_filter1d(
        _resample_closed(bound), BOUND_SIGMA_M / STEP_M, axis=0, mode="wrap"
    )
    t = _tangents(b)
    n = np.column_stack([-t[:, 1], t[:, 0]])
    _, i = cKDTree(xy).query(b)
    side = np.sign(np.einsum("ij,ij->i", xy[i] - b, n))
    side[side == 0] = 1.0
    side = np.sign(gaussian_filter1d(side, 20, mode="wrap"))
    side[side == 0] = 1.0
    return b + half_width * side[:, None] * n


def _recentre(
    xy: np.ndarray, right: np.ndarray, left: np.ndarray
) -> np.ndarray:
    # Not upstream. A medial axis cuts a corner tighter than the corridor is
    # wide: the inner wall is a short arc (or a clipped cusp) and the axis
    # runs along it. The outer wall is intact there, so at a corner the line
    # is the OUTER bound moved inward by the half-width; on straights it is
    # the mean of both bounds' offsets, blended by the curvature's sign.
    # Assumes a constant half-width (the median of the medial axis's
    # distances to the bounds); a corridor that varies at a corner is only as
    # good as that.
    half = float(
        np.median(
            (cKDTree(left).query(xy)[0] + cKDTree(right).query(xy)[0]) / 2
        )
    )
    off_left = _offset_bound(left, xy, half)
    off_right = _offset_bound(right, xy, half)
    # curvature > 0 is a left turn: the left bound is the inner one
    w_right = np.clip(0.5 + 0.5 * _curvature(xy) / CORNER_KAPPA, 0.0, 1.0)
    w_right = gaussian_filter1d(w_right, 10, mode="wrap")
    near_l = off_left[cKDTree(off_left).query(xy)[1]]
    near_r = off_right[cKDTree(off_right).query(xy)[1]]
    mixed = (1.0 - w_right)[:, None] * near_l + w_right[:, None] * near_r
    mixed = gaussian_filter1d(
        mixed, RECENTRE_SIGMA_M / STEP_M, axis=0, mode="wrap"
    )
    return _resample_closed(mixed)


def _check_inside(
    xy: np.ndarray,
    binary: np.ndarray,
    grid: OccupancyGrid,
    right: np.ndarray,
    left: np.ndarray,
) -> None:
    # Not upstream. _recentre assumes one half-width for the whole lap; where
    # the corridor is narrower at a corner the line crosses the inner wall,
    # and the (unsigned) widths would still look fine. Fail closed: every
    # point must be on free cells and on the right side of both bounds.
    col = np.floor((xy[:, 0] - grid.origin.x) / grid.resolution).astype(int)
    row = np.floor((xy[:, 1] - grid.origin.y) / grid.resolution).astype(int)
    h, w = binary.shape
    on_map = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    free = np.zeros(len(xy), dtype=bool)
    free[on_map] = binary[row[on_map], col[on_map]] > 0
    tan = _tangents(xy)
    bad_side = np.zeros(len(xy), dtype=bool)
    for bound, sign in ((left, 1.0), (right, -1.0)):
        b = _resample_closed(bound)
        d = b[cKDTree(b).query(xy)[1]] - xy
        cross = tan[:, 0] * d[:, 1] - tan[:, 1] * d[:, 0]
        bad_side |= sign * cross <= 0
    bad = ~free | bad_side
    if bad.any():
        raise ValueError(
            f"the centerline leaves the corridor at {int(bad.sum())} of "
            f"{len(xy)} points (first near {xy[bad][0].round(2).tolist()}): "
            "the corridor narrows where the re-centring assumes a constant "
            "half-width"
        )


def _check_args(grid, resolution, origin):
    if not np.isclose(resolution, grid.resolution):
        raise ValueError(
            f"resolution {resolution} disagrees with grid {grid.resolution}"
        )
    if not (
        np.isclose(origin.x, grid.origin.x)
        and np.isclose(origin.y, grid.origin.y)
        and np.isclose(origin.yaw, grid.origin.yaw)
    ):
        raise ValueError(f"origin {origin} disagrees with grid {grid.origin}")
    if grid.origin.yaw != 0.0:
        raise ValueError("a map origin with non-zero yaw is not supported")


def _centerline(grid, start_pose, direction):
    binary = _binarize(grid)
    skel = skeletonize(binary > 0, method="lee")
    skel = _prune_spurs(skel)
    loops = []
    for cells in _closed_contours(skel):
        m = _cells_to_m(cells, grid)
        gap = np.hypot(m[:, 0] - start_pose.x, m[:, 1] - start_pose.y).min()
        if gap <= MAX_START_OFFSET:
            loops.append(
                (abs(cv2.contourArea(cells.astype(np.float32))), cells)
            )
    if not loops:
        raise ValueError("no closed track loop near start_pose")
    cells = max(loops, key=lambda t: t[0])[1]
    xy = _resample_closed(
        _cells_to_m(_smooth_closed(cells.astype(float)), grid)
    )
    return binary, xy


def track_bounds(
    grid: OccupancyGrid,
    centerline_xy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(right, left)`` bound polylines in metres, for an oriented
    centreline (left = left of travel)."""
    binary = _binarize(grid)
    return _classify_bounds(
        centerline_xy, _bound_contours(grid, binary, centerline_xy)
    )


def extract_track(
    grid: OccupancyGrid,
    resolution: float,
    origin: Pose2D,
    start_pose: Pose2D,
    direction: Literal["ccw", "cw"],
    name: str = "extracted",
) -> Track:
    _check_args(grid, resolution, origin)
    if direction not in ("ccw", "cw"):
        raise ValueError(f"direction must be 'ccw' or 'cw': {direction}")
    binary, xy = _centerline(grid, start_pose, direction)
    # compare_direction (:683-704): flip when the start heading disagrees.
    near = int(
        np.argmin(np.hypot(xy[:, 0] - start_pose.x, xy[:, 1] - start_pose.y))
    )
    heading = np.array([np.cos(start_pose.yaw), np.sin(start_pose.yaw)])
    if _tangents(xy)[near] @ heading < 0:
        xy = xy[::-1].copy()
        near = int(
            np.argmin(
                np.hypot(xy[:, 0] - start_pose.x, xy[:, 1] - start_pose.y)
            )
        )
    xy = np.roll(xy, -near, axis=0)
    ccw = _signed_area(xy) > 0
    if ccw != (direction == "ccw"):
        raise ValueError(
            f"direction '{direction}' contradicts start_pose.yaw "
            f"({start_pose.yaw:.3f} rad), which runs the loop "
            f"{'ccw' if ccw else 'cw'}"
        )
    right, left = _classify_bounds(xy, _bound_contours(grid, binary, xy))
    xy = _recentre(xy, right, left)
    _check_inside(xy, binary, grid, right, left)
    return Track(
        name=name,
        xy=xy,
        curvature=_curvature(xy),
        w_left=_widths(xy, left),
        w_right=_widths(xy, right),
    )
