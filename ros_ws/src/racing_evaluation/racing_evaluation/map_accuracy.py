"""How far a produced occupancy grid's walls sit from the true walls.

ROS-free (numpy and PIL only). The truth is a map image's free/occupied
boundary: the faces between a free and an occupied truth cell, which is where
the simulated lidar's beams stop. For each occupied cell of the produced grid
the offset is the signed distance from the cell's CENTRE to the nearest truth
face, positive when the centre lies on the truth's occupied side (the wall
side) and negative inside the corridor.

The raw centre distance is kept on purpose (owner ruling): a perfect map
whose wall cells lie flush against the true wall scores ``resolution / 2``
(2.5 cm at 5 cm), never 0. Walls thicker than one cell read as larger
positive offsets, which is correct: those cells are not on the boundary.
Read ``signed_mean`` against that floor.

The metric does no alignment. A method whose map frame is not the world frame
must transform its grid first. Percentiles use numpy's linear interpolation.
"""

import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

# map_saver's defaults (occupied_thresh 0.65, free_thresh 0.196) as
# OccupancyGrid percentages: occupied at or above 65, free at or below 19,
# else unknown.
OCCUPIED_MIN = 65
FREE_MAX = 19


@dataclass(frozen=True)
class Grid:
    """``occupied``/``free``: bool[rows, cols], row 0 = the BOTTOM row."""

    occupied: np.ndarray
    free: np.ndarray
    resolution: float
    origin: tuple[float, float]

    def cell_centres(self, mask: np.ndarray) -> np.ndarray:
        """(n, 2) world centres of the cells set in ``mask``."""
        rows, cols = np.nonzero(mask)
        return np.stack(
            [
                self.origin[0] + (cols + 0.5) * self.resolution,
                self.origin[1] + (rows + 0.5) * self.resolution,
            ],
            axis=1,
        )


@dataclass(frozen=True)
class MapAccuracy:
    """Offsets of the occupied cells; every metric is None for zero cells."""

    wall_cell_count: int
    maximum: float | None  # of |offset|
    p95: float | None  # of |offset|
    median: float | None  # of |offset|
    signed_median: float | None  # + wall side, - corridor side
    signed_mean: float | None


def load_grid(yaml_path: Path) -> Grid:
    """Read a map_server yaml + image: row 0 of the image is the TOP."""
    yaml_path = Path(yaml_path)
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image = np.asarray(
        Image.open(yaml_path.parent / document["image"]).convert("L"),
        dtype=float,
    )
    if document.get("negate", 0):
        probability = image / 255.0
    else:
        probability = (255.0 - image) / 255.0
    occupied = probability > float(document.get("occupied_thresh", 0.65))
    free = probability < float(document.get("free_thresh", 0.196))
    origin = document["origin"]
    return Grid(
        occupied[::-1].copy(),
        free[::-1].copy(),
        float(document["resolution"]),
        (float(origin[0]), float(origin[1])),
    )


def grid_from_occupancy(
    data,
    width: int,
    height: int,
    resolution: float,
    origin,
    occupied_min: int = OCCUPIED_MIN,
    free_max: int = FREE_MAX,
) -> Grid:
    """An OccupancyGrid's row-major data (row 0 = bottom, -1 unknown)."""
    values = np.asarray(data, dtype=int).reshape(height, width)
    return Grid(
        values >= occupied_min,
        (values >= 0) & (values <= free_max),
        float(resolution),
        (float(origin[0]), float(origin[1])),
    )


def wall_edges(truth: Grid) -> np.ndarray:
    """(M, 2, 2) segments: the faces between a free and an occupied cell."""
    occupied, free = truth.occupied, truth.free
    rows, cols = occupied.shape
    res = truth.resolution
    ox, oy = truth.origin
    segments = []
    # Vertical faces between column j and j+1 (a face at x = ox + (j+1)*res).
    pair = (occupied[:, :-1] & free[:, 1:]) | (free[:, :-1] & occupied[:, 1:])
    for i, j in zip(*np.nonzero(pair), strict=True):
        x = ox + (j + 1) * res
        segments.append([[x, oy + i * res], [x, oy + (i + 1) * res]])
    # Horizontal faces between row i and i+1.
    pair = (occupied[:-1] & free[1:]) | (free[:-1] & occupied[1:])
    for i, j in zip(*np.nonzero(pair), strict=True):
        y = oy + (i + 1) * res
        segments.append([[ox + j * res, y], [ox + (j + 1) * res, y]])
    return np.asarray(segments, dtype=float).reshape(-1, 2, 2)


def _point_segment_distance(
    points: np.ndarray, edges: np.ndarray
) -> np.ndarray:
    """min over segments of the distance, (n,) for (n, 2) points."""
    start = edges[None, :, 0, :]
    delta = edges[None, :, 1, :] - start
    to_point = points[:, None, :] - start
    t = np.clip(
        (to_point * delta).sum(-1) / np.maximum((delta**2).sum(-1), 1e-300),
        0.0,
        1.0,
    )
    closest = start + t[..., None] * delta
    return np.sqrt(((points[:, None, :] - closest) ** 2).sum(-1)).min(axis=1)


def _nearest_distance(points: np.ndarray, edges: np.ndarray) -> np.ndarray:
    """Distance to the nearest edge, bucketed so Spielberg stays fast."""
    bucket = 0.5
    mids = edges.mean(axis=1)
    low = np.minimum(mids.min(axis=0), points.min(axis=0))
    mid_cells = np.floor((mids - low) / bucket).astype(int)
    point_cells = np.floor((points - low) / bucket).astype(int)
    by_cell: dict[tuple[int, int], list[int]] = {}
    for index, cell in enumerate(map(tuple, mid_cells)):
        by_cell.setdefault(cell, []).append(index)
    extent = int(max(mid_cells.max(), point_cells.max())) + 2
    distance = np.empty(len(points))
    groups: dict[tuple[int, int], list[int]] = {}
    for index, cell in enumerate(map(tuple, point_cells)):
        groups.setdefault(cell, []).append(index)
    for (cx, cy), members in groups.items():
        member = np.asarray(members)
        reach = 1
        while True:
            candidates: list[int] = []
            for gx in range(cx - reach, cx + reach + 1):
                for gy in range(cy - reach, cy + reach + 1):
                    candidates.extend(by_cell.get((gx, gy), ()))
            if candidates:
                found = _point_segment_distance(
                    points[member], edges[np.asarray(candidates)]
                )
                # Everything within (reach - 1) buckets of the point was
                # searched (a segment's midpoint is within one segment
                # length, below one bucket, of any of its points).
                if found.max() <= (reach - 1) * bucket:
                    distance[member] = found
                    break
            if reach > extent:
                distance[member] = _point_segment_distance(
                    points[member], edges
                )
                break
            reach += 1
    return distance


def _on_wall_side(points: np.ndarray, truth: Grid) -> np.ndarray:
    """True where the point lies in a truth cell that is not free."""
    cols = np.floor((points[:, 0] - truth.origin[0]) / truth.resolution)
    rows = np.floor((points[:, 1] - truth.origin[1]) / truth.resolution)
    rows, cols = rows.astype(int), cols.astype(int)
    inside = (
        (rows >= 0)
        & (rows < truth.free.shape[0])
        & (cols >= 0)
        & (cols < truth.free.shape[1])
    )
    free = np.zeros(len(points), dtype=bool)
    free[inside] = truth.free[rows[inside], cols[inside]]
    return ~free


def map_to_wall(produced: Grid, truth: Grid) -> MapAccuracy:
    """Score ``produced``'s occupied cells against ``truth``'s wall faces."""
    centres = produced.cell_centres(produced.occupied)
    if len(centres) == 0:
        return MapAccuracy(0, None, None, None, None, None)
    edges = wall_edges(truth)
    if len(edges) == 0:
        raise ValueError("truth grid has no free/occupied boundary")
    distance = _nearest_distance(centres, edges)
    signed = np.where(_on_wall_side(centres, truth), distance, -distance)
    return MapAccuracy(
        wall_cell_count=int(len(centres)),
        maximum=float(distance.max()),
        p95=float(np.percentile(distance, 95)),
        median=float(np.median(distance)),
        signed_median=float(np.median(signed)),
        signed_mean=float(signed.mean()),
    )


def scenario_truth_grid(scenario_path: Path) -> Grid:
    """The scenario's gym map image, what the simulated lidar hits.

    `scenario_path` is a config/scenarios/<name>.yaml; it names the map
    (`environment.map`) under `map_directory`, beside the file.
    """
    scenario_path = Path(scenario_path)
    document = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    name = document["environment"]["map"]
    base = scenario_path.parent / document["map_directory"] / name
    return load_grid(base / f"{name}.yaml")


def mm_layer_points(mm: Path, layer: str, work: Path) -> np.ndarray:
    """The (x, y) of `layer`'s points in the metric map `mm`, from mm2txt.

    Shells out to `mm2txt` (mp2p_icp: ROS must be sourced), which writes
    `<mm stem>_<layer>.txt` into `work`, its working directory; the map stays
    ROS-free. Works for a voxel layer's occupied voxel centres.
    """
    work = Path(work)
    work.mkdir(parents=True, exist_ok=True)
    tool = shutil.which("mm2txt")
    if not tool:
        raise FileNotFoundError(
            "mm2txt is not on PATH: source /opt/ros/jazzy/setup.bash"
        )
    subprocess.run(
        [tool, str(Path(mm).resolve())],
        cwd=work,
        check=True,
        capture_output=True,
        env=os.environ,
    )
    return np.loadtxt(work / f"{Path(mm).stem}_{layer}.txt")[:, :2]


def nearest_p95(points: np.ndarray, reference: np.ndarray) -> float:
    """The 95th percentile of each of `points`' distance to the nearest
    point of `reference` (numpy's linear interpolation).

    scipy comes through pip, as a dependency of jax and f1tenth_gym_jax in
    the dev image, not through apt; it is imported here so that importers of
    this module that never call this function need no scipy.
    """
    from scipy.spatial import cKDTree

    return float(np.percentile(cKDTree(reference).query(points)[0], 95))
