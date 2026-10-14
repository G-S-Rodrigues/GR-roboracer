"""Convert the canonical racing track YAML into gym_jax map assets."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


@dataclass(frozen=True)
class CenterlinePoint:
    x: float
    y: float
    curvature: float
    width_left: float
    width_right: float


def _load_points(path: Path) -> tuple[str, list[CenterlinePoint]]:
    # Validate with the authoritative C++ parser before deriving gym assets.
    import racing_common

    racing_common.Track.from_yaml(path)
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    name = str(document["metadata"]["name"])
    points = [
        CenterlinePoint(*(float(value) for value in row))
        for row in document["centerline"]
    ]
    if len(points) < 3:
        raise ValueError(
            "a closed track requires at least three centerline points"
        )
    return name, points


def _geometry(
    points: Sequence[CenterlinePoint],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    xy = np.asarray([(point.x, point.y) for point in points], dtype=float)
    previous_xy = np.roll(xy, 1, axis=0)
    next_xy = np.roll(xy, -1, axis=0)
    tangents = next_xy - previous_xy
    lengths = np.linalg.norm(tangents, axis=1)
    if np.any(lengths <= np.finfo(float).eps):
        raise ValueError("centerline contains a degenerate tangent")
    tangents /= lengths[:, None]
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    left_widths = np.asarray([point.width_left for point in points])
    right_widths = np.asarray([point.width_right for point in points])
    left = xy + normals * left_widths[:, None]
    right = xy - normals * right_widths[:, None]
    yaws = np.arctan2(tangents[:, 1], tangents[:, 0])
    return xy, left, right, yaws


def _inside_ring(
    ring: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    width: int,
    height: int,
) -> np.ndarray:
    """Even-odd fill of one ring at cell centres, one scanline per row."""
    row_y = origin[1] + (np.arange(height) + 0.5) * resolution
    crossings = np.zeros((height, width + 1), dtype=np.int32)
    per_row = np.zeros(height, dtype=np.int32)
    for start, end in zip(ring, np.roll(ring, -1, axis=0), strict=True):
        # Half-open straddle test: a vertex on a row centre counts once.
        straddles = (start[1] > row_y) != (end[1] > row_y)
        rows = np.flatnonzero(straddles)
        if rows.size == 0:
            continue
        x_at = start[0] + (row_y[rows] - start[1]) * (end[0] - start[0]) / (
            end[1] - start[1]
        )
        # Cells whose centre lies left of the crossing: columns [0, first).
        first = np.clip(
            np.ceil((x_at - origin[0]) / resolution - 0.5), 0, width
        ).astype(np.int64)
        np.add.at(crossings, (rows, first), 1)
        np.add.at(per_row, rows, 1)
    # A cell is inside when an odd number of crossings lie to its right.
    left_or_at = np.cumsum(crossings, axis=1)[:, :width]
    return (per_row[:, None] - left_or_at) % 2 == 1


def _rasterise_corridor(
    outer: np.ndarray,
    inner: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    width: int,
    height: int,
) -> np.ndarray:
    """Free iff the cell centre is inside `outer` and outside `inner`.

    The ROS map convention every reader uses (map_server, the gym's
    FLIP_TOP_BOTTOM + truncated-cell lookup, jax_pf): row 0 is the bottom
    row and cell (i, j) is centred at origin + ((j + 0.5), (i + 0.5)) * res.
    PIL's polygon fill is neither: it centres pixels on integer coordinates
    and paints the outline, which put the walls up to 0.078 m off (SIM-1030).
    """
    return _inside_ring(outer, origin, resolution, width, height) & ~(
        _inside_ring(inner, origin, resolution, width, height)
    )


def _write_image(
    path: Path,
    left: np.ndarray,
    right: np.ndarray,
    resolution: float,
    margin: float,
) -> tuple[float, float]:
    boundary = np.vstack((left, right))
    origin = boundary.min(axis=0) - margin
    maximum = boundary.max(axis=0) + margin
    width, height = np.ceil((maximum - origin) / resolution).astype(int) + 1
    if width <= 2 or height <= 2:
        raise ValueError("map dimensions are too small")

    def area(polygon: np.ndarray) -> float:
        next_polygon = np.roll(polygon, -1, axis=0)
        cross_products = (
            polygon[:, 0] * next_polygon[:, 1]
            - next_polygon[:, 0] * polygon[:, 1]
        )
        return abs(float(np.sum(cross_products))) / 2.0

    # Both offset boundaries follow the centerline orientation. Whichever has
    # the larger enclosed area is the outer edge, independent of whether the
    # canonical centerline runs clockwise or counter-clockwise.
    outer, inner = (left, right) if area(left) > area(right) else (right, left)

    free = _rasterise_corridor(
        outer, inner, origin, resolution, int(width), int(height)
    )
    # Row 0 of the array is the bottom row; the file stores the top row first
    # and the gym flips it back (FLIP_TOP_BOTTOM).
    image = Image.fromarray(np.where(free[::-1], 255, 0).astype(np.uint8))
    image.save(path)
    return float(origin[0]), float(origin[1])


def import_track(
    canonical_yaml: Path,
    output_root: Path,
    *,
    resolution: float = 0.05,
    margin: float = 1.0,
    reference_speed: float = 1.0,
) -> Path:
    """Write the four assets required by ``Track.from_track_name``."""
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    name, points = _load_points(canonical_yaml)
    xy, left, right, yaws = _geometry(points)

    track_dir = output_root / name
    track_dir.mkdir(parents=True, exist_ok=True)
    image_path = track_dir / f"{name}.png"
    origin_x, origin_y = _write_image(
        image_path, left, right, resolution, margin
    )

    map_yaml = {
        "image": image_path.name,
        "resolution": resolution,
        "origin": [origin_x, origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.45,
        "free_thresh": 0.196,
    }
    (track_dir / f"{name}.yaml").write_text(
        yaml.safe_dump(map_yaml, sort_keys=False), encoding="utf-8"
    )

    centerline = np.column_stack(
        (
            xy,
            [point.width_left for point in points],
            [point.width_right for point in points],
        )
    )
    np.savetxt(
        track_dir / f"{name}_centerline.csv",
        centerline,
        delimiter=",",
        fmt="%.9f",
    )

    closed_xy = np.vstack((xy, xy[0]))
    segment_lengths = np.linalg.norm(np.diff(closed_xy, axis=0), axis=1)
    arc_lengths = np.concatenate(([0.0], np.cumsum(segment_lengths[:-1])))
    raceline = np.column_stack(
        (
            arc_lengths,
            xy,
            yaws,
            [point.curvature for point in points],
            np.full(len(points), reference_speed),
            np.zeros(len(points)),
        )
    )
    np.savetxt(
        track_dir / f"{name}_raceline.csv",
        raceline,
        delimiter=";",
        fmt="%.9f",
    )
    return track_dir


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("canonical_yaml", type=Path)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--reference-speed", type=float, default=1.0)
    args = parser.parse_args()
    track_dir = import_track(
        args.canonical_yaml,
        args.output_root,
        resolution=args.resolution,
        reference_speed=args.reference_speed,
    )
    print(track_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
