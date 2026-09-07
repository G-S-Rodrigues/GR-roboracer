"""Convert the canonical racing track YAML into gym_jax map assets."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw


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

    polygon = np.vstack((left, right[::-1]))
    pixels = [
        (
            (float(x) - origin[0]) / resolution,
            height - 1 - (float(y) - origin[1]) / resolution,
        )
        for x, y in polygon
    ]
    image = Image.new("L", (int(width), int(height)), color=0)
    ImageDraw.Draw(image).polygon(pixels, fill=255)
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
