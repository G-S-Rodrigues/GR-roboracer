import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

from sim.track_importer import _geometry, _load_points, import_track


def _world_pixel(
    image: Image.Image,
    metadata: dict,
    point: np.ndarray,
) -> int:
    origin_x, origin_y, _ = metadata["origin"]
    resolution = metadata["resolution"]
    # ROS convention: the cell containing the point; row 0 is the bottom.
    pixel_x = math.floor((float(point[0]) - origin_x) / resolution)
    pixel_y = (
        image.height - 1 - math.floor((float(point[1]) - origin_y) / resolution)
    )
    return int(image.getpixel((pixel_x, pixel_y)))


def test_importer_writes_the_four_gym_map_formats(tmp_path: Path) -> None:
    source = Path("config/tracks/analytic_circle.yaml")
    track_dir = import_track(source, tmp_path, resolution=0.1)

    expected = {
        "analytic_circle.yaml",
        "analytic_circle.png",
        "analytic_circle_centerline.csv",
        "analytic_circle_raceline.csv",
    }
    assert {path.name for path in track_dir.iterdir()} == expected

    map_metadata = yaml.safe_load(
        (track_dir / "analytic_circle.yaml").read_text(encoding="utf-8")
    )
    assert map_metadata["image"] == "analytic_circle.png"
    assert map_metadata["resolution"] == pytest.approx(0.1)
    assert len(map_metadata["origin"]) == 3

    centerline = np.loadtxt(
        track_dir / "analytic_circle_centerline.csv", delimiter=","
    )
    raceline = np.loadtxt(
        track_dir / "analytic_circle_raceline.csv", delimiter=";"
    )
    assert centerline.shape == (8, 4)
    assert raceline.shape == (8, 7)
    np.testing.assert_allclose(centerline[0], [10.0, 0.0, 2.0, 3.0])
    assert set(np.unique(Image.open(track_dir / "analytic_circle.png"))) == {
        0,
        255,
    }


def test_closed_loop_has_no_occupied_closing_seam(tmp_path: Path) -> None:
    source = Path("config/tracks/analytic_circle.yaml")
    track_dir = import_track(source, tmp_path, resolution=0.05)
    metadata = yaml.safe_load(
        (track_dir / "analytic_circle.yaml").read_text(encoding="utf-8")
    )
    image = Image.open(track_dir / "analytic_circle.png")
    canonical = yaml.safe_load(source.read_text(encoding="utf-8"))
    centerline = np.asarray(canonical["centerline"], dtype=float)[:, :2]
    closed_centerline = np.vstack((centerline, centerline[0]))

    occupied_samples = []
    for start, end in zip(
        closed_centerline[:-1], closed_centerline[1:], strict=True
    ):
        for fraction in np.linspace(0.0, 1.0, 21, endpoint=False):
            point = start + fraction * (end - start)
            if _world_pixel(image, metadata, point) != 255:
                occupied_samples.append(point.tolist())

    assert occupied_samples == []


def _rings(source: Path) -> tuple[np.ndarray, np.ndarray]:
    """The offset rings as (outer, inner), chosen by this file's own area."""
    _, points = _load_points(source)
    _, left, right, _ = _geometry(points)

    def area(ring: np.ndarray) -> float:
        nxt = np.roll(ring, -1, axis=0)
        return abs(
            float(np.sum(ring[:, 0] * nxt[:, 1] - nxt[:, 0] * ring[:, 1]))
        )

    return (left, right) if area(left) > area(right) else (right, left)


def _edge_distance(
    ring: np.ndarray, px: np.ndarray, py: np.ndarray
) -> np.ndarray:
    """Distance of each point to the ring's nearest edge, one edge at a time."""
    best = np.full(px.shape, np.inf)
    for a, b in zip(ring, np.roll(ring, -1, axis=0), strict=True):
        ab = b - a
        t = np.clip(
            ((px - a[0]) * ab[0] + (py - a[1]) * ab[1]) / (ab @ ab), 0.0, 1.0
        )
        best = np.minimum(
            best, np.hypot(px - a[0] - t * ab[0], py - a[1] - t * ab[1])
        )
    return best


def _inside_even_odd(ring: np.ndarray, px: np.ndarray, py: np.ndarray):
    """Even-odd crossing test, vectorised over points, looped over edges."""
    crossings = np.zeros(px.shape, dtype=int)
    for a, b in zip(ring, np.roll(ring, -1, axis=0), strict=True):
        straddles = (a[1] > py) != (b[1] > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_at = a[0] + (py - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
        crossings += straddles & (px < x_at)
    return crossings % 2 == 1


def _inside_convex(ring: np.ndarray, px: np.ndarray, py: np.ndarray):
    """Half-plane test for a convex ring: no code shared with even-odd."""
    nxt = np.roll(ring, -1, axis=0)
    first, second = nxt[0] - ring[0], nxt[1] - nxt[0]
    turn = np.sign(first[0] * second[1] - first[1] * second[0])
    inside = np.ones(px.shape, dtype=bool)
    for a, b in zip(ring, nxt, strict=True):
        side = (b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0])
        inside &= side * turn > 0.0
    return inside


def _corridor_mismatches(
    source: Path, tmp_path: Path, cells: str, convex: bool
) -> tuple[int, int]:
    """Count cells whose PNG value disagrees with the cell-centre oracle.

    Returns (mismatches, cells examined) with edge ties excluded.
    """
    track_dir = import_track(source, tmp_path, resolution=0.05)
    name = track_dir.name
    metadata = yaml.safe_load(
        (track_dir / f"{name}.yaml").read_text(encoding="utf-8")
    )
    resolution = metadata["resolution"]
    origin_x, origin_y, _ = metadata["origin"]
    image = np.asarray(Image.open(track_dir / f"{name}.png"))
    height, width = image.shape
    outer, inner = _rings(source)

    if cells == "all":
        rows, cols = np.indices((height, width))
        rows, cols = rows.ravel(), cols.ravel()  # rows from the bottom
    else:
        rng = np.random.default_rng(1030)
        samples = [
            np.column_stack(
                (rng.integers(0, width, 2000), rng.integers(0, height, 2000))
            )
        ]
        for ring in (outer, inner):
            for a, b in zip(ring, np.roll(ring, -1, axis=0), strict=True):
                edge = b - a
                length = float(np.hypot(*edge))
                normal = np.array([-edge[1], edge[0]]) / length
                along = np.arange(0.0, length, 0.01)[:, None]
                for offset in (0.0, 0.05, -0.05, 0.1, -0.1):
                    xy = a + along * edge / length + offset * normal
                    samples.append(
                        np.floor(
                            (xy - [origin_x, origin_y]) / resolution
                        ).astype(int)
                    )
        candidates = np.unique(np.vstack(samples), axis=0)
        candidates = candidates[
            (candidates[:, 0] >= 0)
            & (candidates[:, 0] < width)
            & (candidates[:, 1] >= 0)
            & (candidates[:, 1] < height)
        ]
        cols, rows = candidates[:, 0], candidates[:, 1]

    px = origin_x + (cols + 0.5) * resolution
    py = origin_y + (rows + 0.5) * resolution
    inside = _inside_convex if convex else _inside_even_odd
    expected = inside(outer, px, py) & ~inside(inner, px, py)
    actual = image[height - 1 - rows, cols] == 255
    # A tie is only worth deciding for a cell that disagrees.
    wrong = actual != expected
    tie = (
        np.minimum(
            _edge_distance(outer, px[wrong], py[wrong]),
            _edge_distance(inner, px[wrong], py[wrong]),
        )
        < 1e-6
    )
    return int(np.sum(~tie)), int(px.size - np.sum(wrong[wrong] & tie))


@pytest.mark.parametrize(
    ("track", "cells", "convex"),
    [
        ("analytic_circle", "all", True),
        ("spielberg", "walls", False),
    ],
    ids=["analytic_circle", "spielberg"],
)
def test_sim_1030_a_cell_is_free_iff_its_centre_is_in_the_corridor(
    tmp_path: Path, track: str, cells: str, convex: bool
) -> None:
    """SIM-1030: the PNG is in the ROS map convention.

    Row 0 is the bottom row and cell (i, j) is centred at
    origin + ((j + 0.5), (i + 0.5)) * resolution, the convention every
    reader of the file uses. 2026-10-14: the PIL importer misclassified
    1562 (circle) / 11534 (Spielberg) cells, up to 0.078 m from a wall. A
    half-pixel vertex shift alone still left 1342 / 7780. Spielberg is
    asymmetric, so it is the case that catches a y-flip or x/y swap.
    """
    mismatches, examined = _corridor_mismatches(
        Path(f"config/tracks/{track}.yaml"), tmp_path, cells, convex
    )
    assert examined > 1000
    assert mismatches == 0
