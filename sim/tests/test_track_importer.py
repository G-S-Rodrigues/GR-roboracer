import filecmp
import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image
from scipy import ndimage

from sim.track_importer import (
    _geometry,
    _load_points,
    _remove_folds,
    _walls,
    import_track,
)


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
    """The offset rings as (outer, inner), chosen by this file's own area.

    These are the importer's fold-free walls (`_walls`); SIM-1040 and
    SIM-1060 guard their geometry.
    """
    _, points = _load_points(source)
    _, left, right, _ = _walls(points)

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


TRACKS = ["analytic_circle", "spielberg"]


def _orient(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """Twice the signed area of triangle (a, b, c), vectorised over c."""
    return (b[0] - a[0]) * (c[..., 1] - a[1]) - (b[1] - a[1]) * (
        c[..., 0] - a[0]
    )


def _crossings(
    ring: np.ndarray, other: np.ndarray | None = None
) -> list[tuple[int, int, tuple[float, float]]]:
    """Proper crossings of a ring with itself (or with another ring).

    Brute force by orientation tests, deliberately not the importer's
    parametric solve, so the guard does not share the code it checks.
    Returns (segment i, segment j, (x, y)); for one ring i < j and
    adjacent segments are excluded.
    """
    n = len(ring)
    starts, ends = ring, np.roll(ring, -1, axis=0)
    own = other is None
    other_starts = starts if own else other
    other_ends = np.roll(other_starts, -1, axis=0)
    found = []
    for i in range(n):
        p, q = starts[i], ends[i]
        o1 = _orient(p, q, other_starts)
        o2 = _orient(p, q, other_ends)
        o3 = np.array(
            [
                _orient(c, d, p)
                for c, d in zip(other_starts, other_ends, strict=True)
            ]
        )
        o4 = np.array(
            [
                _orient(c, d, q)
                for c, d in zip(other_starts, other_ends, strict=True)
            ]
        )
        hit = (o1 * o2 < 0.0) & (o3 * o4 < 0.0)
        for j in np.flatnonzero(hit):
            if own and (j <= i + 1 or (i == 0 and j == n - 1)):
                continue
            point = p + o3[j] / (o3[j] - o4[j]) * (q - p)
            found.append((i, int(j), (float(point[0]), float(point[1]))))
    return found


def _describe(name: str, found: list) -> str:
    return "; ".join(
        f"{name} crosses at segments {i}/{j} near ({x:.3f}, {y:.3f})"
        for i, j, (x, y) in found
    )


def _rounded_square(half: float, corner: float, spacing: float) -> np.ndarray:
    """A counter-clockwise rounded square at even arc-length spacing."""
    c = half - corner
    parts = []
    for centre, start in (
        ((c, -c), -90.0),
        ((c, c), 0.0),
        ((-c, c), 90.0),
        ((-c, -c), 180.0),
    ):
        angle = np.radians(start + np.linspace(0.0, 90.0, 400))
        parts.append(
            np.column_stack(
                (
                    centre[0] + corner * np.cos(angle),
                    centre[1] + corner * np.sin(angle),
                )
            )
        )
    dense = np.vstack(parts)
    closed = np.vstack((dense, dense[0]))
    along = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1)))
    )
    count = int(round(along[-1] / spacing))
    sample = np.linspace(0.0, along[-1], count, endpoint=False)
    return np.column_stack(
        (
            np.interp(sample, along, closed[:, 0]),
            np.interp(sample, along, closed[:, 1]),
        )
    )


def _offset_left(ring: np.ndarray, distance: float) -> np.ndarray:
    tangents = np.roll(ring, -1, axis=0) - np.roll(ring, 1, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    return ring + distance * np.column_stack((-tangents[:, 1], tangents[:, 0]))


def _circle(radius: float, spacing: float) -> np.ndarray:
    count = int(round(2.0 * np.pi * radius / spacing))
    angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    return np.column_stack((radius * np.cos(angle), radius * np.sin(angle)))


CURL_POINTS = 80
CURL_AT = 100
CURL_THETA = np.linspace(-np.pi, np.pi, CURL_POINTS)


def _with_curl(circle: np.ndarray, size: float) -> np.ndarray:
    """Replace circle[CURL_AT] by a same-winding (counter-clockwise) curl.

    A trochoid whose loop lies on the ring's inside and turns the ring's
    way, of radius about `size`.
    """
    tangent = circle[CURL_AT + 1] - circle[CURL_AT - 1]
    tangent /= np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])
    theta = CURL_THETA
    u = 0.4 * size * theta - size * np.sin(theta)
    v = size * np.cos(theta) + size
    curl = circle[CURL_AT] + np.outer(u, tangent) + np.outer(v, normal)
    return np.vstack((circle[:CURL_AT], curl, circle[CURL_AT + 1 :]))


def _rows_in(rows: np.ndarray, table: np.ndarray) -> np.ndarray:
    return (rows[:, None, :] == table[None, :, :]).all(axis=2).any(axis=1)


def test_sim_1060_remove_folds_cuts_swallowtails_and_curls() -> None:
    """SIM-1060: a fold is cut at its crossing; a loop wider than max_loop
    raises; a crossing-free ring is returned untouched."""
    # (a) the inside offset (0.8) of a rounded square (corner r 0.5) folds
    # into one swallowtail per corner.
    square = _rounded_square(5.0, 0.5, 0.1)
    inner = _offset_left(square, 0.8)
    assert len(_crossings(inner)) == 4, "the construction must fold 4 times"
    cut = _remove_folds(inner, 0.8)
    assert _crossings(cut) == [], _describe("inner", _crossings(cut))
    assert int(np.sum(~_rows_in(cut, inner))) == 4
    assert len(cut) < len(inner)

    # (b) a same-winding curl of radius 0.05 on a radius-5 circle.
    circle = _circle(5.0, 0.1)
    curled = _with_curl(circle, 0.05)
    assert len(_crossings(curled)) >= 1, "the curl must cross itself"
    cut = _remove_folds(curled, 1.0)
    assert _crossings(cut) == [], _describe("ring", _crossings(cut))
    assert int(np.sum(~_rows_in(cut, curled))) == 1, "only the crossing is new"
    curl = curled[CURL_AT : CURL_AT + CURL_POINTS]
    # The trochoid closes its loop where u(theta) = 0, theta about 2.12;
    # the lead-in and lead-out outside it are ordinary path and stay.
    loop = np.abs(CURL_THETA) < 2.10
    assert not _rows_in(curl[loop], cut).any(), "loop points remain"
    assert _rows_in(curl[np.abs(CURL_THETA) > 2.14], cut).all()
    assert _rows_in(np.delete(circle, CURL_AT, axis=0), cut).all()

    # (c) the same curl at radius 1.5 is a loop wider than max_loop.
    with pytest.raises(ValueError, match="self-intersect"):
        _remove_folds(_with_curl(circle, 1.5), 1.0)

    # (d) a crossing-free ring comes back equal.
    assert np.array_equal(_remove_folds(circle, 1.0), circle)

    # (e) the analytic circle's walls need no cutting.
    _, points = _load_points(Path("config/tracks/analytic_circle.yaml"))
    _, left, right, _ = _geometry(points)
    _, walls_left, walls_right, _ = _walls(points)
    assert np.array_equal(walls_left, left)
    assert np.array_equal(walls_right, right)


@pytest.mark.parametrize("track", TRACKS)
def test_sim_1040_the_importer_walls_do_not_cross(track: str) -> None:
    """SIM-1040: neither wall crosses itself or the other.

    2026-10-20: Spielberg's inner wall crossed itself at segments 275/282
    near (-74.878, 52.090), where the centerline's radius (0.64 m) is under
    its half-width (1.1 m).
    """
    _, points = _load_points(Path(f"config/tracks/{track}.yaml"))
    _, left, right, _ = _walls(points)
    found = (
        [("left wall", *c) for c in _crossings(left)]
        + [("right wall", *c) for c in _crossings(right)]
        + [("left x right", *c) for c in _crossings(left, right)]
    )
    assert not found, "; ".join(
        f"{name} crosses itself at segments {i}/{j} near ({x:.3f}, {y:.3f})"
        for name, i, j, (x, y) in found
    )


@pytest.mark.parametrize("track", TRACKS)
def test_sim_1050_the_png_has_exactly_two_occupied_components(
    tmp_path: Path, track: str
) -> None:
    """SIM-1050: the outer wall touches the border and the island is
    enclosed; nothing else is occupied. 2026-10-20: the fold drew an
    11-cell detached spur."""
    track_dir = import_track(Path(f"config/tracks/{track}.yaml"), tmp_path)
    metadata = yaml.safe_load(
        (track_dir / f"{track_dir.name}.yaml").read_text(encoding="utf-8")
    )
    image = np.asarray(Image.open(track_dir / f"{track_dir.name}.png"))
    labels, count = ndimage.label(image != 255, structure=np.ones((3, 3)))
    edge = np.concatenate((labels[0], labels[-1], labels[:, 0], labels[:, -1]))
    border = set(np.unique(edge).tolist()) - {0}
    resolution = metadata["resolution"]
    origin_x, origin_y, _ = metadata["origin"]
    height = image.shape[0]
    described = []
    for label in range(1, count + 1):
        rows, cols = np.nonzero(labels == label)
        described.append(
            f"{rows.size} cells, x "
            f"{origin_x + cols.min() * resolution:.3f}.."
            f"{origin_x + (cols.max() + 1) * resolution:.3f}, y "
            f"{origin_y + (height - 1 - rows.max()) * resolution:.3f}.."
            f"{origin_y + (height - rows.min()) * resolution:.3f}"
            + (" (border)" if label in border else "")
        )
    assert count == 2 and len(border) == 1, (
        f"{count} occupied components; " + "; ".join(described)
    )


def _same_asset(left: Path, right: Path) -> bool:
    """Equal bytes, except a PNG is equal by decoded content.

    An encoder upgrade may change a PNG's bytes and not one pixel, so the
    PNG compares by size, mode and every pixel (Pillow is not pinned).
    """
    if left.suffix != ".png":
        return filecmp.cmp(left, right, shallow=False)
    with Image.open(left) as one, Image.open(right) as two:
        return (
            one.size == two.size
            and one.mode == two.mode
            and np.array_equal(np.asarray(one), np.asarray(two))
        )


def test_sim_1070_a_reencoded_png_is_the_same_asset_a_changed_pixel_is_not(
    tmp_path: Path,
) -> None:
    """SIM-1070 helper: bytes may differ, pixels may not."""
    source = Path("config/scenarios/maps/Spielberg/Spielberg.png")
    with Image.open(source) as image:
        image.save(tmp_path / "recompressed.png", compress_level=0)
        pixels = np.asarray(image).copy()
    assert (tmp_path / "recompressed.png").read_bytes() != source.read_bytes()
    assert _same_asset(source, tmp_path / "recompressed.png")
    pixels[0, 0] = 255 - pixels[0, 0]
    Image.fromarray(pixels).save(tmp_path / "one_pixel.png")
    assert not _same_asset(source, tmp_path / "one_pixel.png")


@pytest.mark.parametrize(
    ("track", "name"),
    [("analytic_circle", "analytic_circle"), ("spielberg", "Spielberg")],
)
def test_sim_1070_the_committed_gym_assets_are_the_importer_output(
    tmp_path: Path, track: str, name: str
) -> None:
    """SIM-1070: no hand edit and no stale redraw of a committed map."""
    track_dir = import_track(Path(f"config/tracks/{track}.yaml"), tmp_path)
    committed = Path("config/scenarios/maps") / name
    files = sorted(path.name for path in track_dir.iterdir())
    assert len(files) == 4
    stale = [
        file
        for file in files
        if not (committed / file).is_file()
        or not _same_asset(track_dir / file, committed / file)
    ]
    assert stale == [], f"{name}: committed {stale} differ from the importer"
