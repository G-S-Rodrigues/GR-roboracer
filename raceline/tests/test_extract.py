"""Tier 1: the map -> track extractor and the canonical track file."""

from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
SPIELBERG = REPO / "config" / "tracks" / "spielberg.yaml"


def _asymmetric_track():
    from raceline.track import Track

    t = np.linspace(0.0, 2 * np.pi, 40, endpoint=False)
    return Track(
        name="oval",
        xy=np.column_stack([8.0 * np.cos(t), 5.0 * np.sin(t)]),
        curvature=0.1 * np.cos(t),
        w_left=1.0 + 0.2 * np.sin(t) ** 2,
        w_right=0.7 + 0.1 * np.cos(t) ** 2,
    )


def test_raceline_1100_track_yaml_matches_the_committed_format(tmp_path):
    """RACELINE-1100: the canonical track file (left width before right)."""
    from raceline.track import load_track, write_track

    # the committed file: 5 columns in canonical order, widths 1.1 / 1.1
    spielberg = load_track(SPIELBERG)
    assert spielberg.name == "Spielberg"
    assert spielberg.xy.shape == (864, 2)
    assert np.allclose(spielberg.w_left, 1.1) and np.allclose(
        spielberg.w_right, 1.1
    )
    assert np.allclose(spielberg.xy[0], [0.0, 0.0])
    assert spielberg.curvature[0] == pytest.approx(0.0000471)

    # round trip of an asymmetric track: left and right must not swap
    track = _asymmetric_track()
    out = write_track(track, tmp_path / "t.yaml", source="unit test")
    back = load_track(out)
    assert back.name == "oval"
    assert np.allclose(back.xy, track.xy, atol=1e-8)
    assert np.allclose(back.curvature, track.curvature, atol=1e-8)
    assert np.allclose(back.w_left, track.w_left, atol=1e-6)
    assert np.allclose(back.w_right, track.w_right, atol=1e-6)
    assert not np.allclose(back.w_left, back.w_right)
    assert out.read_text().startswith("# unit test")

    # a name YAML would read as something else, or fail on, round-trips
    from dataclasses import replace as _replace

    for odd in ("yes", "1.0", "a: b", "#x"):
        p = write_track(_replace(track, name=odd), tmp_path / "n.yaml")
        assert load_track(p).name == odd

    # rejected, as racing_common's track.cpp rejects them
    from dataclasses import replace

    with pytest.raises(ValueError):
        write_track(replace(track, w_left=track.w_left * 0), tmp_path / "a")
    with pytest.raises(ValueError):
        write_track(
            replace(track, xy=np.vstack([track.xy[:1], track.xy])),
            tmp_path / "b",
        )
    # stricter than track.cpp: fewer than 3 points
    with pytest.raises(ValueError):
        write_track(
            replace(
                track,
                xy=track.xy[:2],
                curvature=track.curvature[:2],
                w_left=track.w_left[:2],
                w_right=track.w_right[:2],
            ),
            tmp_path / "c",
        )
    good = tmp_path / "g.yaml"
    write_track(track, good)
    text = good.read_text()
    for name, bad in (
        ("v", text.replace("format_version: 1", "format_version: 2")),
        ("u", text.replace("units: meters", "units: feet")),
        ("o", text.replace("closed: true", "closed: false")),
        ("r", text.replace("  - [", "  - [1.0, ", 1)),
    ):
        p = tmp_path / f"bad_{name}.yaml"
        p.write_text(bad)
        with pytest.raises(ValueError):
            load_track(p)


# ---- synthetic grids -------------------------------------------------------

RES = 0.05
# Measured on the annulus (R 5, widths 1, 0.05 m cells) at 44b2942, after the
# corners were re-centred (the 2026-10-13 numbers, radius off 0.0706, widths
# 0.0874, kappa * R 0.86..1.18, no longer hold). The extractor is
# deterministic, so each tolerance is the measurement plus a margin of about
# a third of a cell:
#   centerline radius off by up to 0.0145 m (mean 0.0089)   -> RADIUS_TOL 0.02
#   widths off by up to 0.0191 (left) / 0.0313 (right)      -> WIDTH_TOL 0.04
#   kappa * R spans 0.958..1.050, mean 1.0032: max deviation 0.0498
#                                       -> KAPPA_MAX_REL 0.07, MEAN_REL 0.01
RADIUS_TOL = 0.02
WIDTH_TOL = 0.04
KAPPA_MAX_REL = 0.07
KAPPA_MEAN_REL = 0.01
# Part 2.3, walls="observed" with a bridged gap (C1 sweep: 11,702 runs that
# MAX_BRIDGE_TURN_DEG / MAX_BRIDGE_M admit, R_in 4.0-6.4, outer / inner / both
# walls, gaps 16-60 deg, gap centres every 5 deg). The extractor is
# deterministic; each tolerance is the worst measurement plus about a third of
# a cell:
#   a bridged side narrower than truth by up to 0.259 (R_in 5.2, both walls,
#   24 deg, c -60)                                   -> BRIDGE_WIDTH_TOL 0.28
#   centreline radius off by up to 0.135 (R_in 5.4, outer, 26 deg, c 0)
#                                                    -> BRIDGE_RADIUS_TOL 0.155
# Wider than truth never exceeds +0.034 per side (R_in 5.4, inner 38 deg, c 110)
# or +0.029 total; WIDTH_TOL 0.04 holds unchanged.
BRIDGE_WIDTH_TOL = 0.28
BRIDGE_RADIUS_TOL = 0.155


def _grid(data, origin=(-7.0, -7.0)):
    from raceline.extract import OccupancyGrid, Pose2D

    return OccupancyGrid(
        data=data.astype(np.int8),
        resolution=RES,
        origin=Pose2D(origin[0], origin[1], 0.0),
    )


def _annulus(r_in=4.0, r_out=6.0, half=7.0, gap_deg=0.0):
    """Free annulus (0) in unknown space (-1) with occupied edges (100)."""
    n = int(round(2 * half / RES))
    c = (np.arange(n) + 0.5) * RES - half  # cell centres, origin = -half
    x, y = np.meshgrid(c, c)
    r = np.hypot(x, y)
    data = np.full((n, n), -1, dtype=np.int8)
    free = (r > r_in) & (r < r_out)
    if gap_deg:
        free &= ~((np.degrees(np.arctan2(y, x)) % 360.0) < gap_deg)
    data[free] = 0
    data[(np.abs(r - r_in) <= RES) | (np.abs(r - r_out) <= RES)] = 100
    data[~free & (np.abs(r - 5.0) < 0.9) & (data != 100)] = 100
    return _grid(np.where(free, 0, data))


def _varying_ring(r_in=4.0, base=5.8, amp=1.0, half=8.0):
    """Free ring whose outer wall is r_in + (base - r_in) + amp*cos(theta):
    the corridor narrows to ``base - amp - r_in`` m on one side."""
    n = int(round(2 * half / RES))
    c = (np.arange(n) + 0.5) * RES - half
    x, y = np.meshgrid(c, c)
    r = np.hypot(x, y)
    r_out = base + amp * np.cos(np.arctan2(y, x))
    free = (r > r_in) & (r < r_out)
    data = np.full((n, n), -1, dtype=np.int8)
    data[free] = 0
    data[(np.abs(r - r_in) <= RES) | (np.abs(r - r_out) <= RES)] = 100
    return _grid(np.where(free, 0, data), origin=(-half, -half))


def _ring(
    r_in=4.0,
    r_out=6.0,
    half=7.0,
    xmax=None,
    ymax=None,
    gap=None,
    gap_wall="outer",
    c_deg=50.0,
    fan_out=0.8,
    fan_in=0.8,
):
    """``_annulus`` generalised: a frame cropped at ``xmax`` / ``ymax``, and a
    gap of ``gap`` degrees centred at ``c_deg`` in the outer, inner or both
    walls. The lidar saw through the gap: free cells fan out to ``fan_out``
    beyond the outer wall / ``fan_in`` into the island, and the wall cells are
    unknown."""
    x1 = half if xmax is None else xmax
    y1 = half if ymax is None else ymax
    nx, ny = int(round((x1 + half) / RES)), int(round((y1 + half) / RES))
    x, y = np.meshgrid(
        -half + (np.arange(nx) + 0.5) * RES, -half + (np.arange(ny) + 0.5) * RES
    )
    r = np.hypot(x, y)
    ang = np.degrees(np.arctan2(y, x))
    data = np.full((ny, nx), -1, dtype=np.int8)
    free = (r > r_in) & (r < r_out)
    data[(np.abs(r - r_in) <= RES) | (np.abs(r - r_out) <= RES)] = 100
    if gap is not None:
        sect = np.abs((ang - c_deg + 180.0) % 360.0 - 180.0) < gap / 2
        if gap_wall in ("outer", "both"):
            data[sect & (np.abs(r - r_out) <= RES)] = -1
            free |= sect & (r > r_in) & (r < r_out + fan_out)
        if gap_wall in ("inner", "both"):
            data[sect & (np.abs(r - r_in) <= RES)] = -1
            free |= sect & (r > r_in - fan_in) & (r < r_out)
    data[free] = 0
    return _grid(data, origin=(-half, -half))


def _ring_start(r_in=4.0):
    from raceline.extract import Pose2D

    return Pose2D(0.0, -(r_in + 1.0), 0.0)


def _extract(grid, start, direction="ccw", walls="observed"):
    from raceline.extract import extract_track

    return extract_track(
        grid, grid.resolution, grid.origin, start, direction, walls=walls
    )


def _area(xy):
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def _assert_annulus(track):
    r = np.hypot(track.xy[:, 0], track.xy[:, 1])
    assert np.abs(r - 5.0).max() <= RADIUS_TOL
    assert np.abs(track.w_left - 1.0).max() <= WIDTH_TOL
    assert np.abs(track.w_right - 1.0).max() <= WIDTH_TOL
    k = track.curvature * 5.0  # ccw: left turn, positive
    assert abs(k.mean() - 1.0) <= KAPPA_MEAN_REL
    assert np.abs(k - 1.0).max() <= KAPPA_MAX_REL


def test_raceline_1060_load_map_decodes_a_map_server_png(tmp_path):
    """RACELINE-1060: ROS orientation (row 0 = bottom), trinary thresholds."""
    import cv2
    from raceline.extract import load_map

    px = np.array([[255, 0, 128], [0, 255, 128]], dtype=np.uint8)
    cv2.imwrite(str(tmp_path / "m.png"), px)

    def write_yaml(negate):
        (tmp_path / "m.yaml").write_text(
            "image: m.png\nresolution: 0.1\norigin: [1.5, -2.0, 0.0]\n"
            f"negate: {negate}\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        )
        return tmp_path / "m.yaml"

    g = load_map(write_yaml(0))
    assert g.resolution == 0.1
    assert (g.origin.x, g.origin.y, g.origin.yaw) == (1.5, -2.0, 0.0)
    # the PNG's bottom row [0, 255, 128] is row 0: black = occupied
    assert g.data.tolist() == [[100, 0, -1], [0, 100, -1]]
    assert g.data.dtype == np.int8
    # negate: 1 inverts black and white
    assert load_map(write_yaml(1)).data.tolist() == [[0, 100, -1], [100, 0, -1]]
    # a colour PNG is not a map
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    cv2.imwrite(str(tmp_path / "m.png"), rgb)
    with pytest.raises(ValueError):
        load_map(write_yaml(0))


def test_raceline_1070_annulus_has_radius_width_and_curvature():
    """RACELINE-1070: R = 5 centerline, widths 1, kappa 1/5, closed, 0.1 m."""
    from raceline.extract import Pose2D

    track = _extract(_annulus(), Pose2D(0.0, -5.0, 0.0))
    _assert_annulus(track)
    seg = np.roll(track.xy, -1, axis=0) - track.xy  # includes the closing one
    step = np.hypot(seg[:, 0], seg[:, 1])
    assert step.min() >= 0.08 and step.max() <= 0.12
    assert not np.allclose(track.xy[0], track.xy[-1])  # no repeated point


def test_raceline_1080_start_direction_and_bound_sides():
    """RACELINE-1080: point 0 at start_pose, direction from yaw, left/right."""
    from raceline.extract import Pose2D, track_bounds

    grid = _annulus()
    ccw = _extract(grid, Pose2D(0.0, -5.0, 0.0), "ccw")
    assert np.hypot(*(ccw.xy[0] - [0.0, -5.0])) <= 0.1
    assert _area(ccw.xy) > 0
    right, left = track_bounds(grid, ccw.xy)
    assert np.median(np.hypot(left[:, 0], left[:, 1])) == pytest.approx(
        4.0, abs=RES
    )  # ccw: the inner wall is on the left
    assert np.median(np.hypot(right[:, 0], right[:, 1])) == pytest.approx(
        6.0, abs=RES
    )

    cw = _extract(grid, Pose2D(0.0, -5.0, np.pi), "cw")
    assert _area(cw.xy) < 0
    right, left = track_bounds(grid, cw.xy)
    assert np.median(np.hypot(left[:, 0], left[:, 1])) == pytest.approx(
        6.0, abs=RES
    )  # cw: the outer wall is on the left
    # a direction that contradicts the yaw is an error, never a flip
    with pytest.raises(ValueError, match="direction"):
        _extract(grid, Pose2D(0.0, -5.0, 0.0), "cw")
    # the explicit resolution/origin must agree with the grid's
    from raceline.extract import extract_track

    with pytest.raises(ValueError):
        extract_track(grid, 0.1, grid.origin, Pose2D(0.0, -5.0, 0.0), "ccw")


def test_raceline_1085_a_centerline_outside_the_corridor_raises():
    """RACELINE-1085: a corridor that narrows at a corner (0.8..2.8 m) puts
    the re-centred line across the inner wall; that is an error, never a
    plausible track."""
    from raceline.extract import Pose2D

    with pytest.raises(ValueError, match="centerline"):
        _extract(_varying_ring(), Pose2D(0.0, -4.9, 0.0))
    # a gentle taper (1.5..2.5 m) stays inside, and still extracts
    track = _extract(_varying_ring(base=5.5, amp=0.5), Pose2D(0.0, -4.75, 0.0))
    assert track.w_left.min() > 0.0 and track.w_right.min() > 0.0


def test_raceline_1090_no_usable_loop_raises_and_noise_is_cleaned():
    """RACELINE-1090: no loop near the start raises; stray noise is cleaned."""
    from raceline.extract import Pose2D

    start = Pose2D(0.0, -5.0, 0.0)
    with pytest.raises(ValueError, match="loop"):
        _extract(_annulus(gap_deg=20.0), start)  # a C, not a ring
    with pytest.raises(ValueError, match="loop"):
        _extract(_annulus(), Pose2D(0.0, 0.0, 0.0))  # 5 m from the loop

    # a stray wall island and a dashed unknown line along the driven path
    # (both seen in the committed survey) must not change the result
    grid = _annulus()
    data = grid.data.copy()
    n = data.shape[0]
    c = (np.arange(n) + 0.5) * RES - 7.0
    x, y = np.meshgrid(c, c)
    data[(np.abs(x - 0.0) < 0.06) & (np.abs(y + 5.0) < 0.06)] = 100
    ang = np.degrees(np.arctan2(y, x)) % 360.0
    dashes = (np.abs(np.hypot(x, y) - 5.0) < 0.03) & (
        (ang * 3.0).astype(int) % 2 == 0
    )
    data[dashes & (data == 0)] = -1
    noisy = _extract(_grid(data), Pose2D(3.5, -3.6, 0.0))
    _assert_annulus(noisy)


# ---- Spielberg ground truth ------------------------------------------------

MAPS = REPO / "config" / "scenarios" / "maps" / "Spielberg"
GYM_MAP = MAPS / "Spielberg.yaml"
SURVEY_MAP = MAPS / "slam" / "Spielberg.yaml"
# Tolerances, from the measurements of 2026-10-13 (raceline/README.md has the
# table). 2026-10-14: part 2.1 redrew the gym PNG in the ROS cell-centre
# convention (SIM-1030), moving its walls up to 0.078 m. The gym-case figures
# below that part 2.1 changed are re-measured and dated; the survey is
# unchanged, so its figures are too. Lateral ceilings are one cell on the gym
# map and three on the survey.
#   gym PNG vs its own free-cell edges, both ways: max 0.0344 / 0.0493 m
#     (left), 0.0000 (right)                              -> GYM_EDGE_TOL 0.05
#     (2026-10-14, after part 2.1: 0.0357 / 0.0494 m left, 0.0000 right)
#   survey vs the gym PNG's edges, extracted bound -> edge: max 0.1293 (left),
#     0.1135 (right); edge -> bound (left): 98.0 % of the edge points within
#     0.15 m. (2026-10-14, against the part 2.1 PNG: 0.0900 / 0.0915 m, and
#     97.85 % within 0.15 m.) The rest is a leak: the survey's free space
#     runs past the wall at the top-left hairpin out to the map frame (up to
#     0.96 m), which no extractor can know
#                                                         -> SLAM_EDGE_TOL 0.15
#     The leak is pinned, not just tolerated (measured at 44b2942):
#       edge -> bound coverage within 0.15 m: 98.02 %    -> SLAM_MIN_COVERAGE
#                                                           0.975
#         (2026-10-14, part 2.1 PNG: 97.85 %)
#       share of the left bound on the map frame: 6.03 % -> SLAM_MAX_FRAME_SHARE
#                                                           0.07
#       largest total width (left + right) on the survey: 3.158 m against the
#       committed 2.2, at (-75.6, 53.3)                  -> SLAM_MAX_TOTAL_WIDTH
#                                                           3.2
#   (2026-10-14, part 2.1 PNG: length ratio 0.9981 gym, 1.0017 survey; the
#   figures below are the pre-2.1 measurements)
#   centerline length / committed length: 0.9863 (gym), 0.9882 (survey), and
#     0.9859 / 0.9858 at 0.025 / 0.0125 m cells. The medial axis cuts the
#     hairpins whatever the resolution; the owner allowed a modest increase of
#     the 1 % ceiling. Re-centring the corners on the outer bound (the
#     corner clean-up of 2026-10-13) lifted it to 0.9986 (gym) / 1.0017
#     (survey)                                            -> LENGTH_RTOL 0.003
#   mean bound length / mean PNG edge length: 0.004 %    -> BOUND_LEN_RTOL 0.001
#     (2026-10-14, part 2.1 PNG: 0.0050 %)
#   median |width - 1.1| (left / right): 0.0083 / 0.0062 m (gym), 0.0186 /
#     0.0144 (survey), at 44b2942                   -> WIDTH_MEDIAN_TOL 0.03
#     (2026-10-14, part 2.1 PNG: 0.0079 / 0.0054 m gym; survey unchanged)
GYM_EDGE_TOL = 0.05
SLAM_EDGE_TOL = 0.15
SLAM_MIN_COVERAGE = 0.975
SLAM_MAX_FRAME_SHARE = 0.07
SLAM_MAX_TOTAL_WIDTH = 3.2
LENGTH_RTOL = 0.003
BOUND_LEN_RTOL = 0.001
WIDTH_MEDIAN_TOL = 0.03
FRAME_CELLS = 2  # bound points this close to the map frame are not walls
MIN_WALL_M2 = 0.5  # the test's own island fill, independent of the extractor


def _committed():
    from raceline.track import load_track

    return load_track(SPIELBERG)


def _resample(poly, step=0.02):
    closed = np.vstack([poly, poly[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(closed, axis=0).T))])
    t = np.arange(0.0, s[-1], step)
    return np.column_stack(
        [np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])]
    )


def _polyline_length(poly):
    return float(
        np.hypot(*np.diff(np.vstack([poly, poly[:1]]), axis=0).T).sum()
    )


def _png_edges(grid):
    """The PNG's own free-cell wall edges: the two long contours of its free
    mask at the cell edge, stray wall islands filled."""
    from scipy import ndimage
    from skimage.measure import find_contours

    free = grid.data == 0
    walls, n = ndimage.label(~free)
    area = ndimage.sum(np.ones_like(walls), walls, index=np.arange(1, n + 1))
    for i in np.flatnonzero(area * grid.resolution**2 < MIN_WALL_M2) + 1:
        free |= walls == i
    # features narrower than 5 cells (0.25 m) are cracks, not walls
    free = ndimage.binary_closing(
        np.pad(free, 3), structure=np.ones((5, 5), bool)
    )[3:-3, 3:-3]
    contours = [
        c
        for c in find_contours(np.pad(free.astype(float), 1), 0.5)
        if np.allclose(c[0], c[-1])
    ]
    contours = sorted(contours, key=len)[-2:]
    assert len(contours) == 2
    return [
        np.column_stack(
            [
                grid.origin.x + (c[:-1, 1] - 1 + 0.5) * grid.resolution,
                grid.origin.y + (c[:-1, 0] - 1 + 0.5) * grid.resolution,
            ]
        )
        for c in contours
    ]


def _extract_spielberg(grid, committed, walls="observed"):
    from raceline.extract import Pose2D, extract_track, track_bounds

    p0, p1 = committed.xy[0], committed.xy[1]
    yaw = float(np.arctan2(p1[1] - p0[1], p1[0] - p0[0]))
    direction = "ccw" if _area(committed.xy) > 0 else "cw"
    track = extract_track(
        grid,
        grid.resolution,
        grid.origin,
        Pose2D(float(p0[0]), float(p0[1]), yaw),
        direction,
        walls=walls,
    )
    right, left = track_bounds(grid, track.xy, walls=walls)
    return track, right, left, yaw


def _inside(grid, pts, cells):
    """Points farther than ``cells`` cells from the map frame."""
    h, w = grid.data.shape
    m = cells * grid.resolution
    x0, y0 = grid.origin.x + m, grid.origin.y + m
    x1 = grid.origin.x + w * grid.resolution - m
    y1 = grid.origin.y + h * grid.resolution - m
    return (
        (pts[:, 0] > x0)
        & (pts[:, 0] < x1)
        & (pts[:, 1] > y0)
        & (pts[:, 1] < y1)
    )


def _edge_errors(grid, bound, edges):
    """Max distance both ways between a bound and the nearest PNG edge,
    ignoring points on (or beyond) the extracted map's frame."""
    from scipy.spatial import cKDTree

    b = _resample(bound)
    sample = b[:: max(len(b) // 200, 1)]
    edge = min(
        (_resample(e) for e in edges),
        key=lambda e: cKDTree(e).query(sample)[0].mean(),
    )
    keep_b, keep_e = (
        _inside(grid, b, FRAME_CELLS),
        _inside(grid, edge, FRAME_CELLS),
    )
    to_edge = cKDTree(edge).query(b[keep_b])[0]
    to_bound = cKDTree(b).query(edge[keep_e])[0]
    return to_edge.max(), to_bound.max(), 1.0 - keep_b.mean()


def _coverage(grid, bound, edges, tol):
    """Share of the PNG's edge points (inside the frame) within ``tol`` of
    the bound."""
    from scipy.spatial import cKDTree

    b = _resample(bound)
    sample = b[:: max(len(b) // 200, 1)]
    edge = min(
        (_resample(e) for e in edges),
        key=lambda e: cKDTree(e).query(sample)[0].mean(),
    )
    edge = edge[_inside(grid, edge, FRAME_CELLS)]
    return float(np.mean(cKDTree(b).query(edge)[0] <= tol))


def _side_fraction(bound, committed):
    """Fraction of the bound's points on the committed centerline's left."""
    from scipy.spatial import cKDTree

    pts = _resample(bound, 0.1)
    _, i = cKDTree(committed.xy).query(pts)
    t = np.roll(committed.xy, -1, axis=0) - np.roll(committed.xy, 1, axis=0)
    t /= np.hypot(*t.T)[:, None]
    d = pts - committed.xy[i]
    return float(np.mean(t[i, 0] * d[:, 1] - t[i, 1] * d[:, 0] > 0))


@pytest.fixture(scope="module")
def spielberg_maps():
    from raceline.extract import load_map

    committed = _committed()
    gym, survey = load_map(GYM_MAP), load_map(SURVEY_MAP)
    out = {
        "committed": committed,
        "gym": gym,
        "survey": survey,
        "gym_edges": _png_edges(gym),
    }
    for walls in ("observed", "non_free"):
        out["gym_result", walls] = _extract_spielberg(gym, committed, walls)
        out["survey_result", walls] = _extract_spielberg(
            survey, committed, walls
        )
    return out


def _common_checks(maps, grid, result):
    committed = maps["committed"]
    track, right, left, yaw = result
    # the extractor starts where the committed track does, heading its way
    assert np.hypot(*(track.xy[0] - committed.xy[0])) <= 0.2
    seg = track.xy[1] - track.xy[0]
    err = np.arctan2(seg[1], seg[0]) - yaw
    assert abs(np.arctan2(np.sin(err), np.cos(err))) <= np.radians(30.0)
    # left and right are the committed track's left and right
    assert _side_fraction(left, committed) >= 0.99
    assert _side_fraction(right, committed) <= 0.01
    # the length is within LENGTH_RTOL of the committed track's, either way
    ratio = _polyline_length(track.xy) / _polyline_length(committed.xy)
    assert 1.0 - LENGTH_RTOL <= ratio <= 1.0 + LENGTH_RTOL
    assert abs(np.median(track.w_left) - 1.1) <= WIDTH_MEDIAN_TOL
    assert abs(np.median(track.w_right) - 1.1) <= WIDTH_MEDIAN_TOL
    # curvature is computed from the path, so it is finite and bounded
    assert np.isfinite(track.curvature).all()
    assert np.abs(track.curvature).max() < 1.0 / 0.5  # tighter than the car


@pytest.mark.parametrize("walls", ["observed", "non_free"])
def test_raceline_1110_gym_png_walls_match_its_own_edges(spielberg_maps, walls):
    """RACELINE-1110: extracted bounds vs the gym PNG's free-cell wall edges,
    under both wall selectors (the gym map has no frame leak: same figures)."""
    maps = spielberg_maps
    grid = maps["gym"]
    result = maps["gym_result", walls]
    _common_checks(maps, grid, result)
    track, right, left, _ = result
    for bound in (left, right):
        to_edge, to_bound, outside = _edge_errors(
            grid, bound, maps["gym_edges"]
        )
        assert to_edge <= GYM_EDGE_TOL
        assert to_bound <= GYM_EDGE_TOL
        assert outside == 0.0
    mean_bound = np.mean(
        [_polyline_length(_resample(b)) for b in (left, right)]
    )
    mean_edge = np.mean(
        [_polyline_length(_resample(e)) for e in maps["gym_edges"]]
    )
    assert abs(mean_bound / mean_edge - 1.0) <= BOUND_LEN_RTOL


# Part 2.3, walls="observed" on the survey. Measured on the branch (padding the
# frame with unknown; the corner pocket no longer fills), against the
# non_free figures above, which stay as they were:
#   largest total width 2.502 (non_free 3.158)         -> 2.55
#   left bound on the frame 3.28 % (non_free 6.03 %)   -> 0.04
#   edge -> bound coverage within 0.15 m 100 % (non_free 97.85 %) -> 0.995
#   edge -> bound max 0.091 (non_free 0.987, asserted for the first time)
#                                              -> SLAM_EDGE_TOL 0.15 (kept)
#   lap p95 0.047 (0.053) -> 0.06; corner p95 0.063 (0.413) -> 0.08;
#   lateral max 0.651 (0.705) -> 0.70
# The margin is about one cell, tighter than the survey's three-cell convention
# above: the leak that needed three cells is gone.
OBSERVED_SURVEY = {
    "total_width": 2.55,
    "frame_share": 0.04,
    "coverage": 0.995,
    "lap_p95": 0.06,
    "corner_p95": 0.08,
    "lateral_max": 0.70,
}


@pytest.mark.parametrize("walls", ["observed", "non_free"])
def test_raceline_1120_survey_walls_match_the_gym_walls(spielberg_maps, walls):
    """RACELINE-1120: the same on the committed slam survey (acceptance 2)."""
    maps = spielberg_maps
    grid = maps["survey"]
    result = maps["survey_result", walls]
    _common_checks(maps, grid, result)
    track, right, left, _ = result
    obs = OBSERVED_SURVEY
    for bound in (left, right):
        to_edge, to_bound, outside = _edge_errors(
            grid, bound, maps["gym_edges"]
        )
        assert to_edge <= SLAM_EDGE_TOL
        if walls == "observed":
            assert to_bound <= SLAM_EDGE_TOL  # the leak is fixed
            assert outside <= obs["frame_share"]
            assert (
                _coverage(grid, bound, maps["gym_edges"], SLAM_EDGE_TOL)
                >= obs["coverage"]
            )
        else:
            assert outside <= SLAM_MAX_FRAME_SHARE
            assert (
                _coverage(grid, bound, maps["gym_edges"], SLAM_EDGE_TOL)
                >= SLAM_MIN_COVERAGE
            )
    # the leak's size, in the gate: the committed track is 2.2 m wide
    ceiling = (
        obs["total_width"] if walls == "observed" else SLAM_MAX_TOTAL_WIDTH
    )
    assert (track.w_left + track.w_right).max() <= ceiling


# Lateral distance of the extracted centerline from the committed one, in the
# corners (committed |kappa| >= CORNER_KAPPA, 1/10 m; 500 of 3386 points) and
# over the lap. The medial axis cuts every corner; a2c6f0b measured:
#   gym:    lap p95 0.272  corner p95 0.649  max 1.025  length 0.9863
#   survey: lap p95 0.273  corner p95 0.649  max 0.817  length 0.9882
# Re-centring the corners on the outer bound measured (sigma 0.2, final 0.1):
#   gym:    lap p95 0.081  corner p95 0.082  max 0.622  length 0.9986
#   survey: lap p95 0.053  corner p95 0.413  max 0.705  length 1.0017
# The max is the 0.165 m hairpin at (-75.4, 52.3), whose drawn centerline no
# bound reveals; the survey's corner p95 is its top-left leak (45 points).
# 2026-10-14, after part 2.1 redrew the gym PNG in the ROS cell-centre
# convention (SIM-1030; the walls moved by up to 0.078 m), the gym case
# measures: lap p95 0.016  corner p95 0.022  max 0.709  length 0.9981. The
# survey case is unchanged. The owner's ruling re-derived the gym ceilings:
# lap p95 0.10 -> 0.02 and corner p95 0.10 -> 0.03 (the measurement plus a
# margin), and the lateral max 0.70 -> 0.75, the one widening: 0.709 m is the
# hairpin near (-75.4, 52.3), which the corrected walls moved by +0.087 m
# (0.622 m before).
CORNER_KAPPA = 0.1
LAP_P95_TOL = {"gym": 0.02, "survey": 0.07}
CORNER_P95_TOL = {"gym": 0.03, "survey": 0.50}
LATERAL_MAX_TOL = {"gym": 0.75, "survey": 0.80}


def _lateral_to_committed(committed, xy):
    from scipy.spatial import cKDTree

    d, _ = cKDTree(_resample(committed.xy, 0.02)).query(xy)
    _, j = cKDTree(committed.xy).query(xy)
    corner = np.abs(committed.curvature)[j] >= CORNER_KAPPA
    return d, corner


@pytest.mark.parametrize("walls", ["observed", "non_free"])
@pytest.mark.parametrize("which", ["gym", "survey"])
def test_raceline_1130_corners_follow_the_committed_centerline(
    spielberg_maps, which, walls
):
    """RACELINE-1130: the corner clean-up. The medial axis cuts the corners;
    the lateral error over them, and the length, must stay with the track."""
    maps = spielberg_maps
    track = maps[which + "_result", walls][0]
    d, corner = _lateral_to_committed(maps["committed"], track.xy)
    assert corner.sum() > 100  # the stretch under test exists
    if which == "survey" and walls == "observed":
        obs = OBSERVED_SURVEY
        assert np.percentile(d, 95) <= obs["lap_p95"]
        assert np.percentile(d[corner], 95) <= obs["corner_p95"]
        assert d.max() <= obs["lateral_max"]
        return
    assert np.percentile(d, 95) <= LAP_P95_TOL[which]
    assert np.percentile(d[corner], 95) <= CORNER_P95_TOL[which]
    assert d.max() <= LATERAL_MAX_TOL[which]


# ---- walls="observed" (part 2.3) -------------------------------------------


def test_raceline_1140_a_wall_next_to_the_frame_is_not_filled_in():
    """RACELINE-1140: an outer wall one cell inside two adjacent frame edges
    (the survey's corner pocket). non_free fills the pocket the frame cuts off
    and takes the frame for the bound; observed does not."""
    grid = _ring(xmax=6.10, ymax=6.10)
    _assert_annulus(_extract(grid, _ring_start(), walls="observed"))
    leaked = _extract(grid, _ring_start(), walls="non_free")
    assert (leaked.w_left + leaked.w_right).max() > 3.0  # 4.008 measured


def _widths_of(track):
    return track.w_left, track.w_right


_GAP_CASES = {
    "outer_fan_6_8": dict(gap=32),
    "outer_fan_to_frame": dict(gap=32, xmax=6.6, ymax=6.6, fan_out=3.0),
    "inner_50": dict(gap=50, gap_wall="inner"),
    # the worst admitted runs of the C1 sweep, one per tolerance
    "worst_narrow": dict(
        r_in=5.2, r_out=7.2, half=8.2, gap=24, gap_wall="both", c_deg=-60
    ),
    "worst_radius": dict(r_in=5.4, r_out=7.4, half=8.4, gap=26, c_deg=0),
    "worst_wide": dict(
        r_in=5.4, r_out=7.4, half=8.4, gap=38, gap_wall="inner", c_deg=110
    ),
}


@pytest.mark.parametrize("name", list(_GAP_CASES))
def test_raceline_1150_an_open_wall_is_bridged_not_taken_as_track(name):
    """RACELINE-1150: a gap the lidar saw through. observed bridges it from the
    walls at its ends: nowhere wider than the annulus (WIDTH_TOL), narrower by
    at most BRIDGE_WIDTH_TOL, the centreline within BRIDGE_RADIUS_TOL. non_free
    takes the free fan for track (total width 2.797 and 3.360 measured)."""
    from raceline.extract import observed_mask, track_bounds

    kw = _GAP_CASES[name]
    r_c = kw.get("r_in", 4.0) + 1.0
    grid = _ring(**kw)
    start = _ring_start(kw.get("r_in", 4.0))
    track = _extract(grid, start, walls="observed")
    assert (track.w_left + track.w_right).max() <= 2.0 + WIDTH_TOL
    for w in (track.w_left, track.w_right):
        assert w.max() <= 1.0 + WIDTH_TOL
        assert w.min() >= 1.0 - BRIDGE_WIDTH_TOL
    r = np.hypot(track.xy[:, 0], track.xy[:, 1])
    assert np.abs(r - r_c).max() <= BRIDGE_RADIUS_TOL
    # the bridge is drawn, not seen: some bound points are unobserved
    right, left = track_bounds(grid, track.xy, walls="observed")
    unobserved = sum(
        int((~observed_mask(grid, b)).sum()) for b in (right, left)
    )
    assert unobserved > 0
    if name.startswith("outer_fan"):
        loose = _extract(grid, start, walls="non_free")
        assert (loose.w_left + loose.w_right).max() > 2.5


def test_raceline_1150_a_wall_beyond_the_frame_stays_narrow():
    """RACELINE-1150 (wall_beyond_frame): the outer wall lies off the map over
    about 30 deg (chord 3.63 m, under MAX_BRIDGE_M). The frame stays the bound
    and the track errs narrow (0.815 against truth 1.0), never wide;
    observed_mask reports those points. Past the cap the same map raises
    (RACELINE-1155 wall_beyond_frame_past_the_cap)."""
    from raceline.extract import observed_mask, track_bounds

    grid = _ring(xmax=5.8)
    track = _extract(grid, _ring_start(), walls="observed")
    assert track.w_left.max() <= 1.0 + WIDTH_TOL
    assert track.w_right.max() <= 1.0 + WIDTH_TOL
    assert min(track.w_left.min(), track.w_right.min()) <= 0.85  # 0.815
    right, left = track_bounds(grid, track.xy, walls="observed")
    assert sum(int((~observed_mask(grid, b)).sum()) for b in (right, left)) > 0


def _no_wall_ring():
    grid = _ring()
    data = np.where(grid.data == 100, -1, grid.data)
    return _grid(data, origin=(grid.origin.x, grid.origin.y))


@pytest.mark.parametrize(
    "name,make,r_in",
    [
        # R_in 3, inner 70 deg: chord 3.80 m (under the cap) but a 90 deg
        # end-tangent turn > MAX_BRIDGE_TURN_DEG
        (
            "turn_past_the_cap",
            lambda: _ring(
                r_in=3.0, r_out=5.0, half=6.0, gap=70, gap_wall="inner"
            ),
            3.0,
        ),
        # R_in 8, inner 60 deg: turn 66.3 deg but a 8.37 m chord > MAX_BRIDGE_M
        (
            "chord_past_the_cap",
            lambda: _ring(
                r_in=8.0, r_out=10.0, half=11.0, gap=60, gap_wall="inner"
            ),
            8.0,
        ),
        # the outer 40 deg gap: chord 4.55 m, turn 44 deg
        ("chord_just_past_the_cap", lambda: _ring(gap=40), 4.0),
        # the outer 120 deg gap (chord 9.45 m, turn 109 deg)
        ("outer_120_deg", lambda: _ring(gap=120), 4.0),
        # the outer wall off the map over ~42 deg: chord 4.73 m > MAX_BRIDGE_M
        ("wall_beyond_frame_past_the_cap", lambda: _ring(xmax=5.6), 4.0),
        ("no_observed_wall_at_all", _no_wall_ring, 4.0),
    ],
)
def test_raceline_1155_a_gap_past_the_cap_raises_unobserved(name, make, r_in):
    """RACELINE-1155: past the cap the extractor stops with a named error
    instead of returning a plausible wrong track."""
    with pytest.raises(ValueError, match="unobserved"):
        _extract(make(), _ring_start(r_in), walls="observed")


@pytest.mark.parametrize("walls", ["observed", "non_free"])
def test_raceline_1160_track_bounds_are_the_walls_extract_track_measured(
    walls,
):
    """RACELINE-1160: track_bounds(grid, track.xy) returns the bounds the
    widths were measured to, bridges included (part 4's verifier relies
    on it)."""
    from raceline.extract import _widths, track_bounds

    grid = _ring(gap=32)
    track = _extract(grid, _ring_start(), walls=walls)
    right, left = track_bounds(grid, track.xy, walls=walls)
    assert np.abs(_widths(track.xy, left) - track.w_left).max() <= 1e-9
    assert np.abs(_widths(track.xy, right) - track.w_right).max() <= 1e-9


def test_raceline_1170_the_survey_bounds_are_seen_walls(spielberg_maps):
    """RACELINE-1170: the map-only check, no ground truth. Every point of the
    observed bounds has an occupied cell within OBSERVED_TOL; non_free's left
    bound runs 1.96 % (74 of 3769 points) along the frame."""
    from raceline.extract import observed_mask

    grid = spielberg_maps["survey"]
    for walls, left_share in (("observed", 0.0), ("non_free", 0.015)):
        _, right, left, _ = spielberg_maps["survey_result", walls]
        share_l = float((~observed_mask(grid, _resample(left, 0.1))).mean())
        share_r = float((~observed_mask(grid, _resample(right, 0.1))).mean())
        assert share_r == 0.0
        if walls == "observed":
            assert share_l == 0.0
        else:
            assert share_l >= left_share


# RACELINE-1510: MOLA's own grid (spec C2, acceptance 2). Measured, not
# chosen. extract_track(walls="observed") closed on every grid of the final
# mapping pipeline (icp_prior_weight 50; 2026-10-30, slam-p2, load 4-8);
# length / committed track length:
#   seed 7, tuning laps   1.0081  1.0088
#   seed 42, tuning laps  1.0079  1.0078
#   seed 7, script laps   1.0090  1.0083  1.0085
#   seed 7, committed     1.0079
# worst |ratio - 1| 0.0090 x 1.25 = 0.01125, rounded up to 0.001
#                                                 -> MOLA_LENGTH_RTOL 0.012
# The 1 % target is met (0.0090); only the guard is above it (owner ruling
# 2026-10-30, T1). The baseline pipeline's five seed-42 laps gave 1.0019-
# 1.0041 (0.006); icp_prior_weight 50 trades that length for closing seed 7.
MOLA_GRID = REPO / "config/scenarios/maps/Spielberg/mola/Spielberg_grid.yaml"
MOLA_LENGTH_RTOL = 0.012


def test_raceline_1510_the_extractor_reads_the_mola_grid_unchanged():
    """RACELINE-1510: `load_map` reads MOLA's grid, it has observed walls,
    and `extract_track(..., walls="observed")` returns a closed track whose
    length is within `MOLA_LENGTH_RTOL` of the committed track's. No load-only
    fallback: a grid the extractor cannot close misses acceptance 2."""
    from raceline.extract import load_map, observed_mask

    assert MOLA_LENGTH_RTOL is not None, "MOLA_LENGTH_RTOL is not measured"
    grid = load_map(MOLA_GRID)
    committed = _committed()
    track, right, left, _ = _extract_spielberg(grid, committed, "observed")
    assert len(track.xy) > 100
    assert abs(_area(track.xy)) > 100.0  # a closed loop encloses area
    assert observed_mask(grid, left).any() and observed_mask(grid, right).any()
    ratio = _polyline_length(track.xy) / _polyline_length(committed.xy)
    assert 1.0 - MOLA_LENGTH_RTOL <= ratio <= 1.0 + MOLA_LENGTH_RTOL, ratio
