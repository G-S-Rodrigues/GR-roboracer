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
# Measured on the annulus (R 5, widths 1, 0.05 m cells), 2026-10-13: the
# skeleton of a rasterized ring wobbles by about a cell, so the centerline
# radius is off by up to 0.0706 m (mean 0.024), the widths by up to 0.0874 m,
# and kappa * R spans 0.86..1.18 around a mean of 1.0096.
RADIUS_TOL = 0.075
WIDTH_TOL = 0.10
KAPPA_MAX_REL = 0.20
KAPPA_MEAN_REL = 0.02


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


def _extract(grid, start, direction="ccw"):
    from raceline.extract import extract_track

    return extract_track(grid, grid.resolution, grid.origin, start, direction)


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
