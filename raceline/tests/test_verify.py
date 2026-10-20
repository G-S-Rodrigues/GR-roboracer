"""Tier 1: the geometric verifier (RACELINE-1310..1365).

Pure numpy: an analytic annulus, circles of other radii and a few corner
cases. No map and no TUM.
"""

import dataclasses
from pathlib import Path

import cv2
import numpy as np
import pytest
from raceline.optimize import Raceline
from raceline.track import Track
from raceline.vehicle import load_vehicle
from test_extract import SLAM_EDGE_TOL

from raceline import geometry, verify

VEHICLES = Path(__file__).resolve().parents[2] / "config" / "vehicles"
R_TRACK = 5.0
WIDTH = 1.0
STEP = 0.1
MARGIN = 0.30


def _vehicle():
    return load_vehicle(VEHICLES / "f1tenth_gym_limits.yaml")


def _annulus(ccw=True) -> Track:
    n = int(round(2 * np.pi * R_TRACK / STEP))
    th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    sign = 1.0 if ccw else -1.0
    xy = R_TRACK * np.column_stack([np.cos(th), sign * np.sin(th)])
    kappa = np.full(n, sign / R_TRACK)
    w = np.full(n, WIDTH)
    return Track("annulus", xy, kappa, w, w.copy())


def _line_from_xy(xy, v=4.0) -> Raceline:
    """psi (ROS yaw) and kappa from xy with the verifier's own formulas."""
    n = len(xy)
    d = np.roll(xy, -1, 0) - np.roll(xy, 1, 0)
    psi = np.arctan2(d[:, 1], d[:, 0])
    a, b, c = np.roll(xy, 1, 0), xy, np.roll(xy, -1, 0)
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - b[:, 1]) - (b[:, 1] - a[:, 1]) * (
        c[:, 0] - b[:, 0]
    )
    kappa = (
        2.0
        * cross
        / (np.hypot(*(b - a).T) * np.hypot(*(c - b).T) * np.hypot(*(c - a).T))
    )
    seg = np.hypot(*(np.roll(xy, -1, 0) - xy).T)
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    vv = np.full(n, float(v))
    return Raceline(s, xy[:, 0], xy[:, 1], psi, kappa, vv, np.zeros(n))


def _circle(r, ccw=True, v=4.0) -> Raceline:
    n = int(round(2 * np.pi * r / STEP))
    th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    sign = 1.0 if ccw else -1.0
    xy = r * np.column_stack([np.cos(th), sign * np.sin(th)])
    return _line_from_xy(xy, v)


def _pairs(track, vehicle=None, names=("input",)):
    vehicle = vehicle or _vehicle()
    req = verify.required_clearances(vehicle, MARGIN)
    return [(verify.track_walls(track, nm), req[nm]) for nm in names]


def _verdict(line, vehicle=None, track=None, names=("input",)):
    vehicle = vehicle or _vehicle()
    track = track or _annulus()
    return verify.verify(line, vehicle, _pairs(track, vehicle, names), track)


def _failures(verdict, word):
    return [f for f in verdict.failures if word in f]


def test_raceline_1310_a_feasible_circle_passes():
    verdict = _verdict(_circle(5.2))
    assert verdict.failures == ()
    # Raw signed distance to the nearer (outer, r = 6) wall.
    assert verdict.min_clearance["input"] == pytest.approx(0.8, abs=0.01)
    assert verdict.max_abs_kappa == pytest.approx(1 / 5.2, abs=1e-3)


def test_raceline_1315_a_line_outside_the_outer_wall_fails():
    verdict = _verdict(_circle(6.3))
    assert not verdict.ok
    assert any("outside" in f for f in verdict.failures)
    assert verdict.min_clearance["input"] < 0


def test_raceline_1320_inside_but_inside_the_margin_fails_on_clearance():
    verdict = _verdict(_circle(5.7))
    assert verdict.min_clearance["input"] == pytest.approx(0.3, abs=0.01)
    assert _failures(verdict, "clearance")
    assert not _failures(verdict, "outside")


def test_raceline_1325_kappa_over_the_vehicle_limit_fails_only_kappa():
    vehicle = dataclasses.replace(_vehicle(), kappa_max=0.1)
    verdict = _verdict(_circle(5.2), vehicle)
    assert len(verdict.failures) == 1
    assert verdict.failures[0].startswith("kappa")


def test_raceline_1330_lateral_acceleration_over_the_ggv_fails():
    verdict = _verdict(_circle(5.2, v=8.0))  # 64 / 5.2 = 12.3 > 10.29
    assert len(verdict.failures) == 1
    assert verdict.failures[0].startswith("a_y")
    assert verdict.max_ay_ratio > 1.0


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_raceline_1335_longitudinal_acceleration_over_the_limit_fails(sign):
    line = _circle(5.2, v=4.0)
    k = 100
    v = line.v.copy()
    a = np.zeros_like(v)
    # v steps 4 -> 6 over 0.1 m (a = 100 m/s^2), or the reverse braking.
    lo, hi = (4.0, 6.0) if sign > 0 else (6.0, 4.0)
    v[:k], v[k:] = lo, hi
    a[k - 1] = sign * (hi**2 - lo**2) / (2 * 0.1)
    line = dataclasses.replace(line, v=v, a=a)
    verdict = _verdict(line)
    assert _failures(verdict, "a_x"), verdict.failures
    assert verdict.max_ax_ratio > 1.0


def _broken(kind):
    line = _circle(5.2)
    if kind == "repeated closing point":
        cols = [np.append(c, c[0]) for c in dataclasses.astuple(line)]
        cols[0][-1] = line.s[-1] + 0.1
        return Raceline(*cols)
    if kind == "one metre gap":
        cut = 10
        return Raceline(*(c[:-cut] for c in dataclasses.astuple(line)))
    s = line.s.copy()
    s[50:] -= 0.1  # a non-increasing step at 49 -> 50
    return dataclasses.replace(line, s=s)


@pytest.mark.parametrize(
    "kind", ["repeated closing point", "one metre gap", "non-monotonic s"]
)
def test_raceline_1340_a_broken_closure_fails(kind):
    verdict = _verdict(_broken(kind))
    assert _failures(verdict, "closure"), verdict.failures


def test_raceline_1345_tum_psi_convention_fails_with_the_offset():
    line = _circle(5.2)
    verdict = _verdict(dataclasses.replace(line, psi=line.psi - np.pi / 2))
    bad = _failures(verdict, "psi")
    assert bad and "pi/2" in bad[0]


def test_raceline_1350_the_reversed_direction_fails():
    verdict = _verdict(_circle(5.2, ccw=False))
    assert len(verdict.failures) == 1
    assert verdict.failures[0].startswith("direction")


def _leaky_walls(track, leak=0.96, span=np.pi * 120 / 180):
    """The annulus with the outer (right) wall pushed out over an arc."""
    plain = verify.track_walls(track, "input")
    th = np.arctan2(plain.right[:, 1], plain.right[:, 0])
    th = np.angle(np.exp(1j * (th - np.pi)))  # leak centred on theta = pi
    r = np.hypot(*plain.right.T) + leak * (np.abs(th) < span / 2)
    right = (r / np.hypot(*plain.right.T))[:, None] * plain.right
    return dataclasses.replace(plain, right=right)


def _bulged(peak=0.8, span=np.pi * 90 / 180):
    n = int(round(2 * np.pi * 5.2 / STEP))
    th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    d = np.angle(np.exp(1j * (th - np.pi)))
    bump = np.where(
        np.abs(d) < span / 2, 0.5 * (1 + np.cos(2 * np.pi * d / span)), 0.0
    )
    r = 5.2 + peak * bump
    xy = r[:, None] * np.column_stack([np.cos(th), np.sin(th)])
    return _line_from_xy(xy)


def test_raceline_1355_a_leak_passes_the_input_walls_but_not_ground_truth():
    track, vehicle = _annulus(), _vehicle()
    req = verify.required_clearances(vehicle, MARGIN)
    pairs = [
        (_leaky_walls(track), req["input"]),
        (verify.track_walls(track, "ground_truth"), req["ground_truth"]),
    ]
    verdict = verify.verify(_bulged(), vehicle, pairs, track)
    assert len(verdict.failures) == 1, verdict.failures
    assert "ground_truth" in verdict.failures[0]
    assert "clearance" in verdict.failures[0]
    assert verdict.min_clearance["input"] >= req["input"]
    assert verdict.min_clearance["ground_truth"] < req["ground_truth"]


def test_raceline_1357_unobserved_wall_points_are_reported_not_failed():
    track, vehicle = _annulus(), _vehicle()
    plain = verify.track_walls(track, "input")
    th = np.degrees(np.arctan2(plain.right[:, 1], plain.right[:, 0]))
    right_obs = ~((th > 60.0) & (th < 90.0))  # a 30 degree arc
    observed = dataclasses.replace(plain, right_observed=right_obs)
    req = verify.required_clearances(vehicle, MARGIN)
    verdict = verify.verify(
        _circle(5.2), vehicle, [(observed, req["input"])], track
    )
    assert verdict.ok
    assert verdict.min_clearance["input_unobserved"] == pytest.approx(
        0.8, abs=0.01
    )
    n = len(plain.right)
    assert verdict.unobserved_share["input"] == pytest.approx(
        (~right_obs).sum() / (2 * n), abs=1.0 / (2 * n)
    )
    assert abs((~right_obs).sum() - n * 30 / 360) <= 1
    plain_verdict = verify.verify(
        _circle(5.2), vehicle, [(plain, req["input"])], track
    )
    assert plain_verdict.unobserved_share["input"] == 0.0
    assert "input_unobserved" not in plain_verdict.min_clearance


def test_raceline_1360_the_extraction_allowance_is_the_pinned_edge_tolerance():
    assert verify.EXTRACTION_ALLOWANCE == SLAM_EDGE_TOL


def _rounded_square(half=5.0, radius=0.5, w=0.8) -> Track:
    """ccw square, straights 10 m, corner arcs of centerline radius 0.5 m."""
    straight = 2 * (half - radius)
    arc = np.pi * radius / 2
    total = 4 * (straight + arc)
    s = np.arange(0.0, total, STEP)
    pts = []
    for si in s:
        side, u = divmod(si, straight + arc)
        if u < straight:  # straight leg of this side, then its arc
            local = np.array([-half + radius + u, -half])
            on_arc = False
        else:
            local = None
            on_arc = True
        if not on_arc:
            p = local
        else:
            t = (u - straight) / radius
            p = np.array([half - radius, -half + radius]) + radius * np.array(
                [np.sin(t), -np.cos(t)]
            )
        rot = side * np.pi / 2
        c, sn = np.cos(rot), np.sin(rot)
        pts.append((c * p[0] - sn * p[1], sn * p[0] + c * p[1]))
    xy = np.array(pts)
    n = len(xy)
    return Track("square", xy, np.zeros(n), np.full(n, w), np.full(n, w))


def _figure_eight(w=0.2) -> Track:
    th = np.linspace(0.0, 2 * np.pi, 400, endpoint=False)
    d = 1 + np.sin(th) ** 2
    xy = 5.0 * np.column_stack([np.cos(th) / d, np.sin(th) * np.cos(th) / d])
    n = len(xy)
    return Track("eight", xy, np.zeros(n), np.full(n, w), np.full(n, w))


def test_raceline_1365_ground_truth_walls_are_one_clean_polygon():
    track = _rounded_square()
    left_raw, right_raw = verify.offsets(track)  # inside is left (ccw)
    # (a) the fixture folds, else the test is vacuous.
    assert len(geometry.crossings(left_raw)) >= 4
    near = np.min(
        np.hypot(
            left_raw[:, None, 0] - track.xy[None, :, 0],
            left_raw[:, None, 1] - track.xy[None, :, 1],
        ),
        axis=1,
    )
    assert near.min() < 0.8 - 0.1
    walls = verify.track_walls(track, "ground_truth")
    # (b) no crossing left, fewer points.
    assert geometry.crossings(walls.left) == []
    assert len(walls.left) < len(left_raw)
    # (c) every point is a full half-width from the centerline.
    dist = geometry.nearest_distance(walls.left, track.xy, 0.01)
    assert dist.min() >= 0.8 - 0.02
    # (d) the unfolded outside wall is untouched.
    assert np.array_equal(walls.right, right_raw)
    # (e) the centerline's clearance to the clean polygon, not to a loop.
    line = _line_from_xy(track.xy, v=1.0)
    verdict = verify.verify(line, _vehicle(), [(walls, 0.0)], track)
    assert verdict.min_clearance["ground_truth"] >= 0.78
    # (f) a track that crosses itself is refused, not guessed at.
    with pytest.raises(ValueError, match="self-intersect"):
        verify.track_walls(_figure_eight(), "ground_truth")


def _circle_with_curl(extent: float, drop: int):
    """A ccw circle wall (r 5, 0.1 m) with a same-winding loop-the-loop
    spliced over ``drop`` segments; returns (wall, circle, k, apex)."""
    th = np.arange(0.0, 2 * np.pi, 0.1 / 5.0)
    circle = 5.0 * np.column_stack([np.cos(th), np.sin(th)])
    k = 100
    u = circle[k + drop] - circle[k]
    u /= np.linalg.norm(u)
    v = np.array([-u[1], u[0]])
    r = drop * 0.1 / (2 * np.pi)
    b = extent
    phi = np.linspace(-np.pi, np.pi, 41)[1:-1]
    x = r * phi - b * np.sin(phi)
    y = b * (1.0 + np.cos(phi))
    curl = circle[k] + (x + r * np.pi)[:, None] * u + y[:, None] * v
    wall = np.vstack([circle[: k + 1], curl, circle[k + drop :]])
    return wall, circle, k, curl[np.argmax(y)]


def test_raceline_1365g_a_small_same_winding_curl_is_cut_by_size():
    wall, circle, k, apex = _circle_with_curl(0.05, 2)
    assert len(geometry.crossings(wall)) >= 1  # the fixture crosses
    assert np.sign(geometry.signed_area(wall)) > 0
    out = geometry.remove_folds(wall, 1.0)
    assert geometry.crossings(out) == []
    assert not (np.abs(out - apex).max(axis=1) < 1e-12).any()  # apex gone
    keep = np.r_[0 : k + 1, k + 2 : len(circle)]
    for p in circle[keep]:
        assert (np.abs(out - p).max(axis=1) == 0).any()


def test_raceline_1365h_a_curl_larger_than_the_bound_is_not_cut():
    wall, _, _, _ = _circle_with_curl(1.5, 10)
    assert len(geometry.crossings(wall)) >= 1
    with pytest.raises(ValueError, match="self-intersect"):
        geometry.remove_folds(wall, 1.0)


# --- RACELINE-1367: the sharp-tip rule (owner E9 = (c), plan D16) ----------

TIP = 4.2  # the rounded square's clean inner wall has tips at (+-4.2, +-4.2)
CELL = 0.05


def _gt_required():
    return verify.required_clearances(_vehicle(), MARGIN)["ground_truth"]


def _tip_cells(drop=()) -> verify.ObservedCells:
    """A 0.05 m grid aligned on multiples of 0.05; the island is the cells
    whose centre is inside the clean inner wall (so a tip is a cell corner).
    ``drop`` is a list of ((x, y), radius): island cells within it are freed."""
    clean = verify.track_walls(_rounded_square(), "ground_truth").left
    centre = clean.mean(axis=0)
    poly = (clean - centre).astype(np.float32)
    k = np.arange(-int(4.4 / CELL), int(4.4 / CELL))
    xs = (k + 0.5) * CELL
    gx, gy = np.meshgrid(xs, xs)
    pts = np.column_stack([gx.ravel(), gy.ravel()])
    inside = np.array(
        [
            cv2.pointPolygonTest(poly, (float(x), float(y)), False) > 0
            for x, y in pts - centre
        ]
    )
    pts = pts[inside]
    for (cx, cy), rho in drop:
        pts = pts[np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) >= rho]
    return verify.ObservedCells(pts, CELL, "synthetic")


TIPS = [(TIP, TIP), (-TIP, TIP), (-TIP, -TIP), (TIP, -TIP)]


def _tip_line(corner_offset=0.20, replace=None) -> Raceline:
    """Corner points 0.20 off each tip along the outward diagonal, and the
    four straights' midpoints on the centerline. ``replace`` maps a point
    index to a new xy."""
    u = corner_offset / np.sqrt(2.0)
    corners = [(TIP + u, TIP + u), (-TIP - u, TIP + u)]
    corners += [(-TIP - u, -TIP - u), (TIP + u, -TIP - u)]
    pts = [
        (0.0, -5.0),
        corners[3],
        (5.0, 0.0),
        corners[0],
        (0.0, 5.0),
        corners[1],
        (-5.0, 0.0),
        corners[2],
    ]
    for i, xy in (replace or {}).items():
        pts[i] = xy
    return _line_from_xy(np.array(pts), v=1.0)


def _tip_verdict(cells, line=None):
    track = _rounded_square()
    walls = verify.track_walls(track, "ground_truth", cells=cells)
    verdict = verify.verify(
        line or _tip_line(), _vehicle(), [(walls, _gt_required())], track
    )
    return walls, verdict


def _clearance_failures(verdict):
    return _failures(verdict, "clearance ground_truth")


def test_raceline_1367a_the_cut_corners_are_tips():
    track = _rounded_square()
    left_raw, right_raw = verify.offsets(track)
    clean, tips, reach = geometry.remove_folds_cuts(left_raw, 0.8)
    assert np.array_equal(clean, geometry.remove_folds(left_raw, 0.8))
    assert len(tips) == 4
    order = np.lexsort((tips[:, 1], tips[:, 0]))
    expect = np.array(sorted(TIPS))
    assert np.abs(tips[order] - expect).max() < 1e-9
    assert len(reach) == 4
    assert np.all((reach > 0) & (reach <= 0.8))
    _, no_tips, no_reach = geometry.remove_folds_cuts(right_raw, 0.8)
    assert len(no_tips) == 0 and len(no_reach) == 0
    walls = verify.track_walls(track, "ground_truth")
    assert len(walls.tips) == 4 and len(walls.tip_reach) == 4
    assert np.abs(walls.tips[order] - expect).max() < 1e-9


def test_raceline_1367b_polygon_tip_fails_and_observed_cells_pass():
    walls, verdict = _tip_verdict(_tip_cells(drop=[(t, 0.15) for t in TIPS]))
    assert not _clearance_failures(verdict)
    mc = verdict.min_clearance
    assert mc["ground_truth"] == pytest.approx(0.20, abs=1e-4)
    assert mc["ground_truth_tip_cells"] == pytest.approx(0.3239, abs=1e-3)
    assert mc["ground_truth_judged"] == pytest.approx(0.3239, abs=1e-3)
    records = verdict.tips["ground_truth"]
    assert len(records) == 4
    for rec in records:
        assert rec["observed"]
        assert rec["cells_at_tip"] == pytest.approx(0.1414, abs=1e-3)
        assert rec["cells_at_tip"] <= rec["reach"]
        assert rec["governs"] == 1


def test_raceline_1367c_when_the_cells_fail_too_the_line_fails():
    _, verdict = _tip_verdict(_tip_cells(drop=[(t, 0.10) for t in TIPS]))
    failures = _clearance_failures(verdict)
    assert len(failures) == 1
    assert "ground_truth" in failures[0] and "sharp tip" in failures[0]
    assert "0.271" in failures[0] and "0.200" in failures[0]
    judged = verdict.min_clearance["ground_truth_judged"]
    assert judged == pytest.approx(0.2707, abs=1e-3)


def test_raceline_1367d_an_unobserved_tip_gets_no_relief():
    cells = _tip_cells(drop=[(t, 0.45) for t in TIPS])
    walls, verdict = _tip_verdict(cells)
    for t, r in zip(walls.tips, walls.tip_reach, strict=True):
        d = geometry.cell_edge_distance(
            t[None, :], cells.centres, cells.resolution
        )[0]
        assert d > r  # else the case is vacuous
    assert all(not rec["observed"] for rec in verdict.tips["ground_truth"])
    assert "ground_truth_tip_cells" not in verdict.min_clearance
    failures = _clearance_failures(verdict)
    assert len(failures) == 1 and "0.200" in failures[0]


def test_raceline_1367e_off_a_tip_the_polygon_governs():
    cells = _tip_cells(
        drop=[(t, 0.15) for t in TIPS] + [((0.0, -TIP), 0.5)],
    )
    line = _tip_line(replace={0: (0.0, -4.4)})
    _, verdict = _tip_verdict(cells, line)
    failures = _clearance_failures(verdict)
    assert len(failures) == 1
    assert "(0.000, -4.400)" in failures[0]
    assert "sharp tip" not in failures[0]
    for rec in verdict.tips["ground_truth"]:
        assert rec["polygon_min"] == pytest.approx(0.20, abs=1e-4)
        assert rec["cells_min"] == pytest.approx(0.3239, abs=1e-3)


def test_raceline_1367f_without_cells_the_polygon_governs():
    walls, verdict = _tip_verdict(None)
    failures = _clearance_failures(verdict)
    assert len(failures) == 1 and "0.200" in failures[0]
    assert "ground_truth_judged" not in verdict.min_clearance
    records = verdict.tips["ground_truth"]
    assert len(records) == 4
    assert all(rec["cells_at_tip"] is None for rec in records)


def test_raceline_1367g_beyond_the_disc_the_polygon_still_governs():
    top_right = 3  # index of the (+, +) corner point in _tip_line
    cells = _tip_cells(
        drop=[(t, 0.15) for t in TIPS] + [((4.4, 4.0), 0.35)],
    )
    line = _tip_line(replace={top_right: (4.4, 4.0)})
    _, verdict = _tip_verdict(cells, line)
    failures = _clearance_failures(verdict)
    assert len(failures) == 1
    assert "(4.400, 4.000)" in failures[0]
    assert "sharp tip" in failures[0] and "0.209" in failures[0]
    mc = verdict.min_clearance
    assert mc["ground_truth_judged"] == pytest.approx(0.2095, abs=1e-3)
    assert mc["ground_truth_tip_cells"] == pytest.approx(0.3202, abs=1e-3)
