"""Upstream extraction baseline: a measurement, never run by the gate.

Runs UNMODIFIED ForzaETH/race_stack extraction (``extract_centerline``,
``smooth_centerline``, ``extract_track_bounds`` from
``planner/global_planner/global_planner/global_planner_utils.py``) on the
Spielberg gym PNG and the committed slam survey, and prints it beside
``raceline.extract`` (part 2) with the metrics of RACELINE-1110 to 1130.

Run (needs the network; the gate never runs this, pytest does not collect it)::

    docker exec gr-roboracer-dev bash -lc 'cd /ws && env -u PYTHONPATH \\
        uv run --project raceline --locked python \\
        raceline/bench/upstream_baseline.py'

Upstream is pinned at ``UPSTREAM_SHA``; the file is fetched at run time into a
temp directory, its sha256 checked, and never vendored or edited.

Stand-ins (``ament_index_python``, ``matplotlib``,
``trajectory_planning_helpers``, ``global_racetrajectory_optimization``,
``geometry_msgs``, ``f110_msgs``,
``visualization_msgs``) satisfy upstream's module-level imports only. Every call
or attribute access on one raises ``StandInCalled``, so a stand-in that is used
fails loudly. ``Point`` is a plain dataclass.

Input preparation transcribes upstream ``GlobalPlannerLogic`` (MIT, see
NOTICE.md): ``filter_map_occupancy_grid`` (logic:516-528) and
``compute_global_trajectory`` (:250-253). The skeleton from skimage 0.26 is
bool; it is cast to uint8 0/255, which ``cv2.findContours`` treats the same as
0/1 (argument preparation, not an upstream edit).

``cent_length`` is the committed Spielberg centerline length. That FAVOURS
upstream: it gets the true lap length, which a real mapping run only
approximates from odometry, while part 2 gets only ``start_pose``.

Not compared for upstream: width medians (upstream's ``dist_to_bounds`` needs
TUM's LGPL ``interp_track``), start-pose and heading checks (upstream orients
later, in ``compute_global_trajectory``) and curvature (upstream computes none
here). The columns are left out for every row. Upstream's bounds use the cell
corner (``px * res + origin``, no half cell), so up to 0.025 m per axis of its
bound error is that convention.

Depends on ``raceline/tests/test_extract.py``'s private helpers (``_png_edges``,
``_edge_errors``, ``_coverage``, ``_side_fraction``, ``_lateral_to_committed``,
``_polyline_length``, ``_extract_spielberg``): a rename there breaks it.
"""

import hashlib
import importlib.util
import sys
import tempfile
import time
import types
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import scipy.signal  # noqa: F401  (upstream imports these; load before stubs)
import skimage.segmentation  # noqa: F401
from skimage.morphology import skeletonize

UPSTREAM_SHA = "202450f51081b618fcca71924ad6d33f44f90e44"
UPSTREAM_PATH = "planner/global_planner/global_planner/global_planner_utils.py"
UPSTREAM_URL = (
    "https://raw.githubusercontent.com/ForzaETH/race_stack/"
    f"{UPSTREAM_SHA}/{UPSTREAM_PATH}"
)
UPSTREAM_SHA256 = (
    "0adbffb30f7134268a52f3dc47c83149ad69347b305ff19c960fb91da3580695"
)
# stack_master/config/global_planner/global_planner_params.yaml:7-8 @202450f:
# occupancy_grid_threshold 10, filter_kernel_size 9. The class default kernel
# is 3 (logic:54), which part 2 ported. On a 0/100 grid any threshold in 1..100
# gives the same mask.
OCCUPANCY_THRESHOLD = 10
KERNELS = (9, 3)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
import test_extract as te  # noqa: E402


class StandInCalled(RuntimeError):
    """A stand-in module was used by upstream code."""


class _StandIn:
    def __init__(self, *args, **kwargs):
        raise StandInCalled(type(self).__name__)


def _stand_in_class(name):
    return type(name, (_StandIn,), {})


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class _StandInModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        raise StandInCalled(f"{self.__name__}.{name}")


def _install_stand_ins():
    def mod(name, **names):
        m = _StandInModule(name)
        m.__dict__.update(names)
        sys.modules[name] = m
        return m

    def fn(name):
        def f(*args, **kwargs):
            raise StandInCalled(name)

        return f

    cls = _stand_in_class
    ament = mod("ament_index_python")
    ament.packages = mod(
        "ament_index_python.packages",
        get_package_share_directory=fn("get_package_share_directory"),
    )
    mpl = mod("matplotlib")
    mpl.pyplot = mod("matplotlib.pyplot")
    mod("trajectory_planning_helpers")
    mod("global_racetrajectory_optimization", helper_funcs_glob=cls("hfg"))
    geo = mod("geometry_msgs")
    geo.msg = mod("geometry_msgs.msg", Point=Point)
    f110 = mod("f110_msgs")
    f110.msg = mod(
        "f110_msgs.msg", Wpnt=cls("Wpnt"), WpntArray=cls("WpntArray")
    )
    vis = mod("visualization_msgs")
    vis.msg = mod(
        "visualization_msgs.msg",
        Marker=cls("Marker"),
        MarkerArray=cls("MarkerArray"),
    )


def _load_upstream(tmp):
    raw = urllib.request.urlopen(UPSTREAM_URL, timeout=60).read()
    got = hashlib.sha256(raw).hexdigest()
    if got != UPSTREAM_SHA256:
        raise RuntimeError(f"upstream sha256 {got} != {UPSTREAM_SHA256}")
    path = Path(tmp) / "global_planner_utils.py"
    path.write_bytes(raw)
    _install_stand_ins()
    spec = importlib.util.spec_from_file_location("upstream_utils", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _prepare(grid, kernel):
    """Upstream logic:516-528 and :250-253 (see module docstring)."""
    m = np.where(grid.data == -1, 100, grid.data)
    bw = np.where(m < OCCUPANCY_THRESHOLD, 255, 0).astype(np.uint8)
    bw = cv2.morphologyEx(
        bw, cv2.MORPH_OPEN, np.ones((kernel, kernel), np.uint8), iterations=2
    )
    skel = (skeletonize(bw, method="lee") > 0).astype(np.uint8) * 255
    return bw, skel


def _upstream(up, grid, committed, kernel):
    """Returns (centerline_m or None, bounds or exception, seconds)."""
    p0, p1 = committed.xy[0], committed.xy[1]
    yaw = float(np.arctan2(p1[1] - p0[1], p1[0] - p0[0]))
    res, o = grid.resolution, grid.origin
    cent_length = te._polyline_length(committed.xy)
    t0 = time.perf_counter()
    bw, skel = _prepare(grid, kernel)
    try:
        cl = up.extract_centerline(skel, cent_length, res, False)
        smooth = up.smooth_centerline(cl)
    except Exception as e:  # upstream failing is a result
        return None, e, time.perf_counter() - t0
    centerline = smooth * res + np.array([o.x, o.y])
    try:
        right, left = up.extract_track_bounds(
            smooth, bw, False, res, Point(o.x, o.y), [p0[0], p0[1], yaw], False
        )
    except Exception as e:
        return centerline, e, time.perf_counter() - t0
    return centerline, (right, left), time.perf_counter() - t0


FAILURES = []


def _fail(e):
    text = f"{type(e).__name__}: {e}"
    if text not in FAILURES:
        FAILURES.append(text)
    return f"failed [{FAILURES.index(text) + 1}]"


def _row(label, which, grid, edges, committed, xy, bounds, secs):
    """Cells: lap p95, corner p95, lateral max, length ratio, bound->edge max,
    edge->bound max (gym) or coverage (survey), frame share, sides, time."""
    cells = {}
    if xy is None:
        cells = {k: _fail(bounds) for k in range(9)}
    else:
        d, corner = te._lateral_to_committed(committed, xy)
        cells[0] = f"{np.percentile(d, 95):.3f}"
        cells[1] = f"{np.percentile(d[corner], 95):.3f}"
        cells[2] = f"{d.max():.3f}"
        cells[3] = (
            f"{te._polyline_length(xy) / te._polyline_length(committed.xy):.4f}"
        )
        if isinstance(bounds, Exception):
            for k in range(4, 9):
                cells[k] = _fail(bounds)
        else:
            right, left = bounds
            e = [te._edge_errors(grid, b, edges) for b in (left, right)]
            cells[4] = " / ".join(f"{x[0]:.3f}" for x in e)
            if which == "gym":
                cells[5] = " / ".join(f"{x[1]:.3f}" for x in e)
            else:
                cov = [
                    te._coverage(grid, b, edges, te.SLAM_EDGE_TOL)
                    for b in (left, right)
                ]
                cells[5] = " / ".join(f"{c:.3f}" for c in cov)
            cells[6] = " / ".join(f"{x[2]:.3f}" for x in e)
            cells[7] = (
                f"{te._side_fraction(left, committed):.3f} / "
                f"{te._side_fraction(right, committed):.3f}"
            )
    cells[8] = f"{secs:.2f}"
    return (
        f"| {which} | {label} | "
        + " | ".join(cells[k] for k in range(9))
        + " |"
    )


def main():
    from raceline.extract import load_map

    committed = te._committed()
    grids = {"gym": load_map(te.GYM_MAP), "survey": load_map(te.SURVEY_MAP)}
    edges = te._png_edges(grids["gym"])
    print(
        "| map | extractor | lap p95 | corner p95 | lateral max | length "
        "ratio | bound->edge max (L / R) | edge->bound max (gym) or "
        "coverage within 0.15 m (survey) (L / R) | bound on frame (L / R) | "
        "side share left (L / R) | s |"
    )
    print("|" + "---|" * 11)
    with tempfile.TemporaryDirectory() as tmp:
        up = _load_upstream(tmp)
        for which, grid in grids.items():
            t0 = time.perf_counter()
            track, right, left, _ = te._extract_spielberg(grid, committed)
            secs = time.perf_counter() - t0
            print(
                _row(
                    "part 2",
                    which,
                    grid,
                    edges,
                    committed,
                    track.xy,
                    (right, left),
                    secs,
                )
            )
            for k in KERNELS:
                xy, res, secs = _upstream(up, grid, committed, k)
                print(
                    _row(
                        f"upstream @{UPSTREAM_SHA[:7]} K={k}",
                        which,
                        grid,
                        edges,
                        committed,
                        xy,
                        res,
                        secs,
                    )
                )
    for i, e in enumerate(FAILURES, 1):
        print(f"\n[{i}] upstream raised {e}")


if __name__ == "__main__":
    main()
