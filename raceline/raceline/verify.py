"""The geometric verifier: is a raceline feasible, and inside the walls?

TUM's own checks (``check_traj.py``) only print warnings; this module returns
failures, and the runner refuses to write a line that has any (plan D8).

The checks, each failure naming the check, the wall set and the worst point:

* signed clearance to each wall set (negative = beyond a wall);
* ``|kappa|`` against the vehicle, and the kappa column against the geometry;
* ``v**2 |kappa|`` against the GGV's lateral limit, ``a`` against its
  longitudinal limit and the engine map;
* closure (``s`` starts at 0, increases, matches the path; the closing gap);
* ``psi`` against the path heading (it is ROS yaw, not TUM's 0 = north);
* direction (the same winding and start as the reference track).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.spatial import cKDTree

from raceline.geometry import (
    crossings,
    remove_folds,
    signed_area,
    tangents,
)
from raceline.optimize import Raceline
from raceline.track import Track
from raceline.vehicle import VehicleLimits

# How far the survey-extracted walls may lie outside the committed walls:
# RACELINE-1120's SLAM_EDGE_TOL (pinned equal by RACELINE-1360).
EXTRACTION_ALLOWANCE = 0.15  # m
# TUM's exported spline overshoots an active bound slightly between its QP
# nodes. Measured worst overshoot 0.0009 m (RACELINE-1010); 0.02 m rounds it up.
BOUND_TOL = 0.02  # m
# Tolerances below are the Step 1 re-run's measurements rounded up (gotcha 14).
# STEP1_MEASURED_PLACEHOLDER
KAPPA_TOL = 0.05  # 1/m: kappa column vs the three-point curvature
PSI_TOL = 0.01  # rad: psi vs the central-difference heading
DS_TOL = 1e-3  # m: |ds - |dxy||
LIMIT_TOL = 1e-6  # relative: v**2 |kappa| and a against their table limits
S0_TOL = 1e-6  # m
MIN_POINTS = 3


@dataclass(frozen=True)
class WallSet:
    name: str  # "input" | "ground_truth"
    left: np.ndarray  # (N, 2) closed polyline, left of travel
    right: np.ndarray
    # One bool per wall point: an occupied map cell lies near it. None means
    # every point is observed (a committed track's walls).
    left_observed: np.ndarray | None = None
    right_observed: np.ndarray | None = None


@dataclass(frozen=True)
class Verdict:
    failures: tuple[str, ...]
    min_clearance: dict[str, float] = field(default_factory=dict)
    max_abs_kappa: float = 0.0
    max_ay_ratio: float = 0.0
    max_ax_ratio: float = 0.0
    unobserved_share: dict[str, float] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.failures


def offsets(track: Track) -> tuple[np.ndarray, np.ndarray]:
    """The raw ``xy +- w n`` offsets (left, right); left normal (-ty, tx)."""
    t = tangents(track.xy)
    n = np.column_stack([-t[:, 1], t[:, 0]])
    return (
        track.xy + track.w_left[:, None] * n,
        track.xy - track.w_right[:, None] * n,
    )


def track_walls(track: Track, name: str) -> WallSet:
    """A committed track's walls as two clean, simple polygons (D13).

    Where a width exceeds the inside radius the inner offset folds over
    itself; ``pointPolygonTest`` on a folded polygon measures to the fold's
    loop and reports a false clearance. The fold is removed. A track that
    crosses itself raises ``ValueError``.
    """
    found = crossings(track.xy)
    if found:
        i, j, x = found[0]
        raise ValueError(
            f"track {track.name!r} centerline self-intersects at "
            f"({x[0]:.3f}, {x[1]:.3f}) (segments {i}, {j}); ground-truth "
            "walls for a track that passes over itself are not modelled"
        )
    left, right = offsets(track)
    max_loop = 0.5 * float(np.min(track.w_left + track.w_right))
    return WallSet(
        name, remove_folds(left, max_loop), remove_folds(right, max_loop)
    )


def required_clearances(
    vehicle: VehicleLimits, margin: float
) -> dict[str, float]:
    """The clearance each wall set must give: the one place the allowance is
    applied. The ground-truth walls may lie ``EXTRACTION_ALLOWANCE`` beyond
    the walls the optimizer saw, so they ask for that much less."""
    base = vehicle.half_width + margin - BOUND_TOL
    return {"input": base, "ground_truth": base - EXTRACTION_ALLOWANCE}


def _signed_clearance(walls: WallSet, xy: np.ndarray) -> np.ndarray:
    """Signed distance to the nearer wall; negative beyond either wall."""
    outer, inner = walls.left, walls.right
    if abs(signed_area(inner)) > abs(signed_area(outer)):
        outer, inner = inner, outer
    centre = outer.mean(axis=0)  # float32 polygons: keep the numbers small
    pts = (xy - centre).astype(np.float32)
    out = (outer - centre).astype(np.float32)
    inn = (inner - centre).astype(np.float32)
    c_out = np.array(
        [cv2.pointPolygonTest(out, (float(x), float(y)), True) for x, y in pts]
    )
    c_in = np.array(
        [cv2.pointPolygonTest(inn, (float(x), float(y)), True) for x, y in pts]
    )
    return np.minimum(c_out, -c_in)


def _at(xy: np.ndarray, i: int) -> str:
    return f"idx {i} xy ({xy[i, 0]:.3f}, {xy[i, 1]:.3f})"


def _wrap(a: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a), np.cos(a))


def _kappa_geometry(xy: np.ndarray) -> np.ndarray:
    a, b, c = np.roll(xy, 1, axis=0), xy, np.roll(xy, -1, axis=0)
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - b[:, 1]) - (b[:, 1] - a[:, 1]) * (
        c[:, 0] - b[:, 0]
    )
    den = np.hypot(*(b - a).T) * np.hypot(*(c - b).T) * np.hypot(*(c - a).T)
    return 2.0 * cross / np.maximum(den, 1e-12)


def _closure(line: Raceline, xy: np.ndarray) -> list[str]:
    out = []
    if abs(line.s[0]) > S0_TOL:
        out.append(f"closure: s[0] = {line.s[0]:.6f}, not 0")
    ds = np.diff(line.s)
    if np.any(ds <= 0):
        i = int(np.argmin(ds))
        out.append(f"closure: s not strictly increasing at {_at(xy, i)}")
    dxy = np.hypot(*np.diff(xy, axis=0).T)
    off = np.abs(ds - dxy)
    if off.max() > DS_TOL:
        i = int(np.argmax(off))
        out.append(
            f"closure: |ds - |dxy|| = {off.max():.2e} > {DS_TOL:.0e} at "
            f"{_at(xy, i)}"
        )
    gap = float(np.hypot(*(xy[-1] - xy[0])))
    median = float(np.median(dxy))
    if gap <= 0.0 or gap > 1.5 * median:
        out.append(
            f"closure: closing gap {gap:.3f} m, expected in (0, "
            f"{1.5 * median:.3f}] (1.5 x median step {median:.3f})"
        )
    return out


def _direction(line: Raceline, xy: np.ndarray, ref: Track) -> list[str]:
    reasons = []
    if np.sign(signed_area(xy)) != np.sign(signed_area(ref.xy)):
        reasons.append("winds the other way round the reference")
    reach = float(ref.w_left[0] + ref.w_right[0])
    if np.hypot(*(xy[0] - ref.xy[0])) > reach:
        reasons.append(f"starts more than {reach:.2f} m from the reference")
    heading = np.array([np.cos(line.psi[0]), np.sin(line.psi[0])])
    if heading @ tangents(ref.xy)[0] <= 0.0:
        reasons.append("starts heading against the reference")
    return [f"direction: {', '.join(reasons)}"] if reasons else []


def verify(
    line: Raceline,
    vehicle: VehicleLimits,
    walls: Sequence[tuple[WallSet, float]],
    reference: Track,
) -> Verdict:
    failures: list[str] = []
    xy = line.xy
    failures += _closure(line, xy)
    failures += _direction(line, xy, reference)

    # psi is ROS yaw: it must follow the path.
    heading = np.arctan2(*(np.roll(xy, -1, 0) - np.roll(xy, 1, 0)).T[::-1])
    dpsi = _wrap(line.psi - heading)
    if np.abs(dpsi).max() > PSI_TOL:
        i = int(np.argmax(np.abs(dpsi)))
        note = ""
        if abs(abs(float(np.mean(dpsi))) - np.pi / 2) < 0.1:
            note = " (offset ~pi/2: TUM's psi is 0 = north, ROS yaw is 0 = +x)"
        failures.append(
            f"psi: differs from the path heading by {dpsi[i]:.4f} rad at "
            f"{_at(xy, i)}{note}"
        )

    kappa = line.kappa
    max_kappa = float(np.abs(kappa).max())
    if max_kappa > vehicle.kappa_max:
        i = int(np.argmax(np.abs(kappa)))
        failures.append(
            f"kappa: |kappa| {max_kappa:.4f} > {vehicle.kappa_max:.4f} at "
            f"{_at(xy, i)}"
        )
    dk = np.abs(kappa - _kappa_geometry(xy))
    if dk.max() > KAPPA_TOL:
        i = int(np.argmax(dk))
        failures.append(
            f"kappa column: differs from the path's curvature by "
            f"{dk.max():.4f} > {KAPPA_TOL} at {_at(xy, i)}"
        )

    v, a = line.v, line.a
    if v.min() <= 0.0 or v.max() > vehicle.v_max * (1 + LIMIT_TOL):
        i = int(np.argmax(np.maximum(v - vehicle.v_max, -v)))
        failures.append(
            f"speed: v outside (0, {vehicle.v_max}] at {_at(xy, i)}"
        )
    ay = np.interp(v, vehicle.ggv[:, 0], vehicle.ggv[:, 2])
    ay_ratio = v**2 * np.abs(kappa) / ay
    if ay_ratio.max() > 1.0 + LIMIT_TOL:
        i = int(np.argmax(ay_ratio))
        failures.append(
            f"a_y: v^2|kappa| is {ay_ratio.max():.6f} x the GGV limit at "
            f"{_at(xy, i)}"
        )
    ax_ggv = np.interp(v, vehicle.ggv[:, 0], vehicle.ggv[:, 1])
    ax_eng = np.interp(
        v, vehicle.ax_max_machines[:, 0], vehicle.ax_max_machines[:, 1]
    )
    ax_ratio = np.where(a > 0, a / np.minimum(ax_ggv, ax_eng), -a / ax_ggv)
    if ax_ratio.max() > 1.0 + LIMIT_TOL:
        i = int(np.argmax(ax_ratio))
        failures.append(
            f"a_x: a = {a[i]:.3f} is {ax_ratio.max():.6f} x the limit at "
            f"{_at(xy, i)}"
        )

    min_clearance: dict[str, float] = {}
    share: dict[str, float] = {}
    for wall_set, required in walls:
        c = _signed_clearance(wall_set, xy)
        i = int(np.argmin(c))
        min_clearance[wall_set.name] = float(c[i])
        if c[i] < required:
            where = " (outside the walls)" if c[i] < 0 else ""
            failures.append(
                f"clearance {wall_set.name}: min {c[i]:.3f} < required "
                f"{required:.3f} at {_at(xy, i)}{where}"
            )
        points, total = [], len(wall_set.left) + len(wall_set.right)
        for wall, seen in (
            (wall_set.left, wall_set.left_observed),
            (wall_set.right, wall_set.right_observed),
        ):
            if seen is not None:
                if len(seen) != len(wall):
                    raise ValueError(
                        f"{wall_set.name}: observed mask has {len(seen)} "
                        f"entries for {len(wall)} wall points"
                    )
                points.append(wall[~np.asarray(seen, dtype=bool)])
        hidden = np.vstack(points) if points else np.zeros((0, 2))
        share[wall_set.name] = len(hidden) / total
        if len(hidden):
            # Reported, never failed (E6): bridged or unknown-adjacent points.
            d, _ = cKDTree(hidden).query(xy)
            min_clearance[f"{wall_set.name}_unobserved"] = float(d.min())

    return Verdict(
        failures=tuple(failures),
        min_clearance=min_clearance,
        max_abs_kappa=max_kappa,
        max_ay_ratio=float(ay_ratio.max()),
        max_ax_ratio=float(ax_ratio.max()),
        unobserved_share=share,
    )
