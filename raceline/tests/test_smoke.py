"""Tier 1: the raceline project's install and its seam to TUM's optimizer."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

INPUTS = Path(__file__).parent / "fixtures" / "tum_inputs"

# Oval: two straights of 2 * A_HALF and two semicircles of radius R_OVAL.
A_HALF = 5.5
R_OVAL = 6.0
SPACING = 1.0
W_LEFT = 1.6
W_RIGHT = 0.6
SAFETY_WIDTH = 0.3
# TUM's exported line is a spline interpolation between QP nodes and can
# overshoot an active bound slightly between them. Measured worst overshoot
# on this oval: 0.0009 m (min -0.4492 against -0.45, max 1.4509 against 1.45).
BOUND_TOL = 0.02
MIN_ARC_CUT = 0.5


def _oval():
    """Counter-clockwise stadium centerline at ~SPACING, no repeated point."""
    straight = 2 * A_HALF
    arc = np.pi * R_OVAL
    total = 2 * straight + 2 * arc
    n = int(round(total / SPACING))
    s = np.linspace(0.0, total, n, endpoint=False)
    pts = []
    for si in s:
        if si < straight:  # bottom straight, travelling +x
            pts.append((-A_HALF + si, -R_OVAL))
        elif si < straight + arc:  # right semicircle
            ang = -np.pi / 2 + (si - straight) / R_OVAL
            pts.append((A_HALF + R_OVAL * np.cos(ang), R_OVAL * np.sin(ang)))
        elif si < 2 * straight + arc:  # top straight, travelling -x
            pts.append((A_HALF - (si - straight - arc), R_OVAL))
        else:  # left semicircle
            ang = np.pi / 2 + (si - 2 * straight - arc) / R_OVAL
            pts.append((-A_HALF + R_OVAL * np.cos(ang), R_OVAL * np.sin(ang)))
    return np.array(pts)


def _left_offset(xy):
    """Signed lateral offset from the oval centerline, left (inside) > 0."""
    q = np.clip(xy[:, 0], -A_HALF, A_HALF)
    rho = np.hypot(xy[:, 0] - q, xy[:, 1])
    return R_OVAL - rho


def test_raceline_1010_mincurv_iqp_on_asymmetric_oval(tmp_path):
    from raceline.tum import run_tum, write_tum_track

    xy = _oval()
    track = write_tum_track(xy, W_RIGHT, W_LEFT, tmp_path / "oval.csv")
    traj = run_tum(track, INPUTS, "mincurv_iqp", SAFETY_WIDTH)

    assert traj.ndim == 2 and traj.shape[1] == 7
    assert np.isfinite(traj).all()
    assert (np.diff(traj[:, 0]) > 0).all()
    gap = np.hypot(*(traj[-1, 1:3] - traj[0, 1:3]))
    assert gap < 0.2
    off = _left_offset(traj[:, 1:3])
    margin = SAFETY_WIDTH / 2
    assert off.min() >= -W_RIGHT + margin - BOUND_TOL
    assert off.max() <= W_LEFT - margin + BOUND_TOL
    # The line must depart from the centerline: a no-op optimizer returns it
    # (offset 0). Measured mean inside offset on the arcs: 1.21 m.
    arcs = np.abs(traj[:, 1]) > A_HALF
    assert off[arcs].mean() > MIN_ARC_CUT


def test_raceline_1020_env_has_no_ros():
    assert importlib.util.find_spec("rclpy") is None
    assert importlib.util.find_spec("racing_common") is None


def test_raceline_1030_tum_column_order(tmp_path):
    from raceline.tum import write_tum_track

    xy = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    path = write_tum_track(xy, 0.4, 1.6, tmp_path / "t.csv")
    data = np.loadtxt(path, delimiter=",", ndmin=2)
    assert data.shape == (3, 4)
    assert (data[:, 2] == 0.4).all()
    assert (data[:, 3] == 1.6).all()
    assert (data[:, :2] == xy).all()


def _small_track(tmp_path):
    from raceline.tum import write_tum_track

    return write_tum_track(_oval(), W_RIGHT, W_LEFT, tmp_path / "oval.csv")


def test_raceline_1040_run_tum_timeout_is_settable(tmp_path):
    import subprocess

    from raceline.tum import run_tum

    track = _small_track(tmp_path)
    # uv start-up plus TUM's imports alone take longer than this.
    with pytest.raises(subprocess.TimeoutExpired):
        run_tum(track, INPUTS, "mincurv_iqp", SAFETY_WIDTH, timeout=0.2)


def test_raceline_1050_run_tum_failure_carries_stderr(tmp_path):
    from raceline.tum import run_tum

    track = _small_track(tmp_path)
    with pytest.raises(RuntimeError) as err:
        run_tum(track, INPUTS, "no_such_opt_type", SAFETY_WIDTH)
    # TUM's own diagnostic, not just an exit status.
    assert "Unknown optimization type" in str(err.value)
