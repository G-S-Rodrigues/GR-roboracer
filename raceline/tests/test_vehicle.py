"""Tier 1: the vehicle-limits loader and the gym-limits tables TUM reads."""

import shutil
from pathlib import Path

import numpy as np
from test_smoke import INPUTS, W_LEFT, W_RIGHT, _oval

VEHICLES = Path(__file__).resolve().parents[2] / "config" / "vehicles"

# The gym-limits GGV (ay = mu * g) and the supervisor acceleration (9.51).
GYM_AY = 1.0489 * 9.81
GYM_AX = 9.51
# Measured on the oval: max v^2*|kappa| = 10.289709000000002 against 10.289709
# (excess 2e-15, float noise: the profile sits exactly on the table), max |ax|
# 5.1536 against 9.51 (no excess). Both tolerances rounded up to 1e-6.
AY_TOL = 1e-6
AX_TOL = 1e-6


def _tum_inputs(tmp_path, veh_dyn_info):
    inputs = tmp_path / "inputs"
    shutil.copytree(veh_dyn_info, inputs / "veh_dyn_info")
    shutil.copy(INPUTS / "racecar_f110.ini", inputs)
    return inputs


def test_raceline_1250_tum_accepts_the_gym_limits_ggv(tmp_path):
    from raceline.tum import run_tum, write_tum_track

    inputs = _tum_inputs(tmp_path, VEHICLES / "f1tenth_gym_limits")
    track = write_tum_track(_oval(), W_RIGHT, W_LEFT, tmp_path / "oval.csv")
    traj = run_tum(track, inputs, "mincurv_iqp", 0.3, timeout=120)

    assert traj.ndim == 2 and traj.shape[1] == 7
    assert np.isfinite(traj).all()
    ay = (traj[:, 5] ** 2 * np.abs(traj[:, 4])).max()
    ax = np.abs(traj[:, 6]).max()
    print(f"max ay {ay:.4f} (bound {GYM_AY:.4f}), max |ax| {ax:.4f}")
    assert ay <= GYM_AY + AY_TOL
    assert ax <= GYM_AX + AX_TOL
    # Grip-limited: the bound is actually exercised, not just respected.
    assert ay > 9.0
