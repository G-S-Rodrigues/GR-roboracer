"""Optimizers (the TUM chain) and the raceline type they return."""

from __future__ import annotations

import csv
import shutil
import tempfile
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from raceline.geometry import nearest_distance
from raceline.track import Track
from raceline.tum import run_tum, write_tum_track
from raceline.vehicle import VehicleLimits

STEP_M = 0.1  # TUM's output spacing (stepsize_interp_after_opt)

DEFAULT_MARGIN = 0.30  # m per side; plan D2


@dataclass(frozen=True)
class Raceline:
    """A CoG path: s, x, y, psi (ROS yaw), kappa, v, a. Closed, no repeat."""

    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray
    kappa: np.ndarray
    v: np.ndarray
    a: np.ndarray

    @property
    def xy(self) -> np.ndarray:
        return np.column_stack([self.x, self.y])


def write_raceline_csv(line: Raceline, path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cols = (line.s, line.x, line.y, line.psi, line.kappa, line.v, line.a)
    with path.open("w", newline="") as f:
        w = csv.writer(f, delimiter=";", lineterminator="\n")
        for row in zip(*cols, strict=True):
            w.writerow([f"{value:.9f}" for value in row])
    return path


def load_raceline_csv(path: Path) -> Raceline:
    rows = np.loadtxt(path, delimiter=";", ndmin=2)
    if rows.shape[1] != 7:
        raise ValueError(f"{path}: expected 7 columns, got {rows.shape[1]}")
    return Raceline(*rows.T)


# TUM's mincurv_iqp reads only GENERAL_OPTIONS and optim_opts_mincurv
# (trajectory_optimizer.py:157-168). Fixed values, each with its reason:
#   dragcoeff 0.0   f1tenth_gym_jax's dynamics have no aerodynamic drag.
#   mass, length    the gym's Param (raceline/README). Inert: with no drag the
#                   mass cancels in the speed profile, and length feeds only
#                   check_traj's printout.
#   the rest        ForzaETH race_stack @202450f's racecar_f110.ini (as in the
#                   part-1 fixture), so the output spacing is 0.1 m.
_GYM_MASS = 3.74  # kg
_GYM_LENGTH = 0.58  # m


def tum_ini(vehicle: VehicleLimits) -> str:
    """The text of TUM's ``racecar_f110.ini``, derived from the vehicle."""
    return f"""[GENERAL_OPTIONS]
ggv_file="ggv.csv"
ax_max_machines_file="ax_max_machines.csv"
stepsize_opts={{"stepsize_prep": 0.05,
               "stepsize_reg": 0.2,
               "stepsize_interp_after_opt": {STEP_M}}}
reg_smooth_opts={{"k_reg": 3,
                 "s_reg": 1}}
veh_params={{"v_max": {vehicle.v_max},
            "length": {_GYM_LENGTH},
            "width": {2 * vehicle.half_width:.6f},
            "mass": {_GYM_MASS},
            "dragcoeff": 0.0,
            "curvlim": {vehicle.kappa_max:.9f},
            "g": 9.81}}
vel_calc_opts={{"dyn_model_exp": 1.0,
               "vel_profile_conv_filt_window": null}}

[OPTIMIZATION_OPTIONS]
optim_opts_mincurv={{"width_opt": 0.8,
                    "iqp_iters_min": 5,
                    "iqp_curverror_allowed": 0.1}}
"""


def write_tum_inputs(vehicle: VehicleLimits, out_dir: Path) -> Path:
    """TUM's inputs directory: the ini and ``veh_dyn_info/`` (two tables)."""
    out_dir = Path(out_dir)
    (out_dir / "veh_dyn_info").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(vehicle.ggv_path, out_dir / "veh_dyn_info" / "ggv.csv")
    shutil.copyfile(
        vehicle.ax_max_machines_path,
        out_dir / "veh_dyn_info" / "ax_max_machines.csv",
    )
    (out_dir / "racecar_f110.ini").write_text(tum_ini(vehicle))
    return out_dir


def _tum_mincurv_iqp(
    track: Track,
    vehicle: VehicleLimits,
    margin: float,
    passes: int,
    timeout: float | None,
) -> Raceline:
    """ForzaETH-style repeated IQP: each pass's line is the next centerline.

    Pass 1 optimizes the input track. Between passes the previous line becomes
    the reference, with its widths measured to the same walls rebuilt as
    ``xy +- w n`` (how TUM reads widths, so the rebuilt walls are the
    conservative ones; folds are not cut from them).
    """
    from raceline.verify import offsets  # verify imports this module

    left, right = offsets(track)  # raw, not the cleaned ground-truth walls
    reference = track
    with tempfile.TemporaryDirectory() as tmp:
        inputs = write_tum_inputs(vehicle, Path(tmp) / "inputs")
        for k in range(passes):
            csv_path = write_tum_track(
                reference.xy,
                reference.w_right,
                reference.w_left,
                Path(tmp) / f"track{k}.csv",
            )
            traj = run_tum(
                csv_path,
                inputs,
                "mincurv_iqp",
                vehicle.safety_width(margin),
                timeout,
            )[:-1]  # TUM repeats the first point at s = L
            if k + 1 < passes:
                xy = traj[:, 1:3]
                reference = Track(
                    f"{track.name}-pass{k + 1}",
                    xy,
                    np.zeros(len(xy)),
                    nearest_distance(xy, left, STEP_M),
                    nearest_distance(xy, right, STEP_M),
                )
    psi = np.arctan2(
        np.sin(traj[:, 3] + np.pi / 2), np.cos(traj[:, 3] + np.pi / 2)
    )
    return Raceline(
        traj[:, 0],
        traj[:, 1],
        traj[:, 2],
        psi,
        traj[:, 4],
        traj[:, 5],
        traj[:, 6],
    )


# An open set: part 8 adds "tum_mintime".
OPTIMIZERS: dict[str, Callable[..., Raceline]] = {
    "tum_mincurv_iqp": _tum_mincurv_iqp
}


def optimize(
    track: Track,
    vehicle: VehicleLimits,
    optimizer: str,
    *,
    margin: float = DEFAULT_MARGIN,
    passes: int = 2,
    timeout: float | None = None,
) -> Raceline:
    if optimizer not in OPTIMIZERS:
        raise ValueError(
            f"unknown optimizer {optimizer!r}; known: {sorted(OPTIMIZERS)}"
        )
    if passes < 1:
        raise ValueError(f"passes must be >= 1, got {passes}")
    return OPTIMIZERS[optimizer](track, vehicle, margin, passes, timeout)
