"""Optimizers (the TUM chain) and the raceline type they return."""

from __future__ import annotations

import csv
import json
import shutil
import tempfile
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from raceline.geometry import nearest_distance
from raceline.track import Track
from raceline.tum import run_tum, write_tum_track
from raceline.vehicle import (
    VehicleLimits,
    check_dynamics,
    load_tyres,
)

STEP_M = 0.1  # TUM's output spacing (stepsize_interp_after_opt)

DEFAULT_MARGIN = 0.30  # m per side; plan D2
# Per optimizer. Min-time: Q14 (owner), the gym margin; the default vehicle
# uses it too (coordinator delegation, E-1; Step 1.3 measured it feasible with
# MINTIME_WIDTH_EXTRA below, 5.8 mm of input-clearance slack).
DEFAULT_MARGINS = {"tum_mincurv_iqp": DEFAULT_MARGIN, "tum_mintime": 0.40}

# --- the min-time ini. Each value with its measurement (part 8, Step 1) -----
# Measured on the Spielberg survey map, h 0.5 regular, TUM's fork @8bedf4ca.
MINTIME_STEP_REG = 0.5  # m; the 8F ini's stepsize_reg (0.2 stalled cold: 8b)
MINTIME_STEP_NON_REG = 0  # 8b Step 1: irregular steps gave no gain
MINTIME_EPS_KAPPA = 0.02  # 8b Step 1 tables
MINTIME_CURV_WINDOW = 2.0  # m preview/review of curvature; 8b Step 1 tables
# E-1 (coordinator, 2026-10-25): +0.01 m per side on BOTH vehicles. Step 1.4:
# without it the default vehicle's input clearance is 0.521 < 0.525 required;
# with it 0.5308 (gym 0.5418 vs 0.530), at 170 / 1518 iterations.
MINTIME_WIDTH_EXTRA = 0.01
MINTIME_LIMIT_SCALE = 0.98  # of mue and the force/power limits; 8b Step 1
MINTIME_TYRE_SET = "gym_fit"  # Q3 (owner): B, the gym's own tyre model
MINTIME_MU_INIT = 1e-3  # R-SEED; Step 1: converges from a min-curv seed
# E-5 (coordinator): 2 x the worst dev iteration count (1518, Step 1.4) rounded
# up, the 8F precedent; the wall bound keeps the 120-min job's worst case at
# about 63 min (Step 1.6); the solver's threads as 4030 ran it.
MINTIME_MAX_ITER = 3036
MINTIME_MAX_WALL_S = 1200
MINTIME_THREADS = 3
_G = 9.81


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
#   mass, length    the vehicle's tum_mintime.yaml (the gym's Param, see
#                   raceline/README). Inert here: with no drag the mass cancels
#                   in the speed profile, and length feeds only check_traj's
#                   printout.
#   the rest        ForzaETH race_stack @202450f's racecar_f110.ini (as in the
#                   part-1 fixture), so the output spacing is 0.1 m.


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
            "length": {vehicle.dynamics.length},
            "width": {2 * vehicle.half_width:.6f},
            "mass": {vehicle.dynamics.mass},
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


def tum_mintime_ini(
    vehicle: VehicleLimits,
    margin: float,
    tyre_set: str = MINTIME_TYRE_SET,
    extra: float = MINTIME_WIDTH_EXTRA,
) -> str:
    """The text of TUM's ``racecar_f110.ini`` for the fork's minimum-time run.

    The min-curv ini with ``stepsize_reg`` swapped, ``curv_calc_opts`` added
    and the four min-time sections appended. Derivations, from the harness
    that measured them (part 8, 8b ``step1.py``): mue = GGV a_y(0) / g; the
    drive force is mass * min(GGV a_x(0), machines a_x(0)), the brake force
    mass * GGV a_x(0), power mass * max(v * a_machines); the last four are
    scaled by ``MINTIME_LIMIT_SCALE``.
    """
    check_dynamics(vehicle)
    dyn = vehicle.dynamics
    tyres = load_tyres(vehicle, tyre_set)
    scale = MINTIME_LIMIT_SCALE
    ggv, machines = vehicle.ggv, vehicle.ax_max_machines
    ini = tum_ini(vehicle)
    old = '"stepsize_reg": 0.2'
    assert old in ini
    ini = ini.replace(old, f'"stepsize_reg": {MINTIME_STEP_REG}')
    curv = json.dumps(
        {
            "d_preview_curv": MINTIME_CURV_WINDOW,
            "d_review_curv": MINTIME_CURV_WINDOW,
            "d_preview_head": 1.0,
            "d_review_head": 1.0,
        }
    )
    ini = ini.replace(
        "[OPTIMIZATION_OPTIONS]",
        f"curv_calc_opts={curv}\n\n[OPTIMIZATION_OPTIONS]",
    )
    sections = {
        "optim_opts_mintime": {
            "width_opt": vehicle.safety_width(margin) + 2 * extra,
            "penalty_delta": 1.0,
            "penalty_F": 0.1,
            "mue": float(ggv[0, 2]) / _G * scale,
            "n_gauss": 5,
            "dn": 0.025,
            "limit_energy": False,
            "energy_limit": 2.0,
            "safe_traj": False,
            "ax_pos_safe": None,
            "ax_neg_safe": None,
            "ay_safe": None,
            "w_tr_reopt": 1.0,
            "w_veh_reopt": 0.8,
            "w_add_spl_regr": 0.0,
            "step_non_reg": MINTIME_STEP_NON_REG,
            "eps_kappa": MINTIME_EPS_KAPPA,
        },
        "vehicle_params_mintime": {
            "wheelbase_front": dyn.wheelbase_front,
            "wheelbase_rear": dyn.wheelbase_rear,
            "track_width_front": dyn.track_width_front,
            "track_width_rear": dyn.track_width_rear,
            "cog_z": dyn.cog_z,
            "I_z": dyn.I_z,
            "liftcoeff_front": dyn.liftcoeff_front,
            "liftcoeff_rear": dyn.liftcoeff_rear,
            "k_brake_front": dyn.k_brake_front,
            "k_drive_front": dyn.k_drive_front,
            "k_roll": dyn.k_roll,
            "t_delta": dyn.t_delta,
            "t_drive": dyn.t_drive,
            "t_brake": dyn.t_brake,
            "power_max": dyn.mass
            * float(np.max(machines[:, 0] * machines[:, 1]))
            * scale,
            "f_drive_max": dyn.mass
            * min(float(ggv[0, 1]), float(machines[0, 1]))
            * scale,
            "f_brake_max": dyn.mass * float(ggv[0, 1]) * scale,
            "delta_max": vehicle.max_steering_angle,
        },
        "tire_params_mintime": {
            "c_roll": dyn.c_roll,
            "f_z0": tyres.f_z0,
            "B_front": tyres.B_front,
            "C_front": tyres.C_front,
            "eps_front": tyres.eps_front,
            "E_front": tyres.E_front,
            "B_rear": tyres.B_rear,
            "C_rear": tyres.C_rear,
            "eps_rear": tyres.eps_rear,
            "E_rear": tyres.E_rear,
        },
        "pwr_params_mintime": {"pwr_behavior": False},
    }
    for key, value in sections.items():
        ini += f"{key}={json.dumps(value)}\n"
    return ini


def write_tum_inputs(
    vehicle: VehicleLimits, out_dir: Path, ini: str | None = None
) -> Path:
    """TUM's inputs directory: the ini and ``veh_dyn_info/`` (two tables).

    ``ini`` is the ini text; the min-curv one when omitted.
    """
    out_dir = Path(out_dir)
    (out_dir / "veh_dyn_info").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(vehicle.ggv_path, out_dir / "veh_dyn_info" / "ggv.csv")
    shutil.copyfile(
        vehicle.ax_max_machines_path,
        out_dir / "veh_dyn_info" / "ax_max_machines.csv",
    )
    (out_dir / "racecar_f110.ini").write_text(
        tum_ini(vehicle) if ini is None else ini
    )
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
