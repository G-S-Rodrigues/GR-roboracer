"""Optimizers (the TUM chain) and the raceline type they return."""

from __future__ import annotations

import csv
import json
import shutil
import tempfile
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from raceline.geometry import nearest_distance
from raceline.track import Track
from raceline.tum import run_tum, run_tum_mintime, write_tum_track
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


def _reference_from_line(name: str, xy: np.ndarray, track: Track) -> Track:
    """A line as the next reference: a track whose centerline it is.

    Its widths are measured to the same walls rebuilt as ``xy +- w n`` (how
    TUM reads widths, so the rebuilt walls are the conservative ones; folds
    are not cut from them), raw, not the cleaned ground-truth walls.
    """
    from raceline.verify import offsets  # verify imports this module

    left, right = offsets(track)
    xy = np.asarray(xy)
    return Track(
        name,
        xy,
        np.zeros(len(xy)),
        nearest_distance(xy, left, STEP_M),
        nearest_distance(xy, right, STEP_M),
    )


def _raceline_from_tum(traj: np.ndarray) -> Raceline:
    """A ``Raceline`` from TUM's 7-column trajectory (no closing row).

    TUM's psi is 0 = north; the raceline's is ROS yaw.
    """
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


# The min-time solver's exported path is more curved than its own states
# (Porto cw, autodrive_roboracer_fast, margin 0.40: kappa -0.663 against the
# model's -0.357 at s 26.5 m), so v^2 |kappa| on it reached 1.052 x the GGV
# a_y. Re-profiled at 0.995 x a_y, the line verified with max a_y ratio 0.995
# and a lap estimate of 7.303 -> 7.306 s. The base car (a_x 1.044) verified
# at 10.004 -> 10.004 s, and margin 0.30 at 7.127 -> 7.128 s (throwaway,
# findings 2026-11-08-part1-step2-ggv-overshoot). That throwaway scaled
# a_y only; the same factor also scales a_x, braking and accelerating, so no
# point sits at ratio 1.0 within CSV rounding (ledger 2026-11-09 00:20). Its
# effect on the lap estimate is the Porto record's (raceline/README.md). It
# is a conservative factor on the limits the line is built to, not a
# verifier tolerance (ledger 2026-11-08 23:55).
REPROFILE_LIMIT_SCALE = 0.995


def _table_min(v_col, y_col, lo: float, hi: float) -> float:
    """Minimum of the piecewise-linear interpolant of ``y_col`` over ``v_col``
    on [lo, hi] (either order): both ends and every knot strictly inside."""
    lo, hi = min(lo, hi), max(lo, hi)
    inside = [x for x in v_col if lo < x < hi]
    return float(np.min(np.interp([lo, hi, *inside], v_col, y_col)))


def reprofile_speed(
    line: Raceline,
    vehicle: VehicleLimits,
    *,
    limit_scale: float = REPROFILE_LIMIT_SCALE,
) -> Raceline:
    """The line's speed re-profiled on its own path; geometry untouched.

    Speed is capped to ``limit_scale`` x the GGV a_y where ``v^2 |kappa|``
    would exceed it, then braked backward and accelerated forward around the
    closed loop (a_x from the GGV, and the machines table when accelerating,
    all x ``limit_scale``), the limits read as ``verify._limit_ratios`` reads
    them. Never faster than the input; ``a`` is the closed forward difference.
    """
    xy = line.xy
    n = len(xy)
    ds = np.empty(n)
    ds[:-1] = line.s[1:] - line.s[:-1]
    ds[-1] = np.hypot(*(xy[0] - xy[-1]))
    ggv_v, ggv_ax, ggv_ay = (
        vehicle.ggv[:, 0],
        vehicle.ggv[:, 1],
        vehicle.ggv[:, 2],
    )
    mach_v, mach_ax = (
        vehicle.ax_max_machines[:, 0],
        vehicle.ax_max_machines[:, 1],
    )
    k_abs = np.abs(line.kappa)
    v = line.v.astype(float).copy()
    for i in range(n):
        if k_abs[i] > 0.0:
            ay = _table_min(ggv_v, ggv_ay, 0.0, v[i])
            v[i] = min(v[i], np.sqrt(limit_scale * ay / k_abs[i]))
    start = int(np.argmin(v))
    for t in range(n):  # braking, backward from the slowest point
        i = (start - 1 - t) % n
        j = (i + 1) % n
        ax = limit_scale * _table_min(ggv_v, ggv_ax, v[j], v[i])
        v[i] = min(v[i], np.sqrt(v[j] ** 2 + 2.0 * ds[i] * ax))
    for t in range(n):  # accelerating, forward from the slowest point
        i = (start + t) % n
        j = (i + 1) % n
        ax = limit_scale * min(
            float(np.interp(v[i], ggv_v, ggv_ax)),
            float(np.interp(v[i], mach_v, mach_ax)),
        )
        v[j] = min(v[j], np.sqrt(v[i] ** 2 + 2.0 * ds[i] * ax))
    a = (np.roll(v, -1) ** 2 - v**2) / (2.0 * ds)
    return replace(line, v=v, a=a)


def _lap_time(line: Raceline) -> float:
    xy = line.xy
    ds = np.hypot(*(np.roll(xy, -1, 0) - xy).T)
    return float(np.sum(2.0 * ds / (line.v + np.roll(line.v, -1))))


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
                reference = _reference_from_line(
                    f"{track.name}-pass{k + 1}", traj[:, 1:3], track
                )
    return _raceline_from_tum(traj)


def mintime_chain(
    track: Track,
    vehicle: VehicleLimits,
    margin: float,
    timeout: float | None,
    *,
    reference_csv: Path | None = None,
    reference_out: Path | None = None,
) -> tuple[Raceline, dict]:
    """Map -> min-curv reference -> min-time seeded from it (R-SEED).

    Without ``reference_csv`` the min-curv stages run at the min-time margin
    (P8-R2: one margin) and their line is written to ``reference_out``. The
    reference is then **always read back from disk**: the CSV keeps 9 decimals
    and the solver's iteration path is sensitive at that level (817 vs 1715
    iterations on the same optimum), so a run from the committed file and an
    end-to-end run make the same computation. Returns the line and the
    solver's status.
    """
    if reference_csv is None:
        if reference_out is None:
            raise ValueError("reference_out is needed without a reference_csv")
        line = _tum_mincurv_iqp(
            track,
            vehicle,
            margin,
            DEFAULT_PASSES["tum_mincurv_iqp"],
            timeout,
        )
        write_raceline_csv(line, reference_out)
        reference_csv = Path(reference_out)
    reference_csv = Path(reference_csv)
    seed = load_raceline_csv(reference_csv)
    reference = _reference_from_line(f"{track.name}-reference", seed.xy, track)
    options = {
        "mu_init": MINTIME_MU_INIT,
        "max_iter": MINTIME_MAX_ITER,
        "max_wall_time_s": MINTIME_MAX_WALL_S,
        "threads": MINTIME_THREADS,
    }
    with tempfile.TemporaryDirectory() as tmp:
        inputs = write_tum_inputs(
            vehicle, Path(tmp) / "inputs", tum_mintime_ini(vehicle, margin)
        )
        csv_path = write_tum_track(
            reference.xy,
            reference.w_right,
            reference.w_left,
            Path(tmp) / "track.csv",
        )
        traj, status = run_tum_mintime(
            csv_path,
            inputs,
            vehicle.safety_width(margin),
            reference_csv,
            options,
            timeout,
        )
    return _raceline_from_tum(traj[:-1]), status  # TUM repeats row 0


def _tum_mintime(track, vehicle, margin, passes, timeout, reference=None):
    """The registry entry: the chain with a throwaway reference file."""
    if passes != DEFAULT_PASSES["tum_mintime"]:
        raise ValueError(
            f"tum_mintime is one solve, passes must be 1, got {passes}"
        )
    with tempfile.TemporaryDirectory() as tmp:
        line, _ = mintime_chain(
            track,
            vehicle,
            margin,
            timeout,
            reference_csv=reference,
            reference_out=Path(tmp) / "reference.csv",
        )
    return line


OPTIMIZERS: dict[str, Callable[..., Raceline]] = {
    "tum_mincurv_iqp": _tum_mincurv_iqp,
    "tum_mintime": _tum_mintime,
}
# The min-curv passes feed one another; min-time is one solve seeded from a
# min-curv line (R-SEED), so it has no passes of its own.
DEFAULT_PASSES = {"tum_mincurv_iqp": 2, "tum_mintime": 1}


def optimize(
    track: Track,
    vehicle: VehicleLimits,
    optimizer: str,
    *,
    margin: float | None = None,
    passes: int | None = None,
    timeout: float | None = None,
    reference: Path | None = None,
) -> Raceline:
    if optimizer not in OPTIMIZERS:
        raise ValueError(
            f"unknown optimizer {optimizer!r}; known: {sorted(OPTIMIZERS)}"
        )
    margin = DEFAULT_MARGINS[optimizer] if margin is None else margin
    passes = DEFAULT_PASSES[optimizer] if passes is None else passes
    if passes < 1:
        raise ValueError(f"passes must be >= 1, got {passes}")
    if reference is None:
        return OPTIMIZERS[optimizer](track, vehicle, margin, passes, timeout)
    if optimizer != "tum_mintime":
        raise ValueError(f"{optimizer} takes no reference line")
    return OPTIMIZERS[optimizer](
        track, vehicle, margin, passes, timeout, reference=reference
    )
