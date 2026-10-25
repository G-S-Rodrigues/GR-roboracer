"""Vehicle limits for the raceline optimizer, read from a vehicle file.

The numbers come from the same file the ROS nodes read (config/vehicles/*.yaml),
so there is no second copy to drift (repo-gotchas #15's shape). The GGV and
ax_max_machines tables sit beside the file at ``<yaml stem>/``.

This module is ROS-free and cannot import tph (it lives in the isolated
tum_env/), so the table checks below mirror tph's ``import_veh_dyn_info`` and
``calc_vel_profile``; RACELINE-1250 proves TUM accepts the committed tables.
"""

from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np
import yaml

# tph.import_veh_dyn_info: ggv accelerations <= 50, machines <= 20.
_TPH_GGV_MAX = 50.0
_TPH_MACHINES_MAX = 20.0


@dataclass(frozen=True)
class VehicleDynamics:
    """TUM's minimum-time vehicle model, ``<stem>/tum_mintime.yaml``.

    Field names are the file's keys, which are TUM's ``vehicle_params_mintime``
    names. Documented in ``config/vehicles/tum_mintime.reference.yaml``.
    """

    mass: float
    I_z: float
    wheelbase_front: float
    wheelbase_rear: float
    cog_z: float
    length: float
    track_width_front: float
    track_width_rear: float
    k_brake_front: float
    k_drive_front: float
    k_roll: float
    t_delta: float
    t_drive: float
    t_brake: float
    c_roll: float
    liftcoeff_front: float
    liftcoeff_rear: float


@dataclass(frozen=True)
class TyreSet:
    """A Pacejka tyre set, ``<stem>/tum_mintime_tyres_<name>.yaml``."""

    name: str
    path: Path
    f_z0: float
    B_front: float
    C_front: float
    eps_front: float
    E_front: float
    B_rear: float
    C_rear: float
    eps_rear: float
    E_rear: float


# Q3 (owner, 2026-10-22): both sets are committed.
TYRE_SETS = ("forzaeth_f110", "gym_fit")


@dataclass(frozen=True)
class VehicleLimits:
    version: str  # racing_metrics.vehicle_parameter_version
    half_width: float  # racing_safety_supervisor.vehicle_half_width
    wheelbase: float  # racing_controller_baseline.wheelbase
    max_steering_angle: float  # racing_safety_supervisor
    kappa_max: float  # tan(max_steering_angle) / wheelbase
    v_max: float  # racing_safety_supervisor.maximum_speed
    a_x_max: float  # racing_safety_supervisor.maximum_acceleration
    ggv: np.ndarray  # (n, 3): v, ax, ay -- TUM's ggv.csv
    ax_max_machines: np.ndarray  # (m, 2): v, ax
    ggv_path: Path
    ax_max_machines_path: Path
    dynamics: VehicleDynamics
    dynamics_path: Path

    def safety_width(self, margin: float) -> float:
        """TUM's ``safety_width``: the car's width plus a wall margin per side.

        The margin is an optimizer option, not a vehicle property.
        """
        if margin < 0:
            raise ValueError(f"wall margin must be >= 0, got {margin}")
        return 2.0 * (self.half_width + margin)


def _parameter(document: dict, node: str, key: str, path: Path):
    try:
        return document[node]["ros__parameters"][key]
    except (KeyError, TypeError) as error:
        raise ValueError(
            f"{path}: missing {node}.ros__parameters.{key}"
        ) from error


def _table(path: Path, columns: int, name: str) -> np.ndarray:
    if not path.is_file():
        raise ValueError(f"{name} table not found: {path}")
    try:
        table = np.loadtxt(path, comments="#", delimiter=",", ndmin=2)
    except ValueError as error:
        raise ValueError(f"{path}: unreadable {name} table: {error}") from error
    if table.shape[1] != columns:
        raise ValueError(
            f"{path}: {name} needs {columns} columns, has {table.shape[1]}"
        )
    return table


def _flat_file(path: Path, cls, what: str) -> dict:
    """A values-only yaml whose keys are exactly ``cls``'s fields."""
    if not path.is_file():
        raise ValueError(f"{what} file not found: {path}")
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{path}: expected a mapping of key: value")
    wanted = {f.name for f in fields(cls)} - {"name", "path"}
    for key in sorted(wanted - set(document)):
        raise ValueError(f"{path}: missing key {key!r}")
    for key in sorted(set(document) - wanted):
        raise ValueError(f"{path}: unknown key {key!r}")
    try:
        return {key: float(value) for key, value in document.items()}
    except (TypeError, ValueError) as error:
        raise ValueError(f"{path}: a value is not a number: {error}") from error


# The default vehicle file says wheelbase 0.33 and the gym's lf + lr is
# 0.3302 (0.2 mm, see f1tenth_gym_limits.yaml's header); 1 mm separates that
# from a mistyped lf or lr, which is centimetres.
_WHEELBASE_TOL = 1e-3


def load_dynamics(path: Path) -> VehicleDynamics:
    """Load ``tum_mintime.yaml``; raise ``ValueError`` naming the bad key."""
    path = Path(path)
    dynamics = VehicleDynamics(**_flat_file(path, VehicleDynamics, "dynamics"))
    for key in ("mass", "I_z"):
        if getattr(dynamics, key) <= 0:
            raise ValueError(f"{path}: {key} must be > 0")
    return dynamics


def check_dynamics(vehicle: "VehicleLimits") -> None:
    """Cross-check the dynamics against the vehicle's wheelbase.

    Lazy, not in ``load_vehicle``: only the minimum-time path reads the
    dynamics, so a wheelbase edit must not break the min-curv loader.
    """
    dynamics = vehicle.dynamics
    reach = dynamics.wheelbase_front + dynamics.wheelbase_rear
    if abs(reach - vehicle.wheelbase) > _WHEELBASE_TOL:
        raise ValueError(
            f"{vehicle.dynamics_path}: wheelbase_front + wheelbase_rear = "
            f"{reach} is not the vehicle's wheelbase {vehicle.wheelbase}"
        )


def load_tyres(vehicle: "VehicleLimits", name: str) -> TyreSet:
    """Load the named Pacejka set that sits beside the vehicle's tables."""
    if name not in TYRE_SETS:
        raise ValueError(
            f"unknown tyre set {name!r}; known sets: {', '.join(TYRE_SETS)}"
        )
    path = vehicle.dynamics_path.parent / f"tum_mintime_tyres_{name}.yaml"
    return TyreSet(name=name, path=path, **_flat_file(path, TyreSet, "tyre"))


def load_vehicle(path: Path) -> VehicleLimits:
    """Load and cross-check a vehicle file; raise ``ValueError`` on a fault."""
    path = Path(path)
    if not path.is_file():
        raise ValueError(f"vehicle file not found: {path}")
    document = yaml.safe_load(path.read_text(encoding="utf-8"))

    def get(node, key):
        return _parameter(document, node, key, path)

    supervisor = "racing_safety_supervisor"
    version = get("racing_metrics", "vehicle_parameter_version")
    recorded = get("racing_recording", "vehicle_parameter_version")
    if version != recorded:
        raise ValueError(
            f"{path}: version mismatch, metrics {version!r} vs "
            f"recording {recorded!r}"
        )
    v_max = float(get(supervisor, "maximum_speed"))
    a_x_max = float(get(supervisor, "maximum_acceleration"))
    steering = float(get(supervisor, "maximum_steering_angle"))
    half_width = float(get(supervisor, "vehicle_half_width"))
    wheelbase = float(get("racing_controller_baseline", "wheelbase"))
    # Pure Pursuit clamps its command to its own cap, so a planner speed above
    # it would be unreachable and read as a slow but plausible lap.
    controller_v_max = float(get("racing_controller_baseline", "maximum_speed"))
    if controller_v_max != v_max:
        raise ValueError(
            f"{path}: maximum_speed differs, controller {controller_v_max} "
            f"vs supervisor {v_max}"
        )

    tables = path.with_suffix("")
    ggv_path = tables / "ggv.csv"
    machines_path = tables / "ax_max_machines.csv"
    ggv = _table(ggv_path, 3, "ggv")
    machines = _table(machines_path, 2, "ax_max_machines")

    if (ggv < 0).any() or (machines < 0).any():
        raise ValueError(f"{tables}: negative speed or acceleration in a table")
    if (ggv[:, 1:] > _TPH_GGV_MAX).any():
        raise ValueError(
            f"{ggv_path}: acceleration above {_TPH_GGV_MAX:g} (tph)"
        )
    if (machines[:, 1] > _TPH_MACHINES_MAX).any():
        raise ValueError(
            f"{machines_path}: acceleration above {_TPH_MACHINES_MAX:g} (tph)"
        )
    if (ggv[:, 1] > a_x_max).any():
        raise ValueError(
            f"{ggv_path}: ggv ax exceeds the supervisor's "
            f"maximum_acceleration {a_x_max}"
        )
    if (machines[:, 1] > a_x_max).any():
        raise ValueError(
            f"{machines_path}: ax_max_machines exceeds the supervisor's "
            f"maximum_acceleration {a_x_max}"
        )
    for table_path, table in ((ggv_path, ggv), (machines_path, machines)):
        # tph interpolates with np.interp, which is silent on an unsorted grid.
        if (np.diff(table[:, 0]) <= 0).any():
            raise ValueError(
                f"{table_path}: speed column is not strictly increasing"
            )
        if table[:, 0].max() < v_max:
            raise ValueError(
                f"{table_path}: does not cover maximum_speed {v_max}"
            )

    return VehicleLimits(
        version=version,
        half_width=half_width,
        wheelbase=wheelbase,
        max_steering_angle=steering,
        kappa_max=float(np.tan(steering) / wheelbase),
        v_max=v_max,
        a_x_max=a_x_max,
        ggv=ggv,
        ax_max_machines=machines,
        ggv_path=ggv_path,
        ax_max_machines_path=machines_path,
        dynamics=load_dynamics(tables / "tum_mintime.yaml"),
        dynamics_path=tables / "tum_mintime.yaml",
    )
