"""Vehicle limits for the raceline optimizer, read from a vehicle file.

The numbers come from the same file the ROS nodes read (config/vehicles/*.yaml),
so there is no second copy to drift (repo-gotchas #15's shape). The GGV and
ax_max_machines tables sit beside the file at ``<yaml stem>/``.

This module is ROS-free and cannot import tph (it lives in the isolated
tum_env/), so the table checks below mirror tph's ``import_veh_dyn_info`` and
``calc_vel_profile``; RACELINE-1250 proves TUM accepts the committed tables.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

# tph.import_veh_dyn_info: ggv accelerations <= 50, machines <= 20.
_TPH_GGV_MAX = 50.0
_TPH_MACHINES_MAX = 20.0


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
    )
