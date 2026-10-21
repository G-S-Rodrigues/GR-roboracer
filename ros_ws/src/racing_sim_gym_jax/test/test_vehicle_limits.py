from pathlib import Path

import numpy as np
import pytest
import yaml
from f1tenth_gym_jax.envs.dynamic_models import upper_accel_limit
from f1tenth_gym_jax.envs.utils import Param

VEHICLES = Path(__file__).resolve().parents[4] / "config" / "vehicles"
GRAVITY = 9.81  # dynamic_models.py: the gym's own constant
GYM_LIMITS_V_MAX = 20.0  # owner ruling 2026-10-14: the gym's own ceiling


def _parameters(vehicle: Path, node: str) -> dict:
    document = yaml.safe_load(vehicle.read_text(encoding="utf-8"))
    return document[node]["ros__parameters"]


def _table(path: Path) -> np.ndarray:
    return np.loadtxt(path, comments="#", delimiter=",", ndmin=2)


def test_simjax_1120_gym_limits_vehicle_is_the_gyms_param() -> None:
    """SIMJAX-1120: the gym-limits vehicle's caps are the gym's `Param`.

    A cap that drifts from the gym is a vehicle the optimizer plans for and
    the simulator does not drive. `Param.model == "st"` pins the reference
    point the file's header describes (the centre of gravity).
    """
    vehicle = VEHICLES / "f1tenth_gym_limits.yaml"
    param = Param()
    supervisor = _parameters(vehicle, "racing_safety_supervisor")
    controller = _parameters(vehicle, "racing_controller_baseline")

    assert supervisor["maximum_steering_angle"] == pytest.approx(
        param.s_max, abs=1e-4
    )
    assert supervisor["maximum_steering_rate"] == pytest.approx(param.sv_max)
    assert supervisor["maximum_acceleration"] == pytest.approx(param.a_max)
    assert supervisor["vehicle_half_width"] == pytest.approx(param.width / 2)
    assert controller["wheelbase"] == pytest.approx(
        param.lf + param.lr, abs=1e-12
    )
    assert supervisor["maximum_speed"] == GYM_LIMITS_V_MAX
    assert controller["maximum_speed"] == GYM_LIMITS_V_MAX
    assert param.v_max >= GYM_LIMITS_V_MAX
    assert param.model == "st"


@pytest.mark.parametrize(
    "vehicle",
    [
        VEHICLES / "f1tenth_gym_limits.yaml",
        VEHICLES / "f1tenth_default.yaml",
    ],
    ids=lambda path: path.stem,
)
def test_simjax_1130_vehicle_tables_are_the_gyms_model(vehicle: Path) -> None:
    """SIMJAX-1130: the GGV and ax_max_machines tables are the gym's model,
    capped by the vehicle's own supervisor acceleration.

    The gym's `upper_accel_limit` runs in float32, so the CSVs (written at 6
    decimals) are compared at 1e-6, and exactly against the cap the loader
    checks them with.
    """
    param = Param()
    cap = _parameters(vehicle, "racing_safety_supervisor")[
        "maximum_acceleration"
    ]
    tables = vehicle.with_suffix("")
    machines = _table(tables / "ax_max_machines.csv")
    ggv = _table(tables / "ggv.csv")

    expected = np.array(
        [
            min(cap, float(upper_accel_limit(v, param.a_max, param.v_switch)))
            for v in machines[:, 0]
        ]
    )
    assert machines[:, 1] == pytest.approx(expected, abs=1e-6)
    assert (machines[:, 1] <= cap).all()
    assert (ggv[:, 1] == cap).all()
    assert ggv[:, 2] == pytest.approx(param.mu * GRAVITY, abs=1e-6)
    assert machines[-1, 0] >= 20.0
    assert ggv[-1, 0] >= 20.0
