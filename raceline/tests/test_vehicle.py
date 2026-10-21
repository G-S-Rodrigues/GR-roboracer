"""Tier 1: the vehicle-limits loader and the gym-limits tables TUM reads."""

import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml
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


GYM_LIMITS = VEHICLES / "f1tenth_gym_limits.yaml"


def _copy_vehicle(tmp_path, edit=None, ggv=None, machines=None):
    """A tmp_path copy of the gym-limits yaml and tables, optionally edited.

    ``edit(document)`` mutates the parsed yaml; ``ggv``/``machines`` replace
    the table text.
    """
    document = yaml.safe_load(GYM_LIMITS.read_text())
    if edit:
        edit(document)
    path = tmp_path / "vehicle.yaml"
    path.write_text(yaml.safe_dump(document))
    tables = tmp_path / "vehicle"
    shutil.copytree(VEHICLES / "f1tenth_gym_limits", tables)
    if ggv is not None:
        (tables / "ggv.csv").write_text(ggv)
    if machines is not None:
        (tables / "ax_max_machines.csv").write_text(machines)
    return path


def _controller(key, value):
    def edit(document):
        document["racing_controller_baseline"]["ros__parameters"][key] = value

    return edit


def _supervisor(key, value):
    def edit(document):
        document["racing_safety_supervisor"]["ros__parameters"][key] = value

    return edit


def _speed_cap(value):
    """Both maximum_speed keys, which the loader requires to agree."""

    def edit(document):
        _controller("maximum_speed", value)(document)
        _supervisor("maximum_speed", value)(document)

    return edit


def test_raceline_1210_gym_limits_vehicle_loads_its_own_numbers():
    from raceline.vehicle import load_vehicle

    limits = load_vehicle(GYM_LIMITS)

    assert limits.version == "f1tenth-gym-limits-v1"
    assert limits.half_width == 0.155
    assert limits.wheelbase == 0.3302
    assert limits.max_steering_angle == 0.4189
    assert limits.kappa_max == pytest.approx(np.tan(0.4189) / 0.3302)
    assert limits.v_max == 20.0
    assert limits.a_x_max == 9.51
    assert limits.ggv.shape == (21, 3)
    assert limits.ax_max_machines.shape[1] == 2
    assert limits.ggv_path.name == "ggv.csv"
    assert limits.safety_width(0.1) == pytest.approx(0.51)
    with pytest.raises(ValueError, match="margin"):
        limits.safety_width(-0.1)


def test_raceline_1220_every_committed_vehicle_loads():
    from raceline.vehicle import load_vehicle

    files = sorted(VEHICLES.glob("*.yaml"))
    assert len(files) >= 2
    versions = [load_vehicle(path).version for path in files]
    assert len(set(versions)) == len(versions)


@pytest.mark.parametrize(
    ("edit", "field", "value"),
    [
        (_speed_cap(7.0), "v_max", 7.0),
        (_supervisor("maximum_acceleration", 9.6), "a_x_max", 9.6),
        (_supervisor("vehicle_half_width", 0.2), "half_width", 0.2),
        (
            _supervisor("maximum_steering_angle", 0.3),
            "max_steering_angle",
            0.3,
        ),
    ],
    ids=["v_max", "a_x_max", "half_width", "max_steering_angle"],
)
def test_raceline_1230_a_changed_key_moves_the_loaded_limit(
    tmp_path, edit, field, value
):
    from raceline.vehicle import load_vehicle

    limits = load_vehicle(_copy_vehicle(tmp_path, edit))
    unedited = load_vehicle(GYM_LIMITS)

    assert getattr(limits, field) == value
    for other in ("v_max", "a_x_max", "half_width", "max_steering_angle"):
        if other != field:
            assert getattr(limits, other) == getattr(unedited, other)
    assert limits.wheelbase == unedited.wheelbase


def test_raceline_1230_a_changed_wheelbase_moves_wheelbase_and_kappa(
    tmp_path,
):
    from raceline.vehicle import load_vehicle

    def edit(document):
        document["racing_controller_baseline"]["ros__parameters"][
            "wheelbase"
        ] = 0.5

    limits = load_vehicle(_copy_vehicle(tmp_path, edit))

    assert limits.wheelbase == 0.5
    assert limits.kappa_max == pytest.approx(np.tan(0.4189) / 0.5)


def _drop(section, key):
    def edit(document):
        del document[section]["ros__parameters"][key]

    return edit


def _metrics_version(document):
    document["racing_metrics"]["ros__parameters"][
        "vehicle_parameter_version"
    ] = "other-v1"


GGV_OK = "0,9.51,10.0\n20,9.51,10.0\n"
MACHINES_OK = "0,9.51\n20,3.0\n"


@pytest.mark.parametrize(
    ("kwargs", "cause"),
    [
        ({"ggv": "0,9.6,10\n20,9.6,10\n"}, "ggv ax"),
        ({"machines": "0,9.6\n20,3\n"}, "ax_max_machines"),
        ({"ggv": "0,9.51,10\n7,9.51,10\n"}, "cover"),
        ({"machines": "0,9.51\n7,3\n"}, "cover"),
        ({"ggv": "-1,9.51,10\n20,9.51,10\n"}, "negative"),
        ({"ggv": "0,-1,10\n20,9.51,10\n"}, "negative"),
        ({"ggv": "0,9.51,10\n20,9.51,60\n"}, "above 50"),
        (
            {
                "edit": _supervisor("maximum_acceleration", 25.0),
                "machines": "0,21\n20,3\n",
            },
            "above 20",
        ),
        ({"ggv": "0,9.51\n20,9.51\n"}, "column"),
        ({"machines": "0,9.51,1\n20,3,1\n"}, "column"),
        ({"edit": _metrics_version}, "version"),
        ({"edit": _controller("maximum_speed", 5.0)}, "differs"),
        (
            {"ggv": "0,9.51,10\n10,9.51,10\n5,9.51,10\n20,9.51,10\n"},
            "increasing",
        ),
        ({"machines": "0,9.51\n10,3\n10,3\n20,3\n"}, "increasing"),
        (
            {"edit": _drop("racing_safety_supervisor", "maximum_speed")},
            "maximum_speed",
        ),
        (
            {"edit": _drop("racing_controller_baseline", "wheelbase")},
            "wheelbase",
        ),
    ],
    ids=[
        "ggv_ax_over_cap",
        "machines_over_cap",
        "ggv_short_of_vmax",
        "machines_short_of_vmax",
        "negative_v",
        "negative_ax",
        "ggv_over_50",
        "machines_ax_over_20",
        "ggv_columns",
        "machines_columns",
        "version_mismatch",
        "controller_speed_differs",
        "ggv_v_not_increasing",
        "machines_v_not_increasing",
        "missing_supervisor_key",
        "missing_controller_key",
    ],
)
def test_raceline_1240_loader_raises_on_an_inconsistent_vehicle(
    tmp_path, kwargs, cause
):
    from raceline.vehicle import load_vehicle

    kwargs = (
        {"ggv": GGV_OK, "machines": MACHINES_OK, **kwargs}
        if ("edit" not in kwargs)
        else kwargs
    )
    path = _copy_vehicle(tmp_path, **kwargs)

    with pytest.raises(ValueError, match=cause):
        load_vehicle(path)


def test_raceline_1240_loader_raises_on_a_missing_table_directory(tmp_path):
    from raceline.vehicle import load_vehicle

    path = _copy_vehicle(tmp_path)
    shutil.rmtree(tmp_path / "vehicle")

    with pytest.raises(ValueError, match="ggv.csv"):
        load_vehicle(path)


def test_raceline_1240_loader_raises_on_a_missing_vehicle_file(tmp_path):
    from raceline.vehicle import load_vehicle

    with pytest.raises(ValueError, match="no_such.yaml"):
        load_vehicle(tmp_path / "no_such.yaml")
