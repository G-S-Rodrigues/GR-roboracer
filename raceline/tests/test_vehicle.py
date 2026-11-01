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


# --- RACELINE-1410: TUM minimum-time dynamics and tyre files (part 8) --------

REFERENCE = VEHICLES / "tum_mintime/reference.yaml"
STEMS = ("f1tenth_default", "f1tenth_gym_limits")
# Every committed vehicle needs a dynamics file (load_vehicle requires it); the
# F1TENTH value pins and the tyre sets stay on STEMS.
DYNAMICS_STEMS = sorted(p.stem for p in VEHICLES.glob("*.yaml"))
TYRE_SETS = ("forzaeth_f110", "gym_fit")
# Values both F1TENTH files hold (not every committed file), from the gym's
# Param @1b4eb3f (the reference file's sources) and the 8F gym ini
# (tire_params_mintime).
DYNAMICS = {
    "mass": 3.74,
    "I_z": 0.04712,
    "wheelbase_front": 0.15875,
    "wheelbase_rear": 0.17145,
    "cog_z": 0.074,
    "length": 0.58,
    "track_width_front": 0.281,
    "track_width_rear": 0.281,
    "k_brake_front": 0.5,
    "k_drive_front": 0.0,
    "k_roll": 0.5,
    "t_delta": 0.1,
    "t_drive": 0.1,
    "t_brake": 0.1,
    "c_roll": 0.0,
    "liftcoeff_front": 0.0,
    "liftcoeff_rear": 0.0,
}
TYRE_B = {
    "forzaeth_f110": (7.4, 7.4),
    "gym_fit": (4.718 / 1.2, 5.4562 / 1.2),
}


def _reference_keys(section):
    return set(yaml.safe_load(REFERENCE.read_text())[section])


def _gaps(runtime: Path, section: str) -> list[str]:
    """Both directions of the key-set check, each naming key and file."""
    have = set(yaml.safe_load(runtime.read_text()))
    want = _reference_keys(section)
    gaps = [
        f"{key!r} is in {runtime.name} but not documented in {REFERENCE.name}"
        for key in sorted(have - want)
    ]
    gaps += [
        f"{key!r} is documented in {REFERENCE.name} but missing from "
        f"{runtime.name}"
        for key in sorted(want - have)
    ]
    return gaps


def _dynamics_files():
    for stem in DYNAMICS_STEMS:
        yield stem, VEHICLES / stem / "tum_mintime.yaml", "dynamics"


def _runtime_files():
    yield from _dynamics_files()
    for stem in STEMS:
        for name in TYRE_SETS:
            yield (
                stem,
                VEHICLES / stem / f"tum_mintime_tyres_{name}.yaml",
                "tyres",
            )


def _assert_exists(stem, runtime):
    assert runtime.is_file(), (
        f"{stem} has no {runtime.relative_to(VEHICLES)}: "
        "every committed vehicle needs one"
    )


@pytest.mark.parametrize("stem", STEMS)
def test_raceline_1410_committed_dynamics_and_tyres_load(stem):
    from raceline.vehicle import TYRE_SETS as KNOWN
    from raceline.vehicle import load_tyres, load_vehicle

    limits = load_vehicle(VEHICLES / f"{stem}.yaml")

    for key, value in DYNAMICS.items():
        assert getattr(limits.dynamics, key) == pytest.approx(value, abs=1e-12)
    assert limits.dynamics_path == VEHICLES / stem / "tum_mintime.yaml"
    assert tuple(KNOWN) == TYRE_SETS
    for name, (b_front, b_rear) in TYRE_B.items():
        tyres = load_tyres(limits, name)
        assert tyres.name == name
        assert tyres.path == VEHICLES / stem / f"tum_mintime_tyres_{name}.yaml"
        assert tyres.f_z0 == 8.6
        assert tyres.B_front == pytest.approx(b_front, abs=1e-12)
        assert tyres.B_rear == pytest.approx(b_rear, abs=1e-12)
        assert (tyres.C_front, tyres.eps_front, tyres.E_front) == (
            1.2,
            -0.1,
            0.85,
        )
        assert (tyres.C_rear, tyres.eps_rear, tyres.E_rear) == (1.2, -0.1, 0.85)


@pytest.mark.parametrize(
    ("stem", "runtime", "section"),
    list(_runtime_files()),
    ids=lambda value: value.name if isinstance(value, Path) else str(value),
)
def test_raceline_1410_runtime_keys_equal_the_reference_keys_both_ways(
    stem, runtime, section
):
    _assert_exists(stem, runtime)
    assert _gaps(runtime, section) == []


def test_raceline_1410_the_key_check_names_key_and_file_in_each_direction(
    tmp_path,
):
    extra = tmp_path / "tum_mintime.yaml"
    document = yaml.safe_load(
        (VEHICLES / "f1tenth_default" / "tum_mintime.yaml").read_text()
    )
    document["undocumented_key"] = 1.0
    del document["mass"]
    extra.write_text(yaml.safe_dump(document))

    gaps = _gaps(extra, "dynamics")

    assert len(gaps) == 2
    assert any(
        "'undocumented_key'" in g and "tum_mintime.yaml" in g for g in gaps
    )
    assert any("'mass'" in g and REFERENCE.name in g for g in gaps)


@pytest.mark.parametrize(
    "runtime",
    [path for _, path, _ in _runtime_files()],
    ids=lambda path: f"{path.parent.name}/{path.name}",
)
def test_raceline_1410_runtime_files_carry_no_comment(runtime):
    _assert_exists(runtime.parent.name, runtime)
    assert "#" not in runtime.read_text()


def test_raceline_1410_reference_keys_are_values_free_with_a_comment():
    lines = [
        line
        for line in REFERENCE.read_text().splitlines()
        if line.startswith("  ") and ":" in line
    ]
    assert len(lines) == len(DYNAMICS) + 9
    document = yaml.safe_load(REFERENCE.read_text())
    for section in ("dynamics", "tyres"):
        for key, value in document[section].items():
            assert value is None, f"{section}.{key} carries a value"
    for line in lines:
        key, _, rest = line.strip().partition(":")
        _, hash_, comment = rest.partition("#")
        assert hash_ and comment.strip(), f"{key!r} has no comment"
        assert rest.partition("#")[0].strip() == "", f"{key!r} has a value"


def _dynamics_copy(tmp_path, edit):
    path = _copy_vehicle(tmp_path)
    runtime = tmp_path / "vehicle" / "tum_mintime.yaml"
    document = yaml.safe_load(runtime.read_text())
    edit(document)
    runtime.write_text(yaml.safe_dump(document))
    return path


@pytest.mark.parametrize(
    ("key", "value"),
    [("mass", 0.0), ("mass", -1.0), ("I_z", 0.0), ("I_z", -0.1)],
)
def test_raceline_1410_loader_raises_on_a_non_positive_mass_or_inertia(
    tmp_path, key, value
):
    from raceline.vehicle import load_vehicle

    path = _dynamics_copy(tmp_path, lambda d: d.update({key: value}))

    with pytest.raises(ValueError, match=key):
        load_vehicle(path)


def test_raceline_1410_loader_raises_when_lf_plus_lr_is_off_the_wheelbase(
    tmp_path,
):
    from raceline.vehicle import check_dynamics, load_vehicle

    path = _dynamics_copy(tmp_path, lambda d: d.update({"wheelbase_rear": 0.2}))

    # Lazy (owner ruling): the loader still loads, the check raises.
    limits = load_vehicle(path)
    with pytest.raises(ValueError, match="wheelbase"):
        check_dynamics(limits)


@pytest.mark.parametrize("stem", DYNAMICS_STEMS)
def test_raceline_1410_committed_dynamics_match_their_wheelbase(stem):
    from raceline.vehicle import check_dynamics, load_vehicle

    _assert_exists(stem, VEHICLES / stem / "tum_mintime.yaml")

    # default: 0.33 against lf + lr 0.3302 (0.2 mm), inside the 1 mm tolerance.
    check_dynamics(load_vehicle(VEHICLES / f"{stem}.yaml"))


def test_raceline_1410_loader_raises_on_a_missing_or_unknown_dynamics_key(
    tmp_path,
):
    from raceline.vehicle import load_vehicle

    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    missing = _dynamics_copy(tmp_path / "a", lambda d: d.pop("k_roll"))
    with pytest.raises(ValueError, match="k_roll"):
        load_vehicle(missing)
    unknown = _dynamics_copy(tmp_path / "b", lambda d: d.update({"zzz": 1.0}))
    with pytest.raises(ValueError, match="zzz"):
        load_vehicle(unknown)


def test_raceline_1410_an_unknown_tyre_set_names_the_known_sets():
    from raceline.vehicle import load_tyres, load_vehicle

    limits = load_vehicle(GYM_LIMITS)

    with pytest.raises(ValueError, match="forzaeth_f110.*gym_fit|gym_fit"):
        load_tyres(limits, "nope")
    with pytest.raises(ValueError, match="nope"):
        load_tyres(limits, "nope")
