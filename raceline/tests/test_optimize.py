"""Tier 1: the optimizer orchestration (stubbed TUM) and one real TUM run."""

import configparser
import json
from pathlib import Path

import numpy as np
import pytest
import test_smoke
from raceline.track import Track
from raceline.vehicle import load_vehicle

from raceline import geometry, optimize, verify

VEHICLES = Path(__file__).resolve().parents[2] / "config" / "vehicles"
BOUND_TOL = verify.BOUND_TOL


def _oval_track() -> Track:
    xy = test_smoke._oval()
    n = len(xy)
    return Track(
        "oval",
        xy,
        np.zeros(n),
        np.full(n, test_smoke.W_LEFT),
        np.full(n, test_smoke.W_RIGHT),
    )


def _fabricated_tum(calls, shift=0.0):
    """A run_tum stand-in: the oval centerline as a closed 7-column result.

    ``shift`` moves x by that much, so a pass's result differs from the track.
    """

    def fake(track_csv, inputs_dir, curv_opt_type, safety_width, timeout=None):
        calls.append(
            {
                "track": np.loadtxt(track_csv, delimiter=","),
                "inputs": Path(inputs_dir),
                "ini": (Path(inputs_dir) / "racecar_f110.ini").read_text(),
                "ggv": (
                    Path(inputs_dir) / "veh_dyn_info" / "ggv.csv"
                ).read_bytes(),
                "type": curv_opt_type,
                "safety_width": safety_width,
            }
        )
        xy = test_smoke._oval() + [shift, 0.0]
        n = len(xy)
        tangent = geometry.tangents(xy)
        psi = np.arctan2(tangent[:, 1], tangent[:, 0]) - np.pi / 2
        rows = np.column_stack(
            [
                np.arange(n) * 1.0,
                xy,
                psi,
                np.full(n, 0.1),
                np.full(n, 3.0),
                np.zeros(n),
            ]
        )
        return np.vstack([rows, rows[:1]])

    return fake


def test_raceline_1370_two_passes_feed_the_result_back(monkeypatch):
    calls = []
    monkeypatch.setattr(optimize, "run_tum", _fabricated_tum(calls))
    vehicle = load_vehicle(VEHICLES / "f1tenth_default.yaml")
    track = _oval_track()
    line = optimize.optimize(track, vehicle, "tum_mincurv_iqp")
    assert len(calls) == 2
    want = vehicle.safety_width(optimize.DEFAULT_MARGIN)
    assert [c["safety_width"] for c in calls] == [want, want]
    assert all(c["type"] == "mincurv_iqp" for c in calls)
    # pass 1 reads (x, y, w_right, w_left) of the asymmetric track.
    first = calls[0]["track"]
    assert np.allclose(first[:, :2], track.xy, atol=1e-8)
    assert np.allclose(first[:, 2], test_smoke.W_RIGHT)
    assert np.allclose(first[:, 3], test_smoke.W_LEFT)
    # pass 2 reads pass 1's result without its closing row, with widths
    # measured to the original (raw) walls.
    second = calls[1]["track"]
    assert np.allclose(second[:, :2], test_smoke._oval(), atol=1e-8)
    assert len(second) == len(track.xy)
    assert np.allclose(second[:, 2], test_smoke.W_RIGHT, atol=0.02)
    assert np.allclose(second[:, 3], test_smoke.W_LEFT, atol=0.02)
    # the ini and the table.
    ini = configparser.ConfigParser()
    ini.read_string(calls[0]["ini"])
    veh = json.loads(ini["GENERAL_OPTIONS"]["veh_params"])
    assert veh["v_max"] == vehicle.v_max
    assert veh["width"] == pytest.approx(2 * vehicle.half_width)
    assert veh["curvlim"] == pytest.approx(vehicle.kappa_max)
    assert veh["dragcoeff"] == 0.0
    assert calls[0]["ggv"] == Path(vehicle.ggv_path).read_bytes()
    # the output: no closing row, psi shifted to the ROS convention.
    n = len(track.xy)
    assert len(line.s) == n
    tangent = geometry.tangents(test_smoke._oval())
    want_psi = np.arctan2(tangent[:, 1], tangent[:, 0])
    delta = np.angle(np.exp(1j * (line.psi - want_psi)))
    assert np.abs(delta).max() < 1e-9
    assert np.allclose(line.kappa, 0.1)


def test_raceline_1370_pass_two_reads_pass_ones_line(monkeypatch):
    calls = []
    monkeypatch.setattr(optimize, "run_tum", _fabricated_tum(calls, 0.05))
    vehicle = load_vehicle(VEHICLES / "f1tenth_default.yaml")
    optimize.optimize(_oval_track(), vehicle, "tum_mincurv_iqp")
    first, second = calls[0]["track"], calls[1]["track"]
    assert np.allclose(first[:, :2], test_smoke._oval(), atol=1e-8)
    assert np.allclose(
        second[:, :2], test_smoke._oval() + [0.05, 0.0], atol=1e-8
    )


def test_raceline_1375_an_unknown_optimizer_is_refused(monkeypatch):
    vehicle = load_vehicle(VEHICLES / "f1tenth_default.yaml")
    with pytest.raises(ValueError, match="tum_mincurv_iqp"):
        optimize.optimize(_oval_track(), vehicle, "nope")
    with pytest.raises(ValueError, match="passes"):
        optimize.optimize(_oval_track(), vehicle, "tum_mincurv_iqp", passes=0)


def test_raceline_1380_real_tum_on_the_oval_passes_the_verifier():
    vehicle = load_vehicle(VEHICLES / "f1tenth_default.yaml")
    track = _oval_track()
    margin = 0.1  # the oval is 2.2 m wide
    line = optimize.optimize(
        track, vehicle, "tum_mincurv_iqp", margin=margin, passes=1, timeout=300
    )
    walls = verify.track_walls(track, "input")
    required = vehicle.half_width + margin - verify.INPUT_BOUND_TOL
    verdict = verify.verify(line, vehicle, [(walls, required)], track)
    assert verdict.ok, verdict.failures
    offset = test_smoke._left_offset(line.xy)
    arcs = np.abs(line.x) > test_smoke.A_HALF
    assert offset[arcs].mean() > test_smoke.MIN_ARC_CUT


# --- RACELINE-1415: the min-curv ini reads mass and length from the file ----

RACELINES = VEHICLES.parent / "racelines" / "spielberg"


@pytest.mark.parametrize(
    ("stem", "version"),
    [
        ("f1tenth_default", "f1tenth-default-v1"),
        ("f1tenth_gym_limits", "f1tenth-gym-limits-v1"),
    ],
)
def test_raceline_1415_min_curv_ini_is_byte_identical_to_the_sidecar(
    stem, version
):
    import yaml

    sidecar = yaml.safe_load(
        (RACELINES / version / "tum_mincurv_iqp.yaml").read_text()
    )
    vehicle = load_vehicle(VEHICLES / f"{stem}.yaml")

    assert optimize.tum_ini(vehicle) == sidecar["parameters"]["ini"]


def test_raceline_1415_mass_and_length_come_from_the_dynamics_file(tmp_path):
    import shutil

    import yaml

    shutil.copy(VEHICLES / "f1tenth_gym_limits.yaml", tmp_path / "v.yaml")
    shutil.copytree(VEHICLES / "f1tenth_gym_limits", tmp_path / "v")
    dyn = tmp_path / "v" / "tum_mintime.yaml"
    document = yaml.safe_load(dyn.read_text())
    document.update(mass=3.75, length=0.59)
    dyn.write_text(yaml.safe_dump(document))

    ini = optimize.tum_ini(load_vehicle(tmp_path / "v.yaml"))

    assert '"mass": 3.75' in ini
    assert '"length": 0.59' in ini


# --- RACELINE-1420: the min-time ini, as the fork reads it ------------------

# GR-roboracer-data fixtures/tum-fork/spielberg-gym-0.40/inputs/
# racecar_f110.ini (part 8F, TUMFORK-4030), gym-limits vehicle, margin 0.40,
# tyre set B. Two values differ from that file on purpose: width_opt is
# 1.11 + 2 * 0.01 = 1.13 (E-1 ruling 2026-10-25: width extra 0.01 m per side
# on both vehicles; Step 1.4 measured it clearing the default vehicle's input
# clearance) and nothing else.
GYM_040_OPTIM = {
    "width_opt": 1.13,
    "penalty_delta": 1.0,
    "penalty_F": 0.1,
    "mue": 1.027922,
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
    "step_non_reg": 0,
    "eps_kappa": 0.02,
}
GYM_040_VEHICLE = {
    "wheelbase_front": 0.15875,
    "wheelbase_rear": 0.17145,
    "track_width_front": 0.281,
    "track_width_rear": 0.281,
    "cog_z": 0.074,
    "I_z": 0.04712,
    "liftcoeff_front": 0.0,
    "liftcoeff_rear": 0.0,
    "k_brake_front": 0.5,
    "k_drive_front": 0.0,
    "k_roll": 0.5,
    "t_delta": 0.1,
    "t_drive": 0.1,
    "t_brake": 0.1,
    "power_max": 255.1114885704,
    "f_drive_max": 34.856052,
    "f_brake_max": 34.856052,
    "delta_max": 0.4189,
}
GYM_040_TIRE_B = {
    "c_roll": 0.0,
    "f_z0": 8.6,
    "B_front": 3.9316666666666666,
    "C_front": 1.2,
    "eps_front": -0.1,
    "E_front": 0.85,
    "B_rear": 4.546833333333334,
    "C_rear": 1.2,
    "eps_rear": -0.1,
    "E_rear": 0.85,
}


def _fork_reads(text):
    """The ini as the fork reads it: configparser, then json.loads per key."""
    parser = configparser.ConfigParser()
    parser.optionxform = str
    parser.read_string(text)
    return parser, {
        section: {
            key: json.loads(value) for key, value in parser[section].items()
        }
        for section in parser.sections()
    }


def _assert_close(got, want, where):
    assert set(got) == set(want), where
    for key, value in want.items():
        if isinstance(value, float):
            assert got[key] == pytest.approx(value, abs=1e-9), (where, key)
        else:
            assert got[key] == value, (where, key)


def test_raceline_1420_gym_ini_equals_the_8f_gym_ini():
    vehicle = load_vehicle(VEHICLES / "f1tenth_gym_limits.yaml")

    text = optimize.tum_mintime_ini(vehicle, 0.40, "gym_fit")

    parser, read = _fork_reads(text)
    assert parser.has_option("OPTIMIZATION_OPTIONS", "optim_opts_mintime")
    flat = {**read["OPTIMIZATION_OPTIONS"], **read["GENERAL_OPTIONS"]}
    _assert_close(flat["optim_opts_mintime"], GYM_040_OPTIM, "optim")
    _assert_close(flat["vehicle_params_mintime"], GYM_040_VEHICLE, "vehicle")
    _assert_close(flat["tire_params_mintime"], GYM_040_TIRE_B, "tire")
    assert flat["pwr_params_mintime"] == {"pwr_behavior": False}
    assert flat["curv_calc_opts"] == {
        "d_preview_curv": 2.0,
        "d_review_curv": 2.0,
        "d_preview_head": 1.0,
        "d_review_head": 1.0,
    }
    assert flat["stepsize_opts"]["stepsize_reg"] == 0.5
    assert flat["stepsize_opts"]["stepsize_interp_after_opt"] == 0.1


def test_raceline_1420_the_sections_sit_where_the_fork_reads_them():
    vehicle = load_vehicle(VEHICLES / "f1tenth_gym_limits.yaml")

    parser, _ = _fork_reads(optimize.tum_mintime_ini(vehicle, 0.40, "gym_fit"))

    # fork trajectory_optimizer.py:196-209 reads these through
    # parser.get(<section>, <key>) with two sections only.
    assert parser.sections() == ["GENERAL_OPTIONS", "OPTIMIZATION_OPTIONS"]
    assert parser.has_option("GENERAL_OPTIONS", "curv_calc_opts")
    for key in (
        "optim_opts_mintime",
        "vehicle_params_mintime",
        "tire_params_mintime",
        "pwr_params_mintime",
    ):
        assert parser.has_option("OPTIMIZATION_OPTIONS", key)


def test_raceline_1420_default_vehicle_values_are_derived_from_its_tables():
    vehicle = load_vehicle(VEHICLES / "f1tenth_default.yaml")
    ggv = np.loadtxt(vehicle.ggv_path, delimiter=",", comments="#")
    machines = np.loadtxt(
        vehicle.ax_max_machines_path, delimiter=",", comments="#"
    )
    scale = 0.98
    mass = 3.74

    _, read = _fork_reads(optimize.tum_mintime_ini(vehicle, 0.40, "gym_fit"))

    optim = read["OPTIMIZATION_OPTIONS"]["optim_opts_mintime"]
    veh = read["OPTIMIZATION_OPTIONS"]["vehicle_params_mintime"]
    # width: safety_width(0.40) = 2 * (0.15 + 0.40) = 1.10, plus 2 * 0.01.
    assert optim["width_opt"] == pytest.approx(1.12, abs=1e-12)
    assert optim["mue"] == pytest.approx(ggv[0, 2] / 9.81 * scale, abs=1e-9)
    assert veh["f_drive_max"] == pytest.approx(
        mass * min(ggv[0, 1], machines[0, 1]) * scale, abs=1e-9
    )
    assert veh["f_brake_max"] == pytest.approx(
        mass * ggv[0, 1] * scale, abs=1e-9
    )
    assert veh["power_max"] == pytest.approx(
        mass * np.max(machines[:, 0] * machines[:, 1]) * scale, abs=1e-9
    )
    assert veh["delta_max"] == 0.4
    assert read["GENERAL_OPTIONS"]["veh_params"]["width"] == pytest.approx(0.30)


def test_raceline_1420_tyre_set_selects_its_values_and_extra_moves_the_width():
    vehicle = load_vehicle(VEHICLES / "f1tenth_gym_limits.yaml")

    _, a = _fork_reads(optimize.tum_mintime_ini(vehicle, 0.40, "forzaeth_f110"))
    _, b = _fork_reads(
        optimize.tum_mintime_ini(vehicle, 0.40, "gym_fit", extra=0.0)
    )

    tyre_a = a["OPTIMIZATION_OPTIONS"]["tire_params_mintime"]
    tyre_b = b["OPTIMIZATION_OPTIONS"]["tire_params_mintime"]
    assert (tyre_a["B_front"], tyre_a["B_rear"]) == (7.4, 7.4)
    assert tyre_b["B_front"] == pytest.approx(4.718 / 1.2, abs=1e-12)
    assert b["OPTIMIZATION_OPTIONS"]["optim_opts_mintime"][
        "width_opt"
    ] == pytest.approx(1.11, abs=1e-12)
    with pytest.raises(ValueError, match="gym_fit"):
        optimize.tum_mintime_ini(vehicle, 0.40, "nope")


# --- RACELINE-1440, 1425, 1435: the min-time chain --------------------------

# Golden: today's between-pass widths on the oval, computed before the
# extract by running the code that was at optimize.py (_tum_mincurv_iqp,
# the `nearest_distance(xy, left, STEP_M)` block) on the oval with the line
# shifted +0.05 in x: left/right = verify.offsets(track), 60 nodes.
GOLDEN_IDX = (0, 7, 20, 45)
GOLDEN_W_LEFT = (
    1.5987096250982555,
    1.6006811945755761,
    1.6545731024872443,
    1.5703498459766303,
)
GOLDEN_W_RIGHT = (
    0.6000641242580672,
    0.6011622636148851,
    0.5498789288258393,
    0.6262647174000601,
)


def test_raceline_1440_reference_from_line_keeps_the_between_pass_widths():
    track = _oval_track()
    xy = test_smoke._oval() + [0.05, 0.0]

    reference = optimize._reference_from_line("oval-pass1", xy, track)

    for i, left, right in zip(
        GOLDEN_IDX, GOLDEN_W_LEFT, GOLDEN_W_RIGHT, strict=True
    ):
        assert reference.w_left[i] == pytest.approx(left, abs=1e-9)
        assert reference.w_right[i] == pytest.approx(right, abs=1e-9)
    assert reference.w_left.sum() == pytest.approx(96.10543918677993, abs=1e-9)
    assert reference.w_right.sum() == pytest.approx(35.89151485416709, abs=1e-9)
    assert reference.name == "oval-pass1"
    assert np.array_equal(reference.xy, xy)


SHIFT = 0.123456789012345  # more decimals than the 9 a CSV keeps


def _fake_mintime(calls, status=None):
    """A run_tum_mintime stand-in that records what it was handed."""

    def fake(track_csv, inputs_dir, safety_width, seed_csv, options, timeout):
        calls.append(
            {
                "track": np.loadtxt(track_csv, delimiter=","),
                "ini": (Path(inputs_dir) / "racecar_f110.ini").read_text(),
                "safety_width": safety_width,
                "seed_path": Path(seed_csv),
                "seed": np.loadtxt(seed_csv, delimiter=";"),
                "options": dict(options),
                "timeout": timeout,
            }
        )
        rows = np.loadtxt(seed_csv, delimiter=";")
        rows = np.vstack([rows, rows[:1]])
        rows[:, 5] += 1.0  # a faster line than the reference
        return rows, status or {"status": "Solve_Succeeded", "iterations": 5}

    return fake


def _chain(monkeypatch, tmp_path, **kwargs):
    mincurv, mintime, loads = [], [], []
    monkeypatch.setattr(optimize, "run_tum", _fabricated_tum(mincurv, SHIFT))
    monkeypatch.setattr(optimize, "run_tum_mintime", _fake_mintime(mintime))
    real_load = optimize.load_raceline_csv

    def spy(path):
        loads.append(Path(path))
        return real_load(path)

    monkeypatch.setattr(optimize, "load_raceline_csv", spy)
    vehicle = load_vehicle(VEHICLES / "f1tenth_gym_limits.yaml")
    out = tmp_path / "reference" / "ref.csv"
    kwargs.setdefault("reference_out", out)
    result = optimize.mintime_chain(
        _oval_track(), vehicle, 0.40, 55.0, **kwargs
    )
    return vehicle, out, mincurv, mintime, loads, result


def test_raceline_1425_chain_runs_mincurv_twice_then_mintime_once(
    monkeypatch, tmp_path
):
    vehicle, out, mincurv, mintime, loads, (line, status) = _chain(
        monkeypatch, tmp_path
    )

    assert len(mincurv) == 2
    want = vehicle.safety_width(0.40)
    assert [c["safety_width"] for c in mincurv] == [want, want]
    assert len(mintime) == 1
    call = mintime[0]
    assert call["safety_width"] == want  # width_opt carries the extra
    assert call["timeout"] == 55.0
    assert status == {"status": "Solve_Succeeded", "iterations": 5}
    assert isinstance(line, optimize.Raceline)
    assert len(line.s) == len(_oval_track().xy)  # closing row dropped
    assert np.allclose(line.v, 4.0)  # the fake's +1 on the reference's 3.0


def test_raceline_1425_reference_is_written_and_read_back_before_use(
    monkeypatch, tmp_path
):
    _, out, mincurv, mintime, loads, _ = _chain(monkeypatch, tmp_path)

    call = mintime[0]
    assert out.is_file()
    assert call["seed_path"] == out
    assert loads == [out]  # read back from disk, once
    written = np.loadtxt(out, delimiter=";")
    assert np.array_equal(call["seed"], written)
    # the 9-decimal file, not the solver's floats, is what the chain used.
    raw = test_smoke._oval()[:, 0] + SHIFT
    assert not np.array_equal(written[:, 1], raw)
    assert np.allclose(written[:, 1], raw, atol=1e-9)
    # the min-time track is built from the read-back line.
    want = optimize._reference_from_line("x", written[:, 1:3], _oval_track())
    assert np.allclose(call["track"][:, :2], written[:, 1:3], atol=1e-8)
    assert np.allclose(call["track"][:, 2], want.w_right, atol=1e-8)
    assert np.allclose(call["track"][:, 3], want.w_left, atol=1e-8)


def test_raceline_1425_options_and_ini_reach_the_solver(monkeypatch, tmp_path):
    _, _, _, mintime, _, _ = _chain(monkeypatch, tmp_path)

    call = mintime[0]
    assert call["options"] == {
        "mu_init": optimize.MINTIME_MU_INIT,
        "max_iter": optimize.MINTIME_MAX_ITER,
        "max_wall_time_s": optimize.MINTIME_MAX_WALL_S,
        "threads": optimize.MINTIME_THREADS,
    }
    assert call["options"]["mu_init"] == 1e-3
    assert "optim_opts_mintime" in call["ini"]


def test_raceline_1425_a_given_reference_skips_the_mincurv_stages(
    monkeypatch, tmp_path
):
    given = tmp_path / "given.csv"
    xy = test_smoke._oval()
    n = len(xy)
    rows = np.column_stack(
        [
            np.arange(n) * 1.0,
            xy,
            np.zeros(n),
            np.full(n, 0.1),
            np.full(n, 3.0),
            np.zeros(n),
        ]
    )
    optimize.write_raceline_csv(optimize.Raceline(*rows.T), given)

    _, out, mincurv, mintime, loads, _ = _chain(
        monkeypatch, tmp_path, reference_csv=given, reference_out=None
    )

    assert mincurv == []
    assert len(mintime) == 1
    assert mintime[0]["seed_path"] == given
    assert np.array_equal(mintime[0]["seed"], np.loadtxt(given, delimiter=";"))


def test_raceline_1425_optimize_resolves_defaults_and_refuses_passes(
    monkeypatch,
):
    mincurv, mintime = [], []
    monkeypatch.setattr(optimize, "run_tum", _fabricated_tum(mincurv))
    monkeypatch.setattr(optimize, "run_tum_mintime", _fake_mintime(mintime))
    vehicle = load_vehicle(VEHICLES / "f1tenth_gym_limits.yaml")

    line = optimize.optimize(_oval_track(), vehicle, "tum_mintime")

    assert len(mincurv) == 2 and len(mintime) == 1
    assert mincurv[0]["safety_width"] == vehicle.safety_width(0.40)
    assert isinstance(line, optimize.Raceline)
    with pytest.raises(ValueError, match="passes"):
        optimize.optimize(_oval_track(), vehicle, "tum_mintime", passes=2)
    with pytest.raises(ValueError, match="reference"):
        optimize.optimize(
            _oval_track(), vehicle, "tum_mincurv_iqp", reference=Path("x.csv")
        )
    assert optimize.DEFAULT_PASSES == {"tum_mincurv_iqp": 2, "tum_mintime": 1}
    assert optimize.DEFAULT_MARGINS == {
        "tum_mincurv_iqp": 0.30,
        "tum_mintime": 0.40,
    }


# Measured 0.195 m on the oval (gym vehicle, margin 0.40); the nearest-point
# comparison itself resolves ~0.05 m at TUM's 0.1 m spacing.
MIN_DEPARTURE = 0.1


def _closure_error(line):
    """Interior arc length against the chord: max |ds - |dxy||.

    The closing row (where TUM's exact-arc-length fix acts) is dropped from a
    ``Raceline``, so the wrap is judged by the verifier's closing-gap check.
    """
    chord = np.hypot(*np.diff(line.xy, axis=0).T)
    return float(np.abs(np.diff(line.s) - chord).max())


def test_raceline_1435_real_chain_on_the_oval_passes_the_verifier(tmp_path):
    vehicle = load_vehicle(VEHICLES / "f1tenth_gym_limits.yaml")
    track = _oval_track()
    margin = optimize.DEFAULT_MARGINS["tum_mintime"]

    line, status = optimize.mintime_chain(
        track, vehicle, margin, 300, reference_out=tmp_path / "reference.csv"
    )
    reference = optimize.load_raceline_csv(tmp_path / "reference.csv")
    assert status["status"] in ("Solve_Succeeded", "Optimal")
    print(f"solver: {status['status']}, {status['iterations']} iterations")

    walls = verify.track_walls(track, "input")
    required = vehicle.half_width + margin - verify.INPUT_BOUND_TOL
    verdict = verify.verify(line, vehicle, [(walls, required)], track)
    assert verdict.ok, verdict.failures
    closure = _closure_error(line)
    print(f"closure {closure:.3e}")
    assert closure < 1e-4
    # It is a different line from its seed, not the seed handed back. The
    # two have different lengths (exact arc length), so compare by nearest
    # point, not by index.
    nearest = np.hypot(
        line.x[:, None] - reference.x[None, :],
        line.y[:, None] - reference.y[None, :],
    ).min(axis=1)
    print(f"max nearest-point gap {nearest.max():.4f} m")
    print(f"v: seed {reference.v.mean():.3f}, line {line.v.mean():.3f}")
    assert nearest.max() > MIN_DEPARTURE
