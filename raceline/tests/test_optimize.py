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
    required = vehicle.half_width + margin - BOUND_TOL
    verdict = verify.verify(line, vehicle, [(walls, required)], track)
    assert verdict.ok, verdict.failures
    offset = test_smoke._left_offset(line.xy)
    arcs = np.abs(line.x) > test_smoke.A_HALF
    assert offset[arcs].mean() > test_smoke.MIN_ARC_CUT
