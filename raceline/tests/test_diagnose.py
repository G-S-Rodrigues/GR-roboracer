"""Tier 1: the diagnostic plots (RACELINE-1465)."""

from pathlib import Path

import numpy as np
import pytest
import test_verify
import yaml
from raceline.optimize import load_raceline_csv

from raceline import diagnose, verify

REPO = Path(__file__).resolve().parents[2]
COMMITTED = REPO / "config" / "racelines" / "spielberg"


def _write(tmp_path, line=None, track=None):
    track = track or test_verify._annulus()
    vehicle = test_verify._vehicle()
    pairs = test_verify._pairs(track, vehicle)
    line = line or test_verify._circle(test_verify.R_TRACK)
    out = tmp_path / "plots"
    paths = diagnose.write(line, vehicle, pairs, track, out)
    return line, vehicle, pairs, out, paths


def test_raceline_1465_writes_the_five_pngs_and_the_profile(tmp_path):
    *_, out, paths = _write(tmp_path)
    for name in (*diagnose.PNG_NAMES, "profile.npz"):
        assert (out / name).is_file(), name
        assert (out / name).stat().st_size > 0, name
    assert {p.name for p in paths} == {*diagnose.PNG_NAMES, "profile.npz"}
    for name in diagnose.PNG_NAMES:
        assert (out / name).read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_raceline_1465_the_profile_is_the_verifiers(tmp_path):
    line, vehicle, pairs, out, _ = _write(tmp_path)
    saved = np.load(out / "profile.npz")
    expected = verify.profile(line, vehicle, pairs)
    assert set(expected) <= set(saved.files)
    for key, value in expected.items():
        np.testing.assert_array_equal(saved[key], value, err_msg=key)
    # and the ratios are the ones the verdict reports
    verdict = verify.verify(line, vehicle, pairs, test_verify._annulus())
    assert saved["ay_ratio"].max() == pytest.approx(verdict.max_ay_ratio)
    assert saved["ax_ratio"].max() == pytest.approx(verdict.max_ax_ratio)
    assert saved["clearance_input"].min() == pytest.approx(
        verdict.min_clearance["input"]
    )


def test_raceline_1465_wall_binding_follows_the_corrected_corridor(tmp_path):
    """The bound is wall - width_opt / 2, i.e. required + the input tolerance
    from the wall: a line riding it binds everywhere, one mid-track nowhere."""
    vehicle = test_verify._vehicle()
    required = verify.required_clearances(vehicle, test_verify.MARGIN)
    base = required["input"] + verify.INPUT_BOUND_TOL
    assert base == pytest.approx(vehicle.half_width + test_verify.MARGIN)
    riding = test_verify.R_TRACK + test_verify.WIDTH - base
    for radius, expect in ((riding, True), (test_verify.R_TRACK, False)):
        line = test_verify._circle(radius, v=2.0)
        *_, out, _ = _write(tmp_path / f"r{radius:.2f}", line)
        saved = np.load(out / "profile.npz")
        assert bool(saved["binding_wall"].all()) is expect, radius
        assert bool(saved["binding_wall"].any()) is expect, radius


def test_raceline_1465_works_on_a_committed_line_without_a_reference(
    tmp_path,
):
    """No reference line, no TUM run dir: the CSV, the vehicle and a track."""
    rel = "f1tenth-default-v1"
    doc = yaml.safe_load((COMMITTED / rel / "tum_mincurv_iqp.yaml").read_text())
    out = tmp_path / "out"
    code = diagnose.main(
        [
            str(COMMITTED / rel / "tum_mincurv_iqp.csv"),
            "--track",
            str(REPO / doc["ground_truth"]["path"]),
            "--vehicle",
            str(REPO / doc["vehicle"]["path"]),
            "--margin",
            str(doc["parameters"]["margin"]),
            "--out",
            str(out),
        ]
    )
    assert code == 0
    saved = np.load(out / "profile.npz")
    # the line's own numbers, against what the verifier recorded for it
    assert saved["ay_ratio"].max() == pytest.approx(
        doc["verdict"]["max_ay_ratio"], abs=1e-6
    )
    line = load_raceline_csv(COMMITTED / rel / "tum_mincurv_iqp.csv")
    assert len(saved["s"]) == len(line.s)
    for name in diagnose.PNG_NAMES:
        assert (out / name).is_file()


def test_raceline_1465_the_default_out_is_never_under_config_racelines():
    """Plots go to the gitignored log tree; a committed line is data."""
    from raceline.__main__ import DEFAULT_OUT, DEFAULT_PLOTS

    assert DEFAULT_OUT not in (DEFAULT_PLOTS, *DEFAULT_PLOTS.parents)
    assert DEFAULT_PLOTS not in (DEFAULT_OUT, *DEFAULT_OUT.parents)
    assert not str(DEFAULT_PLOTS).startswith(str(DEFAULT_OUT))
