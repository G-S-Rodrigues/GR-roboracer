"""Tier 1: the ``python -m raceline`` runner, with TUM stubbed."""

import csv
import hashlib
from pathlib import Path

import cv2
import numpy as np
import raceline.__main__ as runner
import test_extract
import test_smoke
import test_verify
import yaml
from raceline.track import Track, write_track

from raceline import optimize

VEHICLES = Path(__file__).resolve().parents[2] / "config" / "vehicles"
DEFAULT = str(VEHICLES / "f1tenth_default.yaml")
OPTIMIZER_PY = Path(optimize.__file__)
D9_KEYS = {
    "format_version",
    "track",
    "vehicle_version",
    "optimizer",
    "source",
    "ground_truth",
    "vehicle",
    "code",
    "tum",
    "parameters",
    "verdict",
    "command",
}


def _feasible_tum(calls=None, shift=0.0):
    """A run_tum stand-in: the line is the file's own centerline."""

    def fake(track_csv, inputs_dir, curv_opt_type, safety_width, timeout=None):
        if calls is not None:
            calls.append(Path(track_csv))
        xy = np.loadtxt(track_csv, delimiter=",")[:, :2] + [shift, 0.0]
        line = test_verify._line_from_xy(xy, v=3.0)
        rows = np.column_stack(
            [line.s, line.x, line.y, line.psi - np.pi / 2, line.kappa]
            + [line.v, line.a]
        )
        closing = rows[:1].copy()
        closing[0, 0] = line.s[-1] + np.hypot(*(xy[0] - xy[-1]))
        return np.vstack([rows, closing])

    return fake


def _oval_yaml(tmp_path) -> Path:
    xy = test_smoke._oval()
    n = len(xy)
    track = Track(
        "oval",
        xy,
        np.zeros(n),
        np.full(n, test_smoke.W_LEFT),
        np.full(n, test_smoke.W_RIGHT),
    )
    return write_track(track, tmp_path / "oval.yaml")


def _write_map(grid, tmp_path, stem="ring") -> Path:
    px = np.full(grid.data.shape, 128, dtype=np.uint8)
    px[grid.data == 0] = 255
    px[grid.data == 100] = 0
    cv2.imwrite(str(tmp_path / f"{stem}.png"), px[::-1])
    path = tmp_path / f"{stem}.yaml"
    path.write_text(
        f"image: {stem}.png\nresolution: {grid.resolution}\n"
        f"origin: [{grid.origin.x}, {grid.origin.y}, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )
    return path


def _argv(tmp_path, *extra):
    return [
        "--vehicle",
        DEFAULT,
        "--optimizer",
        "tum_mincurv_iqp",
        "--margin",
        "0.1",
        "--out-dir",
        str(tmp_path / "out"),
        *extra,
    ]


def _blob(path: Path) -> str:
    data = path.read_bytes()
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def test_raceline_1385_a_track_run_writes_the_csv_and_provenance(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum())
    oval = _oval_yaml(tmp_path)
    code = runner.main(_argv(tmp_path, "--track", str(oval)))
    assert code == 0
    folder = tmp_path / "out" / "oval" / "f1tenth-default-v1"
    with (folder / "tum_mincurv_iqp.csv").open() as f:
        rows = list(csv.reader(f, delimiter=";"))
    assert len(rows) == len(test_smoke._oval())
    assert all(len(r) == 7 for r in rows)
    assert float(rows[0][0]) == 0.0
    doc = yaml.safe_load((folder / "tum_mincurv_iqp.yaml").read_text())
    assert set(doc) >= D9_KEYS
    digest = hashlib.sha256(oval.read_bytes()).hexdigest()
    assert doc["source"]["sha256"] == digest
    assert doc["code"]["raceline/optimize.py"] == _blob(OPTIMIZER_PY)
    assert doc["parameters"]["margin"] == 0.1
    assert doc["verdict"]["ok"] is True


def _ring_args(tmp_path):
    grid = test_extract._ring()
    start = test_extract._ring_start()
    return _write_map(grid, tmp_path), [
        "--start-pose",
        str(start.x),
        str(start.y),
        str(start.yaw),
        "--direction",
        "ccw",
    ]


def test_raceline_1387_walls_reach_the_extractor_and_the_yaml(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum())
    seen = []
    for name in ("extract_track", "track_bounds"):
        real = getattr(runner, name)

        def recorder(*a, _real=real, _name=name, **k):
            seen.append((_name, k.get("walls")))
            return _real(*a, **k)

        monkeypatch.setattr(runner, name, recorder)
    ring, pose = _ring_args(tmp_path)
    args = _argv(tmp_path, "--map", str(ring), *pose)
    assert runner.main([*args, "--walls", "non_free"]) == 0
    assert seen == [("extract_track", "non_free"), ("track_bounds", "non_free")]
    doc = yaml.safe_load(
        (
            tmp_path
            / "out"
            / "ring"
            / "f1tenth-default-v1"
            / "tum_mincurv_iqp.yaml"
        ).read_text()
    )
    assert doc["parameters"]["walls"] == "non_free"
    assert doc["verdict"]["unobserved_share"]["input"] == 0.0
    seen.clear()
    assert runner.main(args) == 0
    assert seen == [("extract_track", "observed"), ("track_bounds", "observed")]


def test_raceline_1392_an_unobserved_map_is_refused_with_exit_3(
    monkeypatch, tmp_path, capsys
):
    calls = []
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum(calls))
    grid = test_extract._ring(xmax=5.6)
    ring = _write_map(grid, tmp_path)
    start = test_extract._ring_start()
    args = _argv(
        tmp_path,
        "--map",
        str(ring),
        "--start-pose",
        str(start.x),
        str(start.y),
        str(start.yaw),
        "--direction",
        "ccw",
    )
    assert runner.main(args) == 3
    err = capsys.readouterr().err
    assert "input:" in err and "unobserved" in err
    assert not (tmp_path / "out").exists()
    assert calls == []


def test_raceline_1390_a_line_the_verifier_rejects_is_not_written(
    monkeypatch, tmp_path, capsys
):
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum(shift=20.0))
    oval = _oval_yaml(tmp_path)
    assert runner.main(_argv(tmp_path, "--track", str(oval))) == 2
    assert "outside" in capsys.readouterr().err
    assert not (tmp_path / "out").exists()
