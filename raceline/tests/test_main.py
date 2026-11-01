"""Tier 1: the ``python -m raceline`` runner, with TUM stubbed."""

import csv
import hashlib
from pathlib import Path

import cv2
import numpy as np
import pytest
import raceline.__main__ as runner
import test_extract
import test_smoke
import test_verify
import yaml
from raceline.track import Track, write_track

from raceline import diagnose, optimize

VEHICLES = Path(__file__).resolve().parents[2] / "config" / "vehicles"
DEFAULT = str(VEHICLES / "f1tenth_default.yaml")
OPTIMIZER_PY = Path(optimize.__file__)
REPO_ROOT = Path(__file__).resolve().parents[2]
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


REAL_PLOT_WRITE = diagnose.write
PNGS = diagnose.PNG_NAMES


@pytest.fixture(autouse=True)
def _no_plots(monkeypatch):
    """The runner plots by default; these tests are about the rest, and a
    plot costs seconds. The plot tests put the real writer back."""
    calls = []
    monkeypatch.setattr(
        runner.diagnose, "write", lambda *a, **k: calls.append(a) or []
    )
    return calls


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


def test_raceline_1500_provenance_records_the_runner_itself(
    monkeypatch, tmp_path
):
    """N5: __main__.py builds the command and the parameters, so a change to
    it must show up as code drift in the provenance."""
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum())
    oval = _oval_yaml(tmp_path)
    assert runner.main(_argv(tmp_path, "--track", str(oval))) == 0
    folder = tmp_path / "out" / "oval" / "f1tenth-default-v1"
    doc = yaml.safe_load((folder / "tum_mincurv_iqp.yaml").read_text())
    assert doc["code"]["raceline/__main__.py"] == _blob(Path(runner.__file__))


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


@pytest.mark.parametrize("kind", ["malformed", "figure_eight"])
def test_raceline_1392_a_bad_track_input_exits_3_too(
    monkeypatch, tmp_path, capsys, kind
):
    calls = []
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum(calls))
    if kind == "malformed":
        bad = tmp_path / "bad.yaml"
        bad.write_text("format_version: 2\n")
    else:
        bad = write_track(test_verify._figure_eight(), tmp_path / "bad.yaml")
    assert runner.main(_argv(tmp_path, "--track", str(bad))) == 3
    assert "input:" in capsys.readouterr().err
    assert not (tmp_path / "out").exists()
    assert calls == []


def test_raceline_1388_the_ground_truth_map_gives_cells_to_that_set_only(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum())
    recorded = []
    real = runner.verify.verify

    def recorder(line, vehicle, walls, reference):
        recorded.extend(walls)
        return real(line, vehicle, walls, reference)

    monkeypatch.setattr(runner.verify, "verify", recorder)
    ring, _ = _ring_args(tmp_path)
    grid = test_extract._ring()
    oval = _oval_yaml(tmp_path)
    args = _argv(tmp_path, "--track", str(oval))
    assert runner.main([*args, "--ground-truth-map", str(ring)]) == 0
    by_name = {w.name: w for w, _ in recorded}
    assert by_name["input"].cells is None
    cells = by_name["ground_truth"].cells
    assert cells.resolution == grid.resolution
    assert len(cells.centres) == int((grid.data == 100).sum())
    folder = tmp_path / "out" / "oval" / "f1tenth-default-v1"
    doc = yaml.safe_load((folder / "tum_mincurv_iqp.yaml").read_text())
    gt_map = doc["ground_truth_map"]
    assert gt_map["path"] == str(ring)
    assert gt_map["sha256"] == hashlib.sha256(ring.read_bytes()).hexdigest()
    png = ring.with_suffix(".png")
    assert (
        gt_map["image_sha256"] == hashlib.sha256(png.read_bytes()).hexdigest()
    )
    assert doc["verdict"]["tips"]["ground_truth"] == []


def test_raceline_1388_a_ground_truth_map_needs_ground_truth_walls(
    monkeypatch, tmp_path, capsys
):
    calls = []
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum(calls))
    ring, pose = _ring_args(tmp_path)
    args = _argv(tmp_path, "--map", str(ring), *pose)
    assert runner.main([*args, "--ground-truth-map", str(ring)]) == 3
    err = capsys.readouterr().err
    assert "input:" in err and "--ground-truth-map" in err
    assert not (tmp_path / "out").exists()
    assert calls == []


# -- plots (RACELINE-1470) ---------------------------------------------------


def _plotted(directory: Path) -> bool:
    return all((directory / name).is_file() for name in (*PNGS, "profile.npz"))


def test_raceline_1470_a_written_line_is_plotted_to_the_plot_dir(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(runner.diagnose, "write", REAL_PLOT_WRITE)
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum())
    plots = tmp_path / "plots"
    oval = _oval_yaml(tmp_path)
    argv = _argv(tmp_path, "--track", str(oval), "--plots", str(plots))
    assert runner.main(argv) == 0
    assert _plotted(plots)
    folder = tmp_path / "out" / "oval" / "f1tenth-default-v1"
    assert sorted(p.name for p in folder.iterdir()) == [
        "tum_mincurv_iqp.csv",
        "tum_mincurv_iqp.yaml",
    ]


def test_raceline_1470_a_rejected_line_is_plotted_and_not_written(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(runner.diagnose, "write", REAL_PLOT_WRITE)
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum(shift=20.0))
    plots = tmp_path / "plots"
    oval = _oval_yaml(tmp_path)
    argv = _argv(tmp_path, "--track", str(oval), "--plots", str(plots))
    assert runner.main(argv) == 2
    assert _plotted(plots)
    assert not (tmp_path / "out").exists()


def test_raceline_1470_the_default_plot_dir_is_the_log_tree(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(runner.diagnose, "write", REAL_PLOT_WRITE)
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum())
    monkeypatch.setattr(runner, "DEFAULT_PLOTS", tmp_path / "log")
    oval = _oval_yaml(tmp_path)
    assert runner.main(_argv(tmp_path, "--track", str(oval))) == 0
    assert _plotted(
        tmp_path / "log" / "oval" / "f1tenth-default-v1" / "tum_mincurv_iqp"
    )
    assert runner.DEFAULT_OUT not in runner.DEFAULT_PLOTS.parents


def test_raceline_1470_no_plots_writes_none(monkeypatch, tmp_path, _no_plots):
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum())
    oval = _oval_yaml(tmp_path)
    argv = _argv(tmp_path, "--track", str(oval), "--no-plots")
    assert runner.main(argv) == 0
    assert _no_plots == []


# -- the min-time optimizer through the CLI (RACELINE-1445, 1450) -------------

SOLVER = {"status": "Solve_Succeeded", "iterations": 7, "wall_time_s": 1.5}


def _feasible_mintime(calls=None):
    """A run_tum_mintime stand-in: the line is the reference track's own."""

    def fake(track_csv, inputs_dir, safety_width, seed_csv, options, timeout):
        if calls is not None:
            calls.append(Path(seed_csv))
        rows = _feasible_tum()(track_csv, inputs_dir, "mintime", safety_width)
        return rows, dict(SOLVER)

    return fake


def _mintime_argv(tmp_path, out, *extra):
    return [
        "--vehicle",
        DEFAULT,
        "--optimizer",
        "tum_mintime",
        "--out-dir",
        str(out),
        *extra,
    ]


def _run_mintime(monkeypatch, tmp_path, out, *extra):
    mincurv_calls, mintime_calls = [], []
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum(mincurv_calls))
    monkeypatch.setattr(
        optimize, "run_tum_mintime", _feasible_mintime(mintime_calls)
    )
    oval = _oval_yaml(tmp_path)
    code = runner.main(
        _mintime_argv(tmp_path, out, "--track", str(oval), *extra)
    )
    assert code == 0
    folder = out / "oval" / "f1tenth-default-v1"
    doc = yaml.safe_load((folder / "tum_mintime.yaml").read_text())
    return folder, doc, mincurv_calls, mintime_calls


def _sha(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def test_raceline_1445_a_mintime_run_writes_line_reference_and_provenance(
    monkeypatch, tmp_path
):
    out = tmp_path / "out"
    folder, doc, mincurv_calls, mintime_calls = _run_mintime(
        monkeypatch, tmp_path, out
    )
    assert (folder / "tum_mintime.csv").is_file()
    reference = folder / "reference" / "tum_mintime.csv"
    assert reference.is_file()
    # default margin of the optimizer, one solve, the min-curv stage first
    assert doc["parameters"]["margin"] == 0.40
    assert doc["parameters"]["passes"] == 1
    assert len(mincurv_calls) == 2 and len(mintime_calls) == 1
    ref = doc["parameters"]["reference"]
    assert ref["sha256"] == _sha(reference)
    assert ref["margin"] == 0.40
    assert ref["passes"] == 2
    assert "[GENERAL_OPTIONS]" in ref["ini"]
    assert "optim_opts_mintime" not in ref["ini"]
    assert "optim_opts_mintime" in doc["parameters"]["ini"]
    assert doc["parameters"]["tyre_set"] == "gym_fit"
    assert doc["parameters"]["solver_options"]["max_iter"] == 3036
    assert doc["solver"] == {
        "status": "Solve_Succeeded",
        "iterations": 7,
        "wall_s": 1.5,
    }
    vehicle = VEHICLES / "f1tenth_default"
    assert doc["vehicle"]["dynamics_sha256"] == _sha(
        vehicle / "tum_mintime.yaml"
    )
    assert doc["vehicle"]["tyres_sha256"] == _sha(
        vehicle / "tum_mintime_tyres_gym_fit.yaml"
    )
    assert doc["verdict"]["ok"] is True
    # inside the repo the path is repo-relative, else as written
    assert doc["parameters"]["reference"]["path"] == str(reference)


def test_raceline_1445_a_reference_run_records_the_same_parameters(
    monkeypatch, tmp_path
):
    first, doc1, _, _ = _run_mintime(monkeypatch, tmp_path, tmp_path / "a")
    reference = first / "reference" / "tum_mintime.csv"
    second, doc2, mincurv_calls, mintime_calls = _run_mintime(
        monkeypatch, tmp_path, tmp_path / "b", "--reference", str(reference)
    )
    assert doc2["parameters"] == doc1["parameters"]
    assert doc2["vehicle"] == doc1["vehicle"]
    assert mincurv_calls == []  # stages 1-2 skipped
    assert mintime_calls == [reference]  # seeded from the file as given
    assert not (second / "reference").exists()
    # the command differs only by --reference (CHECK-1560 drops it)
    assert doc1["command"] != doc2["command"]
    assert "--reference" in doc2["command"]
    assert "--reference" not in doc1["command"]


def test_raceline_1445_the_repo_relative_reference_path(monkeypatch, tmp_path):
    out = REPO_ROOT / "log" / "pytest-1445" / "out"
    try:
        folder, doc, _, _ = _run_mintime(monkeypatch, tmp_path, out)
        assert doc["parameters"]["reference"]["path"] == (
            "log/pytest-1445/out/oval/f1tenth-default-v1/reference/"
            "tum_mintime.csv"
        )
    finally:
        import shutil

        shutil.rmtree(REPO_ROOT / "log" / "pytest-1445", ignore_errors=True)


def test_raceline_1445_a_mincurv_yaml_is_unchanged_by_the_new_optimizer(
    monkeypatch, tmp_path
):
    monkeypatch.setattr(optimize, "run_tum", _feasible_tum())
    oval = _oval_yaml(tmp_path)
    argv = [
        "--vehicle",
        DEFAULT,
        "--optimizer",
        "tum_mincurv_iqp",
        "--out-dir",
        str(tmp_path / "out"),
        "--track",
        str(oval),
    ]
    assert runner.main(argv) == 0
    folder = tmp_path / "out" / "oval" / "f1tenth-default-v1"
    doc = yaml.safe_load((folder / "tum_mincurv_iqp.yaml").read_text())
    assert set(doc) == D9_KEYS | {"ground_truth_map"}
    assert doc["parameters"]["margin"] == 0.30  # no --margin: today's default
    assert doc["parameters"]["passes"] == 2
    assert set(doc["vehicle"]) == {
        "path",
        "sha256",
        "ggv_sha256",
        "ax_max_machines_sha256",
    }
    assert "reference" not in doc["parameters"]
    assert not (folder / "reference").exists()


def test_raceline_1445_reference_is_refused_with_mincurv(monkeypatch, tmp_path):
    oval = _oval_yaml(tmp_path)
    ref = tmp_path / "ref.csv"
    ref.write_text("")
    with pytest.raises(SystemExit, match="tum_mintime"):
        runner.main(
            _argv(tmp_path, "--track", str(oval), "--reference", str(ref))
        )


def test_raceline_1445_mintime_is_one_solve(monkeypatch, tmp_path):
    oval = _oval_yaml(tmp_path)
    with pytest.raises(SystemExit, match="one solve"):
        runner.main(
            _mintime_argv(
                tmp_path, tmp_path / "o", "--track", str(oval), "--passes", "2"
            )
        )


def test_raceline_1450_the_code_files_cover_the_min_time_optimizer(
    monkeypatch, tmp_path
):
    for name in (
        "tum_env/run_tum_mintime.py",
        "tum_env/pyproject.toml",
        "tum_env/uv.lock",
        "tum_env/run_tum.py",
        "raceline/tum.py",
        "raceline/optimize.py",
    ):
        assert name in runner.CODE_FILES
        assert name in runner.MINTIME_CODE_FILES
        assert (runner.PACKAGE / name).is_file(), name
    # E-3: every file that shapes the min-time inputs counts
    assert tuple(runner.MINTIME_CODE_FILES) == tuple(runner.CODE_FILES)
    _, doc, _, _ = _run_mintime(monkeypatch, tmp_path, tmp_path / "out")
    for name in runner.MINTIME_CODE_FILES:
        assert doc["code"][name] == _blob(runner.PACKAGE / name), name
