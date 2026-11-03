"""Tier 1: the seam to the fork's minimum-time solver, with a fake env."""

import json
from pathlib import Path

import numpy as np

from raceline import tum

OPTIONS = {
    "mu_init": 1e-3,
    "max_iter": 3036,
    "max_wall_time_s": 1200,
    "threads": 3,
}
STATUS = {"status": "Solve_Succeeded", "iterations": 176, "wall_time_s": 3.0}
NOT_CONVERGED = {
    "status": "Maximum_Iterations_Exceeded",
    "iterations": 3036,
    "state_dir": None,
}


class FakeEnv:
    """Stands in for ``tum._tum_env_call`` and plays run_tum_mintime.py."""

    def __init__(self, tmp_path, fail=None, returncode=0):
        self.marker = tmp_path / "fake-ran"
        self.calls = []
        self.fail = fail
        self.returncode = returncode
        self.trajectory = np.arange(14, dtype=float).reshape(2, 7)

    def __call__(self, script, args, timeout, env=None):
        self.marker.write_text("ran")
        self.calls.append((script, [str(a) for a in args], timeout, env))
        out_csv = Path(args[6])
        export = Path(args[4])
        if self.fail:
            line = "MINTIME_NOT_CONVERGED " + json.dumps(self.fail) + "\n"
            return 1, line, "log"
        if self.returncode:
            return self.returncode, "", "Traceback: boom"
        (export / "mintime").mkdir(parents=True)
        (export / "mintime" / "solver_status.json").write_text(
            json.dumps(STATUS)
        )
        np.savetxt(out_csv, self.trajectory, delimiter=";")
        return 0, "", ""


def _run(monkeypatch, tmp_path, **kwargs):
    fake = FakeEnv(tmp_path, **kwargs)
    monkeypatch.setattr(tum, "_tum_env_call", fake)
    inputs = tmp_path / "inputs"
    seed = tmp_path / "seed.csv"
    result = None
    error = None
    try:
        result = tum.run_tum_mintime(
            tmp_path / "t.csv", inputs, 1.1, seed, dict(OPTIONS), timeout=77
        )
    except Exception as exc:  # noqa: BLE001 - the tests inspect it
        error = exc
    return fake, result, error


def test_raceline_1430_the_fake_env_really_runs(monkeypatch, tmp_path):
    fake, _, _ = _run(monkeypatch, tmp_path)

    assert fake.marker.read_text() == "ran"
    assert len(fake.calls) == 1


def test_raceline_1430_returns_the_trajectory_and_the_solver_status(
    monkeypatch, tmp_path
):
    fake, result, error = _run(monkeypatch, tmp_path)

    assert error is None
    traj, status = result
    assert np.array_equal(traj, fake.trajectory)
    assert status == STATUS


def test_raceline_1430_builds_argv_options_and_environment(
    monkeypatch, tmp_path
):
    fake, _, _ = _run(monkeypatch, tmp_path)

    script, args, timeout, env = fake.calls[0]
    assert script == "run_tum_mintime.py"
    assert timeout == 77
    track, inputs, width, seed, export, options_json, out_csv = args
    assert (track, inputs, width, seed) == (
        str(tmp_path / "t.csv"),
        str(tmp_path / "inputs"),
        "1.1",
        str(tmp_path / "seed.csv"),
    )
    options = json.loads(options_json)
    assert options == {
        "mu_init": 1e-3,
        "max_iter": 3036,
        "max_wall_time_s": 1200,
        "exact_arc_length": True,
    }
    assert env["OPENBLAS_NUM_THREADS"] == "3"
    assert env["OMP_NUM_THREADS"] == "3"
    assert env["MKL_NUM_THREADS"] == "3"


def test_raceline_1430_exports_into_a_temp_dir_never_the_package(
    monkeypatch, tmp_path
):
    fake, _, _ = _run(monkeypatch, tmp_path)

    export = Path(fake.calls[0][1][4])
    assert tum._TUM_ENV not in export.parents
    assert not export.exists()  # cleaned up: a temp dir, not a kept one


def test_raceline_1430_not_converged_raises_naming_status_and_iterations(
    monkeypatch, tmp_path
):
    _, result, error = _run(monkeypatch, tmp_path, fail=NOT_CONVERGED)

    assert result is None
    assert isinstance(error, tum.MintimeFailed)
    assert error.status == "Maximum_Iterations_Exceeded"
    assert error.iterations == 3036
    assert "Maximum_Iterations_Exceeded" in str(error)
    assert "3036" in str(error)


def test_raceline_1430_any_other_failure_carries_stderr(monkeypatch, tmp_path):
    _, _, error = _run(monkeypatch, tmp_path, returncode=2)

    assert isinstance(error, RuntimeError)
    assert not isinstance(error, tum.MintimeFailed)
    assert "Traceback: boom" in str(error)
