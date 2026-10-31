"""Seam to ForzaETH's TUM trajectory optimizer.

TUM's optimizer cannot run on this project's Python 3.12 / numpy 2 (see
README.md), so it runs in the isolated Python 3.8 project ``tum_env/`` as a
subprocess, exchanging CSV files.
"""

import json
import os
import signal
import subprocess
import tempfile
from pathlib import Path

import numpy as np

_TUM_ENV = Path(__file__).resolve().parents[1] / "tum_env"


def write_tum_track(
    xy: np.ndarray, w_right: np.ndarray, w_left: np.ndarray, path: Path
) -> Path:
    """Write TUM's track format: x_m, y_m, w_tr_right_m, w_tr_left_m."""
    xy = np.asarray(xy, dtype=float)
    n = len(xy)
    w_right = np.broadcast_to(np.asarray(w_right, dtype=float), (n,))
    w_left = np.broadcast_to(np.asarray(w_left, dtype=float), (n,))
    data = np.column_stack([xy[:, 0], xy[:, 1], w_right, w_left])
    path = Path(path)
    np.savetxt(
        path,
        data,
        delimiter=",",
        header="x_m,y_m,w_tr_right_m,w_tr_left_m",
        fmt="%.9f",
    )
    return path


def _tum_env_call(script, args, timeout, env=None):
    """Run ``script`` in the py3.8 ``tum_env`` project; return what it said.

    Returns ``(returncode, stdout, stderr)``. PYTHONPATH is unset so the ROS
    environment never reaches TUM's env; ``env`` adds variables (thread
    counts). On ``timeout`` (seconds) the whole process group is killed and
    ``subprocess.TimeoutExpired`` is raised.
    """
    cmd = [
        "env",
        "-u",
        "PYTHONPATH",
        "uv",
        "run",
        "--project",
        str(_TUM_ENV),
        "--locked",
        "python",
        str(_TUM_ENV / script),
        *[str(arg) for arg in args],
    ]
    # Own process group, so a timeout kills uv's child python as well.
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
        env={**os.environ, **(env or {})},
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.communicate()
        raise
    return proc.returncode, stdout, stderr


def run_tum(
    track_csv: Path,
    inputs_dir: Path,
    curv_opt_type: str,
    safety_width: float,
    timeout: float | None = None,
) -> np.ndarray:
    """Return [s, x, y, psi, kappa, vx, ax], closed, from TUM.

    ``timeout`` is in seconds and unset by default: a Spielberg run takes
    ~450 s, so the caller sets the bound. On expiry the whole process group is
    killed and ``subprocess.TimeoutExpired`` is raised. A non-zero exit raises
    ``RuntimeError`` carrying TUM's stderr.

    ``psi`` is TUM's heading: 0 = north (+y), from ``arctan2(...) - pi/2``
    (tph ``calc_head_curv_an``). The gym's yaw is 0 along +x. Convert before
    writing a gym ``<map>_raceline.csv``; the gym reads that column as yaw.
    """
    with tempfile.TemporaryDirectory() as tmp:
        out_csv = Path(tmp) / "traj.csv"
        returncode, _, stderr = _tum_env_call(
            "run_tum.py",
            [track_csv, inputs_dir, curv_opt_type, safety_width, out_csv],
            timeout,
        )
        if returncode != 0:
            raise RuntimeError(f"TUM optimizer exited {returncode}:\n{stderr}")
        return np.loadtxt(out_csv, delimiter=";", ndmin=2)


class MintimeFailed(RuntimeError):
    """The fork's solver stopped short of an optimum (not converged)."""

    def __init__(self, status, iterations):
        super().__init__(
            f"minimum-time solve did not converge: status {status!r} "
            f"after {iterations} iterations"
        )
        self.status = status
        self.iterations = iterations


def run_tum_mintime(
    track_csv: Path,
    inputs_dir: Path,
    safety_width: float,
    seed_csv: Path,
    options: dict,
    timeout: float | None = None,
):
    """Solve minimum time seeded from ``seed_csv``; return ``(traj, status)``.

    ``traj`` is the closed 7-column trajectory, ``status`` the fork's own
    ``solver_status.json``. ``options`` holds the fork's ``MintimeOptions``
    fields (``mu_init``, ``max_iter``, ``max_wall_time_s``) and ``threads``,
    which sets the BLAS thread counts of the child; ``exact_arc_length`` is
    always on (Q11). The export directory is a temp dir, never the package's.
    ``timeout`` is in seconds, as for ``run_tum``. A solve that stops short of
    an optimum raises ``MintimeFailed``; any other failure ``RuntimeError``.
    """
    fields = dict(options)
    threads = fields.pop("threads", None)
    fields["exact_arc_length"] = True
    env = None
    if threads is not None:
        env = {
            name: str(threads)
            for name in (
                "OPENBLAS_NUM_THREADS",
                "OMP_NUM_THREADS",
                "MKL_NUM_THREADS",
            )
        }
    with tempfile.TemporaryDirectory() as tmp:
        export_dir = Path(tmp) / "export"
        out_csv = Path(tmp) / "traj.csv"
        returncode, stdout, stderr = _tum_env_call(
            "run_tum_mintime.py",
            [
                track_csv,
                inputs_dir,
                safety_width,
                seed_csv,
                export_dir,
                json.dumps(fields),
                out_csv,
            ],
            timeout,
            env,
        )
        if returncode != 0:
            for line in stdout.splitlines():
                if line.startswith("MINTIME_NOT_CONVERGED "):
                    report = json.loads(line.split(" ", 1)[1])
                    raise MintimeFailed(report["status"], report["iterations"])
            raise RuntimeError(
                f"TUM minimum-time optimizer exited {returncode}:\n{stderr}"
            )
        status = json.loads(
            (export_dir / "mintime" / "solver_status.json").read_text()
        )
        return np.loadtxt(out_csv, delimiter=";", ndmin=2), status
