"""Seam to ForzaETH's TUM trajectory optimizer.

TUM's optimizer cannot run on this project's Python 3.12 / numpy 2 (see
README.md), so it runs in the isolated Python 3.8 project ``tum_env/`` as a
subprocess, exchanging CSV files.
"""

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
        # PYTHONPATH is unset so the ROS environment never reaches TUM's env.
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
            str(_TUM_ENV / "run_tum.py"),
            str(track_csv),
            str(inputs_dir),
            curv_opt_type,
            str(safety_width),
            str(out_csv),
        ]
        # Own process group, so a timeout kills uv's child python as well.
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        try:
            _, stderr = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.communicate()
            raise
        if proc.returncode != 0:
            raise RuntimeError(
                f"TUM optimizer exited {proc.returncode}:\n{stderr}"
            )
        return np.loadtxt(out_csv, delimiter=";", ndmin=2)
