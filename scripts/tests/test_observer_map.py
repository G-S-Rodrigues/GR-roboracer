"""Tests for scripts/observer_map.py (CHECK-2450..2455): the helper that
serves the scenario's map on /map for a video whose run publishes none."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import observer_map  # noqa: E402

PORTO = REPO_ROOT / "config/scenarios/porto.yaml"
SPIELBERG = REPO_ROOT / "config/scenarios/spielberg.yaml"
SCRIPT = REPO_ROOT / "scripts/observer_map.py"


def _map_base(scenario):
    """The scenario's own map directory, wherever the repo keeps it."""
    from racing_sim_gym_jax.scenario import load_scenario

    loaded = load_scenario(scenario)
    return loaded.map_directory / loaded.map_name, loaded.map_name


def _msg(width, height, resolution, origin, frame="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=origin[0], y=origin[1])
            ),
        ),
    )


def test_check_2450_map_yaml_per_pose_source() -> None:
    """CHECK-2450: ground truth serves maps/<M>/<M>.yaml, mola serves
    maps/<M>/mola/<M>_grid.yaml, amcl and slam_toolbox are refused (they
    publish /map), and a missing file raises naming it."""
    base, name = _map_base(PORTO)
    truth = observer_map.map_yaml(PORTO, "ground_truth")
    assert Path(truth) == (base / f"{name}.yaml").resolve()
    base, name = _map_base(SPIELBERG)
    mola = observer_map.map_yaml(SPIELBERG, "mola")
    assert Path(mola) == (base / "mola" / f"{name}_grid.yaml").resolve()
    for source in ("amcl", "slam_toolbox"):
        with pytest.raises(observer_map.Refused, match="publish"):
            observer_map.map_yaml(PORTO, source)
    with pytest.raises(FileNotFoundError, match="Porto"):
        observer_map.map_yaml(PORTO, "mola")


def test_check_2450_missing_map_exits_2_naming_it() -> None:
    """CHECK-2450: the command line exits 2 and names the missing file."""
    done = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--scenario",
            str(PORTO),
            "--pose-source",
            "mola",
            "--dry-run",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert done.returncode == 2
    assert "Porto" in done.stderr + done.stdout


def test_check_2451_dry_run_prints_yaml_and_both_argvs() -> None:
    """CHECK-2451: --dry-run prints the yaml and both argvs, runs nothing."""
    done = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--scenario",
            str(PORTO),
            "--pose-source",
            "ground_truth",
            "--dry-run",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert done.returncode == 0, done.stderr
    out = done.stdout
    assert "Porto/Porto.yaml" in out
    assert "observer_map_server" in out
    assert "use_sim_time:=false" in out
    assert "node_names:=[observer_map_server]" in out
    assert "autostart:=true" in out
    commands = observer_map.argvs("/x/y.yaml")
    assert len(commands) == 2
    assert any("yaml_filename:=/x/y.yaml" in " ".join(c) for c in commands)


def test_check_2452_existing_map_publisher_is_refused() -> None:
    """CHECK-2452: a /map that already has a publisher exits 3, naming
    the nodes; no publisher is fine."""
    assert observer_map.refusal([]) is None
    message = observer_map.refusal(["/map_server", "/slam_toolbox"])
    assert message is not None
    assert "/map_server" in message and "/slam_toolbox" in message
    assert observer_map.EXIT_REFUSED == 3


def test_check_2453_grid_matches_the_yaml() -> None:
    """CHECK-2453: frame, resolution, origin and size are checked against
    the yaml and its image; a Spielberg grid against Porto's yaml fails
    with the reason."""
    base, name = _map_base(PORTO)
    porto = base / f"{name}.yaml"
    base, name = _map_base(SPIELBERG)
    spielberg = base / f"{name}.yaml"
    from racing_evaluation.map_accuracy import load_grid

    grid = load_grid(porto)
    rows, cols = grid.occupied.shape
    good = _msg(cols, rows, grid.resolution, grid.origin)
    assert observer_map.grid_matches(good, porto) == (True, "")
    ok, _ = observer_map.grid_matches(good, porto)
    assert ok
    other = load_grid(spielberg)
    srows, scols = other.occupied.shape
    wrong = _msg(scols, srows, other.resolution, other.origin)
    ok, reason = observer_map.grid_matches(wrong, porto)
    assert not ok and reason
    ok, reason = observer_map.grid_matches(
        _msg(cols, rows, grid.resolution, grid.origin, frame="odom"), porto
    )
    assert not ok and "frame" in reason


def test_check_2454_either_child_exiting_stops_the_other() -> None:
    """CHECK-2454: when one child exits the helper returns non-zero and
    the other is stopped."""
    start = time.monotonic()
    code = observer_map.supervise([["sleep", "30"], ["false"]])
    assert code != 0
    assert time.monotonic() - start < 10
    code = observer_map.supervise([["sleep", "30"], ["sh", "-c", "exit 0"]])
    assert code != 0


def test_check_2455_unknown_pose_source_is_a_usage_error() -> None:
    """CHECK-2455: an unknown --pose-source exits 2 with a usage message,
    not a ValueError traceback."""
    done = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--scenario",
            str(PORTO),
            "--pose-source",
            "bogus",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert done.returncode == 2
    assert "usage" in done.stderr and "invalid choice" in done.stderr
    assert "Traceback" not in done.stderr
