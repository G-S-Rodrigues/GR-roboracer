"""Guards on `scripts/autodrive_qualify.sh`, the host wrapper of the live
Porto qualification run (SIM-3730, #44 part 10). The sim, compose and the run
itself are a live check; here a stub `docker` first on `PATH` logs its argv.
"""

from __future__ import annotations

import os
import stat
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "autodrive_qualify.sh"

DOCKER = """#!/bin/sh
echo "docker $*" >> "$STUB_LOG"
case "$1" in
  ps) [ -n "$STUB_PS" ] && echo "$STUB_PS"; exit 0 ;;
esac
exit "${STUB_EXIT:-0}"
"""


def _run(tmp_path: Path, name: str, *args: str, **extra: str):
    stub = tmp_path / "docker"
    stub.write_text(DOCKER)
    stub.chmod(stub.stat().st_mode | stat.S_IXUSR)
    log = tmp_path / f"{name}.log"
    env = {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "STUB_LOG": str(log),
        "STUB_PS": "gr-roboracer-dev",
        **extra,
    }
    done = subprocess.run(
        ["bash", str(SCRIPT), *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    lines = log.read_text().splitlines() if log.exists() else []
    return done, lines


def test_check_2140_dry_run_prints_the_three_commands(tmp_path) -> None:
    """CHECK-2140: the script passes `bash -n`; `--dry-run` runs nothing but
    `docker ps`, and prints (1) the compose line that recreates only
    `autodrive-sim` with the bridge host set to `gr-roboracer-dev`, (2) the
    `docker exec` that runs SIM-3730 on the given domain in the given
    workdir, and (3) the stop of the sim container."""
    assert SCRIPT.is_file(), "scripts/autodrive_qualify.sh missing"
    syntax = subprocess.run(
        ["bash", "-n", str(SCRIPT)], capture_output=True, text=True
    )
    assert syntax.returncode == 0, syntax.stderr
    done, calls = _run(
        tmp_path,
        "dry",
        "--domain",
        "64",
        "--workdir",
        "/ws/.scratch/autodrive-p10",
        "--dry-run",
    )
    assert done.returncode == 0, done.stderr
    out = done.stdout
    for part in (
        "AUTODRIVE_BRIDGE_HOST=gr-roboracer-dev",
        "--profile autodrive up -d --no-deps --force-recreate autodrive-sim",
        "docker exec gr-roboracer-dev bash -lc",
        "cd /ws/.scratch/autodrive-p10",
        "ROS_DOMAIN_ID=64",
        "tests/autodrive/test_sim_3730_porto_qualification.py",
        "docker stop gr-roboracer-autodrive-sim",
    ):
        assert part in out, (part, out)
    assert all(c.startswith("docker ps") for c in calls), calls


def test_check_2150_bad_domain_and_missing_dev_are_refused(tmp_path) -> None:
    """CHECK-2150: no domain, 42, or a domain outside 60-69 gives exit 2; a
    dev container that is not running gives exit 4; neither starts compose."""
    assert SCRIPT.is_file(), "scripts/autodrive_qualify.sh missing"
    for args in ((), ("--domain", "42"), ("--domain", "59"), ("--domain", "x")):
        done, calls = _run(tmp_path, "bad", *args, "--dry-run")
        assert done.returncode == 2, (args, done.stderr)
        assert not any("compose" in c for c in calls), calls
    done, calls = _run(
        tmp_path, "nodev", "--domain", "64", "--dry-run", STUB_PS=""
    )
    assert done.returncode == 4, done.stderr
    assert not any("compose" in c for c in calls), calls
