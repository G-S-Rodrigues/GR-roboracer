"""Guards on the shared heavy-run lock: the compose mount and the wrapper.

The wrapper is run for real against a temporary lane directory; what the
compose file resolves to is not checked (CI never runs compose), only its text.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "heavy_lane.sh"
COMPOSE = REPO_ROOT / "docker" / "docker-compose.yaml"
HOST_LANE = "/tmp/gr-roboracer-lane"


@pytest.fixture
def lane(tmp_path: Path) -> Path:
    """A lane the wrapper uses instead of the host's. Where /lane exists (a
    container started from the mounted compose) the wrapper prefers it, so
    these tests skip there and run on the host and in CI."""
    if Path("/lane").is_dir():
        pytest.skip("/lane is mounted: the wrapper would use it, not tmp_path")
    return tmp_path


def _run(lane: Path, *args: str, **kw) -> subprocess.CompletedProcess:
    env = {**os.environ, "GR_LANE_DIR": str(lane)}
    env.pop("HEAVY_LANE_HELD", None)
    return subprocess.run(
        ["bash", str(SCRIPT), *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
        **kw,
    )


def test_check_2400_dev_and_fast_sim_mount_the_one_host_lane() -> None:
    """CHECK-2400: `dev` and `fast-sim` both bind the same host directory at
    /lane, the path scripts/heavy_lane.sh looks for in a container."""
    services = yaml.safe_load(COMPOSE.read_text())["services"]
    for name in ("dev", "fast-sim"):
        assert f"{HOST_LANE}:/lane" in services[name]["volumes"], name


def test_check_2410_the_wrapper_runs_the_command_and_returns_its_exit(
    lane: Path,
) -> None:
    """CHECK-2410: the command runs after `--` and its exit status is the
    wrapper's, 0 and non-zero alike; the owner file names the run."""
    ok = _run(lane, "--max-load", "1000", "--", "bash", "-c", "echo hi")
    assert ok.returncode == 0 and ok.stdout.strip() == "hi"
    assert "bash -c echo hi" in (lane / "owner").read_text()
    bad = _run(lane, "--max-load", "1000", "--", "bash", "-c", "exit 7")
    assert bad.returncode == 7


def test_check_2420_a_second_holder_waits_then_gives_up(lane: Path) -> None:
    """CHECK-2420: while one run holds the lane, a second with --wait 1 exits 3
    and names the holder; once the first ends, the second runs."""
    holder = subprocess.Popen(
        ["bash", str(SCRIPT), "--max-load", "1000", "--", "sleep", "20"],
        env={**os.environ, "GR_LANE_DIR": str(lane)},
        start_new_session=True,
    )
    try:
        for _ in range(50):
            if (lane / "owner").exists():
                break
            time.sleep(0.1)
        blocked = _run(lane, "--wait", "1", "--max-load", "1000", "--", "true")
        assert blocked.returncode == 3
        assert "sleep 20" in blocked.stderr
    finally:
        # The whole group: a child that outlives the wrapper keeps the lane.
        os.killpg(holder.pid, signal.SIGKILL)
        holder.wait()
    freed = _run(lane, "--wait", "5", "--max-load", "1000", "--", "true")
    assert freed.returncode == 0


def test_check_2430_a_nested_call_does_not_deadlock_on_its_parent(
    lane: Path,
) -> None:
    """CHECK-2430: the wrapper inside a wrapped run (HEAVY_LANE_HELD set by the
    outer one) runs the command at once instead of waiting on the lock its own
    parent holds."""
    inner = f"bash {SCRIPT} --wait 2 --max-load 1000 -- echo nested"
    out = _run(lane, "--max-load", "1000", "--", "bash", "-c", inner)
    assert out.returncode == 0 and out.stdout.strip() == "nested"
