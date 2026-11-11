"""Guards on the shared heavy-run lock: the compose mount and the wrapper.

The wrapper is run for real against a temporary lane directory; what the
compose file resolves to is not checked (CI never runs compose), only its text.
"""

from __future__ import annotations

import os
import shutil
import signal
import stat
import subprocess
import tempfile
import time
from collections.abc import Iterator
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "heavy_lane.sh"
COMPOSE = REPO_ROOT / "docker" / "docker-compose.yaml"
HOST_LANE = "/tmp/gr-roboracer-lane"


@pytest.fixture
def lane(tmp_path: Path) -> Path:
    """A lane the wrapper uses instead of the host's (GR_LANE_DIR wins over
    /lane, so this runs inside a mounted container too)."""
    return tmp_path


OTHER_UID = 65534


def _as(uid: int | None, argv: list[str]) -> list[str]:
    """argv, run as `uid` when that is set and this test is root."""
    if uid is None or os.geteuid() != 0:
        return argv
    return [
        "setpriv",
        f"--reuid={uid}",
        f"--regid={uid}",
        "--clear-groups",
        *argv,
    ]


def _run(
    lane: Path,
    *args: str,
    as_uid: int | None = None,
    script: Path = SCRIPT,
    **kw,
) -> subprocess.CompletedProcess:
    env = {**os.environ, "GR_LANE_DIR": str(lane)}
    env.pop("HEAVY_LANE_HELD", None)
    if as_uid is not None and os.geteuid() == 0:
        kw.setdefault("cwd", "/")
    return subprocess.run(
        _as(as_uid, ["bash", str(script), *args]),
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
    holder_env = {**os.environ, "GR_LANE_DIR": str(lane)}
    holder_env.pop("HEAVY_LANE_HELD", None)
    holder = subprocess.Popen(
        ["bash", str(SCRIPT), "--max-load", "1000", "--", "sleep", "20"],
        env=holder_env,
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


def test_check_2440_a_container_without_the_mount_exits_4_not_a_private_lock(
    tmp_path: Path,
) -> None:
    """CHECK-2440: with no GR_LANE_DIR, no lane mount and the container
    marker present, the wrapper exits 4 and does not run the command; with
    the marker absent (the host) it does not take that exit."""
    marker = tmp_path / "dockerenv"
    marker.write_text("")
    env = {
        **os.environ,
        "GR_LANE_MOUNT": str(tmp_path / "no-lane"),
        "GR_CONTAINER_MARKER": str(marker),
        "HOME": str(tmp_path),
    }
    env.pop("GR_LANE_DIR", None)
    env.pop("HEAVY_LANE_HELD", None)
    cmd = ["bash", str(SCRIPT), "--wait", "1", "--max-load", "1000", "--"]
    out = subprocess.run(
        [*cmd, "echo", "ran"],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert out.returncode == 4
    assert "ran" not in out.stdout and "recreate" in out.stderr


class _Field:
    """A lane as compose leaves it: made by one user, used by another.

    The base is 0755 under /tmp so uid 65534 can reach the script copy and the
    lane (tmp_path's parents are 0700). When this test is not root, the same
    modes (dir 0555, files 0444) make the lane unwritable to it too.
    """

    def __init__(self, base: Path) -> None:
        self.base = base
        self.script = base / "heavy_lane.sh"
        self.script.write_bytes(SCRIPT.read_bytes())
        self.script.chmod(0o755)
        self.lane = base / "lane"
        self.mark = base / "mark"
        self.mark.mkdir()
        self.mark.chmod(0o1777)

    def make(
        self,
        *,
        lock: int | None,
        owner: int | None,
        mode: int | None = None,
    ) -> Path:
        root = os.geteuid() == 0
        self.lane.mkdir()
        if lock is not None:
            (self.lane / "heavy.lock").touch()
            (self.lane / "heavy.lock").chmod(lock)
        if owner is not None:
            (self.lane / "owner").write_text("root-made stamp\n")
            (self.lane / "owner").chmod(owner)
        self.lane.chmod(
            mode if mode is not None else (0o755 if root else 0o555)
        )
        return self.lane


@pytest.fixture
def field() -> Iterator[_Field]:
    base = Path(tempfile.mkdtemp(dir="/tmp"))
    base.chmod(0o755)
    f = _Field(base)
    yield f
    if f.lane.exists():
        f.lane.chmod(0o755)
    shutil.rmtree(base, ignore_errors=True)


def _side() -> tuple[int | None, int | None]:
    """The two owners that meet on one lane: root and uid 65534 under root,
    the one user twice otherwise."""
    return (None, OTHER_UID) if os.geteuid() == 0 else (None, None)


def test_check_2441_a_lane_the_runner_cannot_write_still_takes_the_lock(
    field: _Field,
) -> None:
    """CHECK-2441: compose makes the lane root 0755 and the host user cannot
    write in it. heavy.lock is readable, so the wrapper runs the command
    (exit 0) and says once that the owner stamp could not be written; it does
    not exit 4."""
    lane = field.make(lock=0o444, owner=0o444, mode=0o555)
    out = _run(
        lane,
        "--wait",
        "1",
        "--max-load",
        "1000",
        "--",
        "echo",
        "ran",
        as_uid=OTHER_UID,
        script=field.script,
    )
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == "ran"
    assert out.stderr.count("cannot write") == 1


def test_check_2442_the_lock_is_shared_across_owners_in_an_unwritable_lane(
    field: _Field,
) -> None:
    """CHECK-2442: in a lane the waiter cannot write, a holder on one side
    makes the other side's --wait 1 exit 3, both ways round; once the holder
    is dead the other side takes the lane. flock ignores uids, so what this
    guards is the read-only fd. Readiness is a marker the holder's command
    writes, never `owner`."""
    lane = field.make(lock=0o644, owner=0o644)
    a, b = _side()
    for holder_uid, waiter_uid in ((a, b), (b, a)):
        held = field.mark / f"held-{holder_uid}"
        env = {**os.environ, "GR_LANE_DIR": str(lane)}
        env.pop("HEAVY_LANE_HELD", None)
        holder = subprocess.Popen(
            _as(
                holder_uid,
                [
                    "bash",
                    str(field.script),
                    "--max-load",
                    "1000",
                    "--",
                    "bash",
                    "-c",
                    f"touch {held}; sleep 20",
                ],
            ),
            env=env,
            cwd="/",
            start_new_session=True,
        )
        try:
            for _ in range(100):
                if held.exists():
                    break
                time.sleep(0.1)
            assert held.exists(), "the holder never took the lane"
            blocked = _run(
                lane,
                "--wait",
                "1",
                "--max-load",
                "1000",
                "--",
                "true",
                as_uid=waiter_uid,
                script=field.script,
            )
            assert blocked.returncode == 3, blocked.stderr
        finally:
            os.killpg(holder.pid, signal.SIGKILL)
            holder.wait()
        freed = _run(
            lane,
            "--wait",
            "5",
            "--max-load",
            "1000",
            "--",
            "true",
            as_uid=waiter_uid,
            script=field.script,
        )
        assert freed.returncode == 0, freed.stderr


def test_check_2443_a_container_run_opens_the_lane_for_the_host(
    field: _Field,
) -> None:
    """CHECK-2443: a root run on the /lane mount leaves the dir 1777 and
    heavy.lock and owner 0666, so a uid-65534 run stamps its own owner. Then
    the reverse order: with fs.protected_regular a plain `>` by root on a
    65534-owned owner in the sticky dir is refused, so the stamp is written in
    place and still names the root run."""
    assert os.geteuid() == 0, (
        "CHECK-2443 needs root to make a second uid: run it in dev"
    )
    lane = field.make(lock=0o644, owner=0o644)
    env = {**os.environ, "GR_LANE_MOUNT": str(lane)}
    env.pop("GR_LANE_DIR", None)
    env.pop("HEAVY_LANE_HELD", None)

    def go(uid: int | None, word: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            _as(
                uid,
                [
                    "bash",
                    str(field.script),
                    "--wait",
                    "5",
                    "--max-load",
                    "1000",
                    "--",
                    "echo",
                    word,
                ],
            ),
            env=env,
            cwd="/",
            capture_output=True,
            text=True,
            timeout=60,
        )

    assert go(None, "first-root").returncode == 0
    assert stat.S_IMODE(lane.stat().st_mode) == 0o1777
    assert stat.S_IMODE((lane / "heavy.lock").stat().st_mode) == 0o666
    assert stat.S_IMODE((lane / "owner").stat().st_mode) == 0o666
    other = go(OTHER_UID, "second-uid")
    assert other.returncode == 0 and "cannot write" not in other.stderr
    assert "second-uid" in (lane / "owner").read_text()
    (lane / "owner").unlink()
    assert go(OTHER_UID, "third-uid").returncode == 0
    assert (lane / "owner").stat().st_uid == OTHER_UID
    again = go(None, "fourth-root")
    assert again.returncode == 0 and "cannot write" not in again.stderr
    assert "fourth-root" in (lane / "owner").read_text()


def test_check_2444_an_unwritable_lane_with_no_lock_is_the_one_exit_4(
    field: _Field,
) -> None:
    """CHECK-2444: nothing to read and nowhere to create the lock, so the
    wrapper exits 4, does not run the command, and names the repair."""
    lane = field.make(lock=None, owner=None, mode=0o555)
    out = _run(
        lane,
        "--wait",
        "1",
        "--max-load",
        "1000",
        "--",
        "echo",
        "ran",
        as_uid=OTHER_UID,
        script=field.script,
    )
    assert out.returncode == 4
    assert "ran" not in out.stdout and "chmod 1777" in out.stderr
