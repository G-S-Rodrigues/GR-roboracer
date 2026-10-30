"""Guards on `scripts/autodrive_route.sh`, the Windows-to-`dev` route with
TCP_NODELAY on every hop (part 8: Nagle on the socat hop took 48 ms per
reply, 4.5 ms with `nodelay`). The sockets themselves are a live check; here
a stub `docker`, `socat`, `ss` and `hostname` first on `PATH` log their argv.
"""

from __future__ import annotations

import os
import stat
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "autodrive_route.sh"
RELAY = REPO_ROOT / "scripts" / "autodrive_relay.py"

DOCKER = """#!/bin/sh
echo "docker $*" >> "$STUB_LOG"
case "$1" in
  ps) [ -n "$STUB_PS" ] && echo "$STUB_PS"; exit 0 ;;
  inspect) echo "$STUB_INSPECT"; exit 0 ;;
esac
exit 0
"""
SOCAT = """#!/bin/sh
echo "socat $*" >> "$STUB_LOG"
exec sleep 30
"""
SS = """#!/bin/sh
[ -n "$STUB_SS" ] && echo "$STUB_SS"
exit 0
"""


def _run(tmp_path: Path, name: str, *args: str, **extra: str):
    for tool, body in (("docker", DOCKER), ("socat", SOCAT), ("ss", SS)):
        stub = tmp_path / tool
        stub.write_text(body)
        stub.chmod(stub.stat().st_mode | stat.S_IXUSR)
    log = tmp_path / f"{name}.log"
    env = {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "STUB_LOG": str(log),
        "STUB_INSPECT": "gr-roboracer:dev gr_default 172.18.0.2",
        "AUTODRIVE_WSL_IP": "172.24.0.9",
        "AUTODRIVE_ROUTE_PID": str(tmp_path / f"{name}.pid"),
        **extra,
    }
    done = subprocess.run(
        ["bash", str(SCRIPT), *args], env=env, capture_output=True, text=True
    )
    lines = log.read_text().splitlines() if log.exists() else []
    return done, lines


def test_check_2100_both_hops_set_nodelay(tmp_path) -> None:
    """CHECK-2100: the script passes `bash -n`; a start runs the relay
    container publishing 127.0.0.1:4567 with the server's address and
    NODELAY=1, and socat on the WSL IP with `nodelay` on both addresses; the
    relay sets TCP_NODELAY on both legs."""
    assert SCRIPT.is_file(), "scripts/autodrive_route.sh missing"
    syntax = subprocess.run(
        ["bash", "-n", str(SCRIPT)], capture_output=True, text=True
    )
    assert syntax.returncode == 0, syntax.stderr
    done, calls = _run(tmp_path, "start")
    try:
        assert done.returncode == 0, done.stderr
        run = [c for c in calls if c.startswith("docker run")]
        assert len(run) == 1, calls
        for part in (
            "-p 127.0.0.1:4567:4567",
            "UP_HOST=172.18.0.2",
            "UP_PORT=4567",
            "NODELAY=1",
            "--network gr_default",
            "gr-roboracer:dev python3 /relay.py",
        ):
            assert part in run[0], run[0]
        socat = [c for c in calls if c.startswith("socat")]
        assert len(socat) == 1, calls
        assert (
            "TCP-LISTEN:4567,bind=172.24.0.9,fork,reuseaddr,nodelay" in socat[0]
        ), socat[0]
        assert "TCP:127.0.0.1:4567,nodelay" in socat[0], socat[0]
        assert "-ip 172.24.0.9 -port 4567" in done.stdout
    finally:
        _run(tmp_path, "stop", "stop")
    assert "TCP_NODELAY" in RELAY.read_text()
    assert "NODELAY" in RELAY.read_text()


def test_check_2110_taken_port_and_missing_dev_are_refused(tmp_path) -> None:
    """CHECK-2110: a container publishing 4567, or any listener on it, gives
    exit 3 and no `docker run`; a dev container with no address gives exit 4
    and no `docker run`."""
    assert SCRIPT.is_file(), "scripts/autodrive_route.sh missing"
    for key in ("STUB_PS", "STUB_SS"):
        done, calls = _run(tmp_path, key, **{key: "x"})
        assert done.returncode == 3, (key, done.stderr)
        assert not any(c.startswith("docker run") for c in calls), calls
    done, calls = _run(tmp_path, "nodev", STUB_INSPECT="")
    assert done.returncode == 4, done.stderr
    assert not any(c.startswith("docker run") for c in calls), calls
