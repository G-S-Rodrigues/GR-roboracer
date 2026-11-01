"""Guards on `scripts/autodrive_route.sh`, the Windows-to-`dev` route with
TCP_NODELAY on every hop (part 8: Nagle on the socat hop took 48 ms per
reply, 4.5 ms with `nodelay`). The sockets themselves are a live check; here
a stub `docker`, `socat` and `ss` first on `PATH` log their argv; the relay's
two TCP_NODELAY calls are checked in-process against a local upstream.
"""

from __future__ import annotations

import asyncio
import importlib.util
import os
import signal
import socket
import stat
import subprocess
import time
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
SOCAT = """#!/bin/bash
echo "socat $*" >> "$STUB_LOG"
[ -n "$STUB_SOCAT_FAIL" ] && exit 1
exec -a socat sleep 30
"""
SS = """#!/bin/sh
echo "ss $*" >> "$STUB_LOG"
[ -n "$STUB_SS" ] && echo "$STUB_SS"
exit 0
"""


def _alive(pid: int) -> bool:
    """Running, not a zombie (a container's PID 1 may never reap one)."""
    try:
        state = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1]
    except OSError:
        return False
    return state.split()[0] != "Z"


def _run(tmp_path: Path, name: str, *args: str, **extra: str):
    """Run the script; the pidfile is shared by every call in `tmp_path`, so
    a `stop` finds the socat a `start` left."""
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
        "AUTODRIVE_ROUTE_PID": str(tmp_path / "route.pid"),
        **extra,
    }
    began = time.monotonic()
    done = subprocess.run(
        ["bash", str(SCRIPT), *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    done.elapsed = time.monotonic() - began
    lines = log.read_text().splitlines() if log.exists() else []
    return done, lines


def test_check_2100_both_hops_set_nodelay(tmp_path) -> None:
    """CHECK-2100: the script passes `bash -n`; a start runs the relay
    container publishing 127.0.0.1:4567 with the server's address and
    NODELAY=1, and socat on the WSL IP with `nodelay` on both addresses; the
    start returns in under 5 s (socat keeps no
    caller pipe)."""
    assert SCRIPT.is_file(), "scripts/autodrive_route.sh missing"
    syntax = subprocess.run(
        ["bash", "-n", str(SCRIPT)], capture_output=True, text=True
    )
    assert syntax.returncode == 0, syntax.stderr
    done, calls = _run(tmp_path, "start")
    try:
        assert done.returncode == 0, done.stderr
        # socat must not hold the caller's stdout/stderr: a capturing caller
        # (an agent's shell, `$(...)`) would wait for it to exit.
        assert done.elapsed < 5, f"start took {done.elapsed:.1f} s"
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


def test_check_2110_taken_port_and_missing_dev_are_refused(tmp_path) -> None:
    """CHECK-2110: a container publishing 4567, or any listener on it, gives
    exit 3 and no `docker run`; a dev container with no address gives exit 4
    and no `docker run`."""
    assert SCRIPT.is_file(), "scripts/autodrive_route.sh missing"
    for key in ("STUB_PS", "STUB_SS"):
        done, calls = _run(tmp_path, key, **{key: "x"})
        assert done.returncode == 3, (key, done.stderr)
        assert not any(c.startswith("docker run") for c in calls), calls
        # Both probes must look at 4567, not some other port.
        assert any(
            c.startswith("docker ps") and "publish=4567" in c for c in calls
        ), calls
        if key == "STUB_SS":  # `docker ps` found nothing, so `ss` ran
            assert any(c.startswith("ss ") and ":4567" in c for c in calls), (
                calls
            )
    done, calls = _run(tmp_path, "nodev", STUB_INSPECT="")
    assert done.returncode == 4, done.stderr
    assert not any(c.startswith("docker run") for c in calls), calls


def test_check_2120_stop_and_failed_socat_clean_up(tmp_path) -> None:
    """CHECK-2120: `stop` kills the socat a `start` left, removes the
    pidfile and stops the relay container; a socat that dies at once gives
    exit 1, stops the relay and leaves no pidfile; `stop` does not kill a
    pidfile PID that is no longer a socat (the PID was reused)."""
    done, _ = _run(tmp_path, "start")
    assert done.returncode == 0, done.stderr
    pid = int((tmp_path / "route.pid").read_text())
    assert _alive(pid)
    done, calls = _run(tmp_path, "stop", "stop")
    assert done.returncode == 0, done.stderr
    assert "docker stop autodrive-route-relay" in calls, calls
    assert not (tmp_path / "route.pid").exists()
    for _ in range(50):
        if not _alive(pid):
            break
        time.sleep(0.1)
    else:
        os.kill(pid, signal.SIGKILL)
        raise AssertionError("stop left socat running")
    done, calls = _run(tmp_path, "fail", STUB_SOCAT_FAIL="1")
    assert done.returncode == 1, done.stderr
    assert "did not stay up" in done.stderr
    assert "docker stop autodrive-route-relay" in calls, calls
    assert not (tmp_path / "route.pid").exists(), "dead PID left behind"
    other = subprocess.Popen(["sleep", "30"])
    try:
        (tmp_path / "route.pid").write_text(f"{other.pid}\n")
        done, _ = _run(tmp_path, "reuse", "stop")
        assert done.returncode == 0, done.stderr
        time.sleep(0.5)
        assert other.poll() is None, "stop killed a PID that is not socat"
        assert not (tmp_path / "route.pid").exists()
    finally:
        other.kill()
        other.wait()


def test_check_2130_relay_sets_nodelay_on_both_legs(
    tmp_path, monkeypatch
) -> None:
    """CHECK-2130: the relay calls `_nodelay` on the client leg and on the
    upstream leg, and each call leaves TCP_NODELAY set. The wrapper clears
    the option first because asyncio sets it by default, which would hide a
    removed call from a bare `getsockopt`."""

    async def go() -> list:
        async def upstream(reader, writer):
            await reader.read(1)
            writer.close()

        up = await asyncio.start_server(upstream, "127.0.0.1", 0)
        up_port = up.sockets[0].getsockname()[1]
        monkeypatch.setenv("UP_HOST", "127.0.0.1")
        monkeypatch.setenv("UP_PORT", str(up_port))
        monkeypatch.setenv("NODELAY", "1")
        spec = importlib.util.spec_from_file_location("relay_t", RELAY)
        relay = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(relay)
        seen: list = []
        original = relay._nodelay

        def traced(writer) -> None:
            sock = writer.get_extra_info("socket")
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0)
            original(writer)
            seen.append(
                (
                    writer.get_extra_info("peername")[1] == up_port,
                    sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY),
                )
            )

        async def no_pipe(reader, writer) -> None:
            writer.close()

        monkeypatch.setattr(relay, "_nodelay", traced)
        monkeypatch.setattr(relay, "_pipe", no_pipe)
        srv = await asyncio.start_server(relay._handle, "127.0.0.1", 0)
        port = srv.sockets[0].getsockname()[1]
        _, w = await asyncio.open_connection("127.0.0.1", port)
        await asyncio.sleep(0.5)
        w.close()
        srv.close()
        up.close()
        return seen

    seen = asyncio.run(go())
    assert sorted(seen) == [(False, 1), (True, 1)], seen
