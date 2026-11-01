"""Tier 2: SIMAD-2110/2111, the server exits 0 on SIGINT once it is serving.

`ros2 launch` forwards SIGINT to the server and SIGTERM-groups send it again
(SIM-3610's teardown signals the whole group), so the server gets one or two
SIGINTs at any point in its life: during start-up, while a client replays,
idle, and again while it is already tearing down. An unhandled
KeyboardInterrupt kills a Python process with -2 ("process has died"), which
a launch reads as a fault, so every case here must exit 0. (A SIGINT during
the interpreter's own imports, before any of this package's code runs, is not
covered: nothing of ours is there to handle it.)
"""

import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import _server_launch as launch_support  # noqa: E402

LISTEN_TIMEOUT_S = 60.0
EXIT_TIMEOUT_S = 20.0


def _start(tmp_path, port):
    parameters = {
        **launch_support.VEHICLE,
        "world_to_map.x": 0.0,
        "world_to_map.y": 0.0,
        "world_to_map.yaw": 0.0,
        "track_path": str(launch_support.PORTO),
        "scenario_id": "porto",
        "port": port,
        "host": "127.0.0.1",
    }
    params = tmp_path / "params.yaml"
    params.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
    remaps = ["-r", "__ns:=/test_simad", "-r", "__node:=racing_sim"]
    for name in launch_support.ABSOLUTE_NAMES:
        remaps += ["-r", f"{name}:={launch_support.topic(name)}"]
    log = (tmp_path / "server.log").open("w")
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "racing_sim_autodrive.server_node",
            "--ros-args",
            *remaps,
            "--params-file",
            str(params),
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    return process, tmp_path / "server.log"


def _wait_listening(process, log):
    deadline = time.monotonic() + LISTEN_TIMEOUT_S
    while time.monotonic() < deadline:
        if "listening on" in log.read_text(errors="replace"):
            return
        assert process.poll() is None, log.read_text(errors="replace")
        time.sleep(0.05)
    raise AssertionError("never listened: " + log.read_text(errors="replace"))


def _finish(process, log):
    try:
        code = process.wait(timeout=EXIT_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        pytest.fail("did not exit: " + log.read_text(errors="replace"))
    assert code == 0, (code, log.read_text(errors="replace")[-1500:])


@pytest.mark.parametrize("gap", [None, 0.0, 0.05, 0.15, 0.3, 0.6, 1.2])
def test_simad_2110_sigint_while_idle_and_again_during_teardown(tmp_path, gap):
    """SIMAD-2110: one SIGINT, or a second `gap` seconds after the first
    (the launch forwards one while the caller's group signal is the other),
    each exit 0."""
    process, log = _start(tmp_path, launch_support.free_port())
    try:
        _wait_listening(process, log)
        process.send_signal(2)
        if gap is not None:
            time.sleep(gap)
            if process.poll() is None:
                process.send_signal(2)
        _finish(process, log)
    finally:
        if process.poll() is None:
            process.kill()


def test_simad_2111_sigint_while_a_client_replays(tmp_path):
    """SIMAD-2111: SIGINT with the replay client connected and sending frames,
    twice, exits 0."""
    port = launch_support.free_port()
    process, log = _start(tmp_path, port)
    replay = None
    try:
        _wait_listening(process, log)
        replay = subprocess.Popen(
            launch_support.replay_command(
                port, "--pace", "recorded", "--count", "400"
            ),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(2.0)
        process.send_signal(2)
        time.sleep(0.1)
        if process.poll() is None:
            process.send_signal(2)
        _finish(process, log)
    finally:
        if replay is not None:
            replay.kill()
            replay.wait()
        if process.poll() is None:
            process.kill()
