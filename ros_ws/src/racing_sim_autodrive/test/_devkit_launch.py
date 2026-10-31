"""Shared pieces of the tier-2 front end B tests (not a test).

B and the devkit stand-in are started as processes. Stack-side names go under
`PREFIX`; devkit-side names (the stand-in's namespace, reset topic and /tf, B's
`devkit_namespace` and `devkit_reset_topic`) under `DEVKIT_PREFIX`, which
carries the stack domain `S`, so that two workers on neighbouring domains never
share a name on either of them (plan P6). The devkit domain is `S+1` (`S-1` if
that exceeds 101): B holds the devkit side on a second rclpy context there.
"""

import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from urllib.parse import urlparse

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import _server_launch as server_launch  # noqa: E402

REPO = server_launch.REPO
FIXTURE = server_launch.FIXTURE
PORTO = server_launch.PORTO
VEHICLE = server_launch.VEHICLE

STACK_DOMAIN = int(os.environ.get("ROS_DOMAIN_ID", "0"))
PREFIX = "/test_simad_devkit"
DEVKIT_PREFIX = f"/test_simad_devkit_dk{STACK_DOMAIN}"
NAMESPACE = f"{DEVKIT_PREFIX}/autodrive/roboracer_1"
RESET_TOPIC = f"{DEVKIT_PREFIX}/autodrive/reset_command"
DEVKIT_TF = f"{DEVKIT_PREFIX}/tf"
ABSOLUTE_NAMES = server_launch.ABSOLUTE_NAMES

# The stationary opening of the recording: loops cleanly, car at rest.
STATIONARY = ("--start-frame", "1", "--end-frame", "41", "--loop")


def topic(name: str) -> str:
    return f"{PREFIX}{name}"


def devkit_domain() -> int:
    """The domain the stand-in and B's devkit side run on: S+1, or S-1 if
    S+1 is past the last valid domain (101)."""
    return STACK_DOMAIN + 1 if STACK_DOMAIN + 1 <= 101 else STACK_DOMAIN - 1


DEVKIT_URL = os.environ.get(
    "AUTODRIVE_DEVKIT_URL", "http://autodrive-devkit:4567"
)


def devkit_reachable() -> bool:
    parts = urlparse(DEVKIT_URL)
    try:
        socket.create_connection(
            (parts.hostname, parts.port), timeout=1
        ).close()
    except OSError:
        return False
    return True


def check_not_live_devkit() -> None:
    """Refuse to run beside a live devkit whose domain could be ours.

    With AUTODRIVE_ROS_DOMAIN_ID set, its domain must be neither S nor the
    devkit-side domain. Unset, the container's domain is unknown (compose
    defaults it to 42), so refuse only if a devkit answers on its port: CI
    runs on domain 42 with no devkit and must pass.
    """
    live = os.environ.get("AUTODRIVE_ROS_DOMAIN_ID")
    clash = f"stack domain {STACK_DOMAIN} / devkit domain {devkit_domain()}"
    if live is None:
        if devkit_reachable():
            raise AssertionError(
                f"a devkit answers at {DEVKIT_URL} and"
                " AUTODRIVE_ROS_DOMAIN_ID is unset (compose defaults it to"
                f" 42): {clash} may clash with it. Stop it, or restart it on"
                " its own domain: AUTODRIVE_ROS_DOMAIN_ID=<D> docker compose"
                " -f docker/docker-compose.yaml --profile autodrive up -d"
                " --no-deps autodrive-devkit"
            )
        return
    if int(live) in (STACK_DOMAIN, devkit_domain()):
        raise AssertionError(
            f"{clash} equals AUTODRIVE_ROS_DOMAIN_ID={live}: a test must"
            " never subscribe to the live devkit"
        )


def start_bridge(tmp_path, **parameters):
    """B as a subprocess; returns (process, log path)."""
    check_not_live_devkit()
    values = {
        **VEHICLE,
        "world_to_map.x": 0.0,
        "world_to_map.y": 0.0,
        "world_to_map.yaw": 0.0,
        "track_path": str(PORTO),
        "scenario_id": "porto",
        "devkit_namespace": NAMESPACE,
        "devkit_reset_topic": RESET_TOPIC,
        "devkit_domain_id": devkit_domain(),
        **parameters,
    }
    params = tmp_path / "params.yaml"
    params.write_text(yaml.safe_dump({"/**": {"ros__parameters": values}}))
    remaps = ["-r", f"__ns:={PREFIX}", "-r", "__node:=racing_sim"]
    for name in ABSOLUTE_NAMES:
        remaps += ["-r", f"{name}:={topic(name)}"]
    log = tmp_path / "bridge.log"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "racing_sim_autodrive.devkit_node",
            "--ros-args",
            *remaps,
            "--params-file",
            str(params),
        ],
        stdout=log.open("w"),
        stderr=subprocess.STDOUT,
    )
    return process, log


def wait_for_log(process, log, text, timeout_s=60.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if text in log.read_text(errors="replace"):
            return
        assert process.poll() is None, log.read_text(errors="replace")
        time.sleep(0.05)
    raise AssertionError(f"no {text!r}: " + log.read_text(errors="replace"))


def start_standin(tmp_path, *extra, replies="replies.jsonl", domain=None):
    """The devkit stand-in as a subprocess on the devkit domain."""
    if domain is None:
        domain = devkit_domain()
    env = {**os.environ, "ROS_DOMAIN_ID": str(domain)}
    out = tmp_path / replies
    log = tmp_path / "standin.log"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "racing_sim_autodrive.devkit_standin",
            "--fixture",
            str(FIXTURE),
            "--namespace",
            NAMESPACE,
            "--reset-topic",
            RESET_TOPIC,
            "--tf-topic",
            DEVKIT_TF,
            "--replies-out",
            str(out),
            "--wait-subscribers",
            "10",
            *extra,
        ],
        env=env,
        stdout=log.open("w"),
        stderr=subprocess.STDOUT,
    )
    return process, out, log


def replies(path):
    import json

    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line)["reply"])
        except ValueError:
            break  # a half-written last line
    return rows


def stop(process, signal_number=2, timeout_s=20.0):
    if process.poll() is None:
        process.send_signal(signal_number)
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    return process.returncode
