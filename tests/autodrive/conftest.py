"""Fixtures for tests/autodrive: SIM-3710 and SIM-3720 against the real
devkit container (#44 part 9).

These need the pinned `autodrive-devkit` container, which the CI image does
not have, so they live here and NEVER under `tests/system/`: that tree is what
the ready gate and the nightly select from, and a test that needs the
container must not be selectable by either. They are run by hand, for a part's
PR, on every edition bump (`config/autodrive/simulator.yaml`) and by part 10.
With the container down they FAIL, naming the command that starts it; they
never skip.

Domains (plan P6). The stack side runs on `ROS_DOMAIN_ID` (S). The devkit
container runs on `AUTODRIVE_ROS_DOMAIN_ID` (D), which is required, is not S,
and is no one else's stack domain: the real devkit puts a global `/tf`
(`world -> roboracer_1 -> lidar`) on D. SIM-3720's stand-in side runs on S+1
(S-1 if S+1 > 101), prefixed with S in every name.

Run, from `dev` with ROS sourced:

    docker compose -f docker/docker-compose.yaml --profile autodrive \
        up -d --no-deps autodrive-devkit      # AUTODRIVE_ROS_DOMAIN_ID=D
    ROS_DOMAIN_ID=S AUTODRIVE_ROS_DOMAIN_ID=D pytest tests/autodrive
"""

import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from urllib.parse import urlparse

import pytest
import rclpy
import yaml
from rclpy.context import Context

REPO = Path(__file__).resolve().parents[2]
TEST_DIR = REPO / "ros_ws/src/racing_sim_autodrive/test"
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(REPO / "tests/lib"))

import _server_launch as server_launch  # noqa: E402

FIXTURE = server_launch.FIXTURE
PORTO = server_launch.PORTO
VEHICLE = server_launch.VEHICLE
ABSOLUTE_NAMES = server_launch.ABSOLUTE_NAMES

DEVKIT_URL = os.environ.get(
    "AUTODRIVE_DEVKIT_URL", "http://autodrive-devkit:4567"
)
STACK_DOMAIN = int(os.environ.get("ROS_DOMAIN_ID", "0"))
COMPOSE = (
    "AUTODRIVE_ROS_DOMAIN_ID=<D> docker compose -f docker/docker-compose.yaml"
    " --profile autodrive up -d --no-deps autodrive-devkit"
)
LIDAR_TOPIC = "/autodrive/roboracer_1/lidar"
PREFIX_B = "/test_autodrive_live_b"
PREFIX_A = "/test_autodrive_live_a"


def live_domain() -> int:
    value = os.environ.get("AUTODRIVE_ROS_DOMAIN_ID")
    if value is None:
        pytest.fail(
            "AUTODRIVE_ROS_DOMAIN_ID is required: the domain the devkit"
            f" container runs on. Start it with: {COMPOSE}"
        )
    domain = int(value)
    if domain == STACK_DOMAIN:
        pytest.fail(
            f"AUTODRIVE_ROS_DOMAIN_ID={domain} equals ROS_DOMAIN_ID="
            f"{STACK_DOMAIN}: the devkit's global /tf would land on the stack"
            " graph"
        )
    return domain


def standin_domain() -> int:
    return STACK_DOMAIN + 1 if STACK_DOMAIN + 1 <= 101 else STACK_DOMAIN - 1


@pytest.fixture(scope="session")
def devkit() -> dict:
    """The container is reachable and its bridge node is up, or fail."""
    domain = live_domain()
    parts = urlparse(DEVKIT_URL)
    try:
        socket.create_connection(
            (parts.hostname, parts.port), timeout=3
        ).close()
    except OSError as error:
        pytest.fail(f"no devkit at {DEVKIT_URL} ({error}). Start it: {COMPOSE}")
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("autodrive_live_probe", context=context)
    try:
        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            # Humble's node names do not reach a Jazzy graph listing, its
            # publishers do: the bridge is up when it publishes the lidar.
            if node.get_publishers_info_by_topic(LIDAR_TOPIC):
                break
            time.sleep(0.2)
        else:
            pytest.fail(
                f"no autodrive_bridge publisher on {LIDAR_TOPIC}"
                f" (domain {domain}),"
                " within 10 s."
                f" Start the container: {COMPOSE}"
            )
    finally:
        node.destroy_node()
        context.shutdown()
    return {"url": DEVKIT_URL, "domain": domain}


def _remaps(prefix: str) -> list[str]:
    args = ["-r", f"__ns:={prefix}", "-r", "__node:=racing_sim"]
    for name in ABSOLUTE_NAMES:
        args += ["-r", f"{name}:={prefix}{name}"]
    return args


def _params(tmp_path, name, values) -> Path:
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": values}}))
    return path


def base_values(**extra) -> dict:
    return {
        **VEHICLE,
        "world_to_map.x": 0.0,
        "world_to_map.y": 0.0,
        "world_to_map.yaw": 0.0,
        "track_path": str(PORTO),
        "scenario_id": "porto",
        **extra,
    }


def start_node(tmp_path, module, prefix, tag, **values):
    """A bridge node (A or B) as a subprocess under a test prefix."""
    log = tmp_path / f"{tag}.log"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            module,
            "--ros-args",
            *_remaps(prefix),
            "--params-file",
            str(_params(tmp_path, tag, base_values(**values))),
        ],
        stdout=log.open("w"),
        stderr=subprocess.STDOUT,
    )
    return process, log


def wait_for_log(process, log, text, timeout_s=60.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if text in log.read_text(errors="replace"):
            return
        assert process.poll() is None, log.read_text(errors="replace")
        time.sleep(0.05)
    raise AssertionError(f"no {text!r}: " + log.read_text(errors="replace"))


def stop(process, timeout_s=20.0):
    if process.poll() is None:
        process.send_signal(2)
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def free_port() -> int:
    return server_launch.free_port()


def replay(url, tmp_path, tag, *window, pace="recorded", replies=True):
    """The recorded stream through Socket.IO to `url`; returns (proc, out)."""
    out = tmp_path / f"{tag}_replies.jsonl"
    command = [
        sys.executable,
        "-m",
        "racing_sim_autodrive.replay",
        "--fixture",
        str(FIXTURE),
        "--url",
        url,
        "--pace",
        pace,
        *window,
    ]
    if replies:
        command += ["--replies-out", str(out)]
    log = tmp_path / f"{tag}_replay.log"
    process = subprocess.Popen(
        command, stdout=log.open("w"), stderr=subprocess.STDOUT
    )
    return process, out, log


def read_replies(path: Path) -> list[dict]:
    import json

    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line)["reply"])
        except ValueError:
            break
    return rows
