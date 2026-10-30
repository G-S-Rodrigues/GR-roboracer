"""Shared launch description for the tier-2 server tests (not a test)."""

import socket
import sys
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions

sys.path.insert(0, str(Path(__file__).resolve().parent))

import _vehicle_values  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
FIXTURE = Path(__file__).parent / "fixtures" / "porto_stream.jsonl.gz"
PORTO = REPO / "config" / "tracks" / "porto.yaml"

VEHICLE = _vehicle_values.vehicle()

# colcon runs other packages' tier-2 tests in parallel on one domain, so the
# server's absolute names must not land on the shared graph (SIMAD-2100).
PREFIX = "/test_simad"
ABSOLUTE_NAMES = (
    "/scan",
    "/ground_truth/scan",
    "/imu",
    "/odom",
    "/ground_truth/odom",
    "/ground_truth/track_relative_state",
    "/tf",
    "/drive",
)


def topic(name: str) -> str:
    return f"{PREFIX}{name}"


def free_port() -> int:
    """An ephemeral port, never the sim's 4567 (held by other parts)."""
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def replay_command(port: int, *extra: str) -> list[str]:
    return [
        sys.executable,
        "-m",
        "racing_sim_autodrive.replay",
        "--fixture",
        str(FIXTURE),
        "--url",
        f"http://127.0.0.1:{port}",
        *extra,
    ]


def server_description(port: int, **overrides):
    parameters = {
        **VEHICLE,
        "world_to_map.x": 0.0,
        "world_to_map.y": 0.0,
        "world_to_map.yaw": 0.0,
        "track_path": str(PORTO),
        "scenario_id": "porto",
        "port": port,
        "host": "127.0.0.1",
        **overrides,
    }
    server = launch_ros.actions.Node(
        package="racing_sim_autodrive",
        executable="racing_sim_autodrive_server",
        name="racing_sim",
        namespace=PREFIX,
        remappings=[(n, topic(n)) for n in ABSOLUTE_NAMES],
        parameters=[parameters],
        output="screen",
    )
    return server, launch.LaunchDescription(
        [server, launch_testing.actions.ReadyToTest()]
    )
