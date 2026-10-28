"""Shared launch description for the tier-2 server tests (not a test)."""

import socket
import sys
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import yaml

REPO = Path(__file__).resolve().parents[4]
FIXTURE = Path(__file__).parent / "fixtures" / "porto_stream.jsonl.gz"
PORTO = REPO / "config" / "tracks" / "porto.yaml"

VEHICLE_DIR = REPO / "config" / "vehicles" / "autodrive_roboracer"


def _vehicle() -> dict:
    """Vehicle values from part 4's files, not copies (plan Step 0).

    lr, wheel radius and the throttle gain are read from source.yaml and
    identified.yaml. h and the steering limit are not single keys there
    (h is the rest height plus com_z; the file's 0.5236 is the guide's
    rounding of the prefab's 0.523633), so they stay the anchored part-4
    numbers; kp/ki are UNTUNED.
    """
    source = yaml.safe_load((VEHICLE_DIR / "source.yaml").read_text())
    identified = yaml.safe_load((VEHICLE_DIR / "identified.yaml").read_text())
    values = source["values"]
    return {
        "lr": float(values["com_x_lr_m"]["value"]),
        "h": 0.07354,
        "wheel_radius": float(values["wheel_radius_m"]["value"]),
        "throttle_gain": float(
            identified["measurements"]["throttle_gain"]["value"]
        ),
        "max_steering_angle": 0.523633,
        "kp": 0.02,
        "ki": 0.05,
    }


VEHICLE = _vehicle()


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
        parameters=[parameters],
        output="screen",
    )
    return server, launch.LaunchDescription(
        [server, launch_testing.actions.ReadyToTest()]
    )
