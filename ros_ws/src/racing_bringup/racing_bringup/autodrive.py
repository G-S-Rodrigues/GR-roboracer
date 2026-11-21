"""What `simulator:=autodrive` runs, for every launch file that starts it."""

from pathlib import Path

import yaml

# What `simulator:=autodrive` runs: the only vehicle and the only track the
# AutoDRIVE build has active, the server's values and the world_to_map it
# publishes under (config/autodrive/porto_frame.yaml: measured in part 10;
# plan D3). Paths are repository-relative, like the launch files that use
# them.
AUTODRIVE_VEHICLES = ("autodrive_roboracer", "autodrive_roboracer_fast")
AUTODRIVE_SCENARIO = "porto"
AUTODRIVE_SERVER_CONFIG = "config/vehicles/autodrive_roboracer/server.yaml"
AUTODRIVE_FRAME = "config/autodrive/porto_frame.yaml"
# config/autodrive/simulator.yaml bridge_port (BRINGUP-1600 keeps them equal).
AUTODRIVE_PORT = "4567"


def world_to_map() -> dict:
    """The server's `world_to_map.*` parameters, from AUTODRIVE_FRAME."""
    document = yaml.safe_load(Path(AUTODRIVE_FRAME).read_text(encoding="utf-8"))
    return {
        f"world_to_map.{key}": float(value)
        for key, value in document["world_to_map"].items()
    }


def server_parameters(tracks: dict, port) -> list:
    """The `parameters` of racing_sim_autodrive_server: the server's values
    file (not the vehicle file, BRINGUP-1670), the Porto frame and track, the
    scenario's id and the port it listens on.

    `tracks` is `scenario_parameters.track_parameters`' result; `port` is
    `autodrive_port`'s text, so a bad one raises here, only when the server
    launches.
    """
    return [
        AUTODRIVE_SERVER_CONFIG,
        {
            **world_to_map(),
            "track_path": tracks["racing_safety_supervisor"]["track_path"],
            "scenario_id": AUTODRIVE_SCENARIO,
            "port": int(port),
        },
    ]


def check(vehicle: str, scenario: str) -> None:
    """Refuse a vehicle or scenario the AutoDRIVE build does not have.

    The build has two car files (the base and the fast caps) and one active
    track, so no other is offered.
    """
    if Path(vehicle).stem not in AUTODRIVE_VEHICLES:
        raise RuntimeError(
            "simulator:=autodrive runs vehicle "
            f"{' or '.join(AUTODRIVE_VEHICLES)} only, not {vehicle}"
        )
    if Path(scenario).stem != AUTODRIVE_SCENARIO:
        raise RuntimeError(
            f"simulator:=autodrive runs scenario {AUTODRIVE_SCENARIO} only "
            f"(the build has one active track), not {scenario}"
        )
