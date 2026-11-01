"""The server's vehicle parameters for tests: one source (not a test).

Read by `_server_launch.py` (the tier-2 server tests) and by
`racing_sim_adapter/test/conformance.py` (the second registration), so the
two cannot disagree. Since part 8 the values live in
`config/vehicles/autodrive_roboracer/server.yaml`, the file the launch hands
the server; BRINGUP-1670 pins it to part 4's `source.yaml` / `identified.yaml`.
"""

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[4]
SERVER_FILE = (
    REPO / "config" / "vehicles" / "autodrive_roboracer" / "server.yaml"
)


def vehicle() -> dict:
    document = yaml.safe_load(SERVER_FILE.read_text())
    return {
        name: float(value)
        for name, value in document["racing_sim"]["ros__parameters"].items()
    }
