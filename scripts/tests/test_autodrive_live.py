"""CHECK-2490: one reader for the supervisor's stale-input timeout (A7-3).

Eight tests used to say what the timeout is: two byte-identical readers
(SIM-3610, SIM-3620) and six literals `0.1` passed to `judge_stale_input`.
The AutoDRIVE files now carry 200 ms (owner call #27), so a literal is untrue.
`autodrive_live` imports `scenario_runner` (rclpy), so this runs in `dev` with
ROS sourced, like `test_autodrive_porto_drive.py`.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "tests" / "lib") not in sys.path:
    sys.path.insert(0, str(ROOT / "tests" / "lib"))

from racing_test_keywords import autodrive_live  # noqa: E402

JUDGES = ("judge_stale_input", "_judge_stale_input")


def test_check_2490_one_stale_input_timeout_reader(tmp_path) -> None:
    """CHECK-2490: (a) `stale_input_timeout_s` returns a vehicle file's
    `racing_safety_supervisor` timeout in seconds, from an absolute path or
    one relative to the repository; (b) no live AutoDRIVE test defines its own
    reader, or passes a judge a numeric literal as the timeout."""
    vehicle = tmp_path / "vehicle.yaml"
    vehicle.write_text(
        "racing_safety_supervisor:\n"
        "  ros__parameters:\n"
        "    stale_input_timeout_ms: 150\n"
    )
    assert autodrive_live.stale_input_timeout_s(vehicle) == 0.15
    assert autodrive_live.stale_input_timeout_s(str(vehicle)) == 0.15
    assert (
        autodrive_live.stale_input_timeout_s(
            "config/vehicles/autodrive_roboracer.yaml"
        )
        == 0.2
    )

    files = sorted((ROOT / "tests" / "autodrive").glob("*.py")) + sorted(
        (ROOT / "tests" / "system").glob("test_sim_36[12]0_*.py")
    )
    assert files
    found = []
    for path in files:
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.FunctionDef)
                and node.name == "_stale_input_timeout_seconds"
            ):
                found.append(f"{path.name}:{node.lineno} defines a reader")
            if not isinstance(node, ast.Call):
                continue
            name = getattr(node.func, "id", None) or getattr(
                node.func, "attr", None
            )
            if name in JUDGES and len(node.args) >= 3:
                third = node.args[2]
                if isinstance(third, ast.Constant) and isinstance(
                    third.value, (int, float)
                ):
                    found.append(f"{path.name}:{node.lineno} literal timeout")
    assert not found, found
