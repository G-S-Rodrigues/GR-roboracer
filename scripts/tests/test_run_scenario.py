"""Tests for scripts/run_scenario.py: what it hands the scenario runner."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "run_scenario.py"
VEHICLE = "config/vehicles/f1tenth_gym_limits.yaml"


def _run(monkeypatch, arguments: list[str]) -> dict:
    """Run the script's main() with the scenario runner replaced by a
    recorder, and return the keywords it was called with."""
    spec = importlib.util.spec_from_file_location("run_scenario", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    seen: dict = {}

    def recorder(**keywords):
        seen.update(keywords)
        return {"lap_completed": True, "collision_count": 0}

    monkeypatch.setattr(module, "run_scenario", recorder)
    monkeypatch.setattr(sys, "argv", ["run_scenario.py", *arguments])
    assert module.main() == 0
    return seen


def test_check_1320_vehicle_is_passed_through_only_when_given(
    monkeypatch, capsys
) -> None:
    """CHECK-1320: `--vehicle X` reaches the launch as `vehicle:=X`; without
    it the launch's default stands (no `vehicle` key at all)."""
    given = _run(monkeypatch, ["--seed", "1", "--vehicle", VEHICLE])
    absent = _run(monkeypatch, ["--seed", "1"])
    capsys.readouterr()

    assert given["launch_arguments"] == {"vehicle": VEHICLE}
    assert not absent.get("launch_arguments")


def test_check_1430_planner_arguments_are_passed_through_only_when_given(
    monkeypatch, capsys
) -> None:
    """CHECK-1430: `--planner`, `--raceline-optimizer` and `--speed-source`
    reach the launch as `planner:=`, `raceline_optimizer:=` and
    `speed_source:=` next to `vehicle:=`; none given, no launch arguments."""
    given = _run(
        monkeypatch,
        [
            "--seed",
            "1",
            "--planner",
            "optimized_raceline",
            "--raceline-optimizer",
            "tum_mincurv_iqp",
            "--speed-source",
            "trajectory",
            "--vehicle",
            VEHICLE,
        ],
    )
    absent = _run(monkeypatch, ["--seed", "1"])
    capsys.readouterr()

    assert given["launch_arguments"] == {
        "planner": "optimized_raceline",
        "raceline_optimizer": "tum_mincurv_iqp",
        "speed_source": "trajectory",
        "vehicle": VEHICLE,
    }
    assert not absent.get("launch_arguments")
