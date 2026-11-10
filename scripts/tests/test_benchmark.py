"""CHECK-2700..2780: the benchmark command (tier 1, ROS-free).

Every fixture is synthetic or a committed file. The command compares peers
and gates nothing (ADR 0005, ADR 0008 section 5): nothing here ranks or
bounds a value.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest

from scripts import benchmark

ROOT = Path(__file__).resolve().parents[2]
LIB_ROOT = ROOT / "tests" / "lib"
if str(LIB_ROOT) not in sys.path:
    sys.path.insert(0, str(LIB_ROOT))


def _stack(pose=("amcl",), planner=("optimized_raceline",), ctrl=()) -> dict:
    """A reference_stack.yaml-shaped dict."""

    def alternatives(names, status=None):
        out = []
        for name in names:
            entry = {"implementation": name, "package": f"pkg_{name}"}
            if status:
                entry["status"] = status
            out.append(entry)
        return out

    return {
        "roles": {
            "pose_source": {
                "implementation": "ground_truth",
                "package": "racing_bringup",
                "alternatives": alternatives(pose),
            },
            "planner": {
                "implementation": "track_centerline",
                "package": "racing_bringup",
                "alternatives": alternatives(planner),
            },
            "controller": {
                "implementation": "pure_pursuit",
                "package": "racing_controller_baseline",
                "alternatives": alternatives(ctrl),
            },
        }
    }


# -- CHECK-2700 ---------------------------------------------------------------


def test_check_2700_added_pose_source_maps_to_localization() -> None:
    base = _stack()
    head = _stack(pose=("amcl", "far_lio"))
    added = benchmark.added_implementations(base, head)
    assert added == [("pose_source", "far_lio")]
    assert benchmark.benchmark_roles(added) == (["localization"], [])


def test_check_2700_added_experimental_counts() -> None:
    head = _stack(pose=("amcl", "far_lio"))
    for entry in head["roles"]["pose_source"]["alternatives"]:
        entry["status"] = "experimental"
    added = benchmark.added_implementations(_stack(), head)
    assert added == [("pose_source", "far_lio")]


def test_check_2700_added_planner_maps_to_planning() -> None:
    head = _stack(planner=("optimized_raceline", "spline"))
    added = benchmark.added_implementations(_stack(), head)
    assert added == [("planner", "spline")]
    assert benchmark.benchmark_roles(added) == (["planning"], [])


def test_check_2700_added_controller_is_unbenchmarked() -> None:
    head = _stack(ctrl=("mpc",))
    added = benchmark.added_implementations(_stack(), head)
    assert added == [("controller", "mpc")]
    assert benchmark.benchmark_roles(added) == ([], [("controller", "mpc")])


def test_check_2700_both_roles_in_fixed_order() -> None:
    head = _stack(pose=("amcl", "x"), planner=("optimized_raceline", "y"))
    added = benchmark.added_implementations(_stack(), head)
    assert benchmark.benchmark_roles(added)[0] == ["localization", "planning"]


def test_check_2700_nothing_added() -> None:
    base = _stack()
    assert benchmark.added_implementations(base, copy.deepcopy(base)) == []
    # promotion: only `status` flips
    promoted = _stack(pose=("amcl",))
    base = copy.deepcopy(promoted)
    base["roles"]["pose_source"]["alternatives"][0]["status"] = "experimental"
    assert benchmark.added_implementations(base, promoted) == []
    # a paths edit
    edited = copy.deepcopy(promoted)
    edited["roles"]["pose_source"]["alternatives"][0]["paths"] = ["a/*"]
    assert benchmark.added_implementations(promoted, edited) == []


def test_check_2700_rename_counts_as_added() -> None:
    added = benchmark.added_implementations(
        _stack(pose=("amcl",)), _stack(pose=("amcl2",))
    )
    assert added == [("pose_source", "amcl2")]


def test_check_2700_dispatch_expands_all() -> None:
    assert benchmark.dispatch_roles("localization") == ["localization"]
    assert benchmark.dispatch_roles("planning") == ["planning"]
    assert benchmark.dispatch_roles("all") == ["localization", "planning"]
    with pytest.raises(ValueError):
        benchmark.dispatch_roles("controller")


# -- CHECK-2780 ---------------------------------------------------------------


def _runs():
    from scripts import benchmark_runs  # noqa: PLC0415

    return benchmark_runs


def test_check_2780_launch_arguments_centerline() -> None:
    runs = _runs()
    arguments = runs.launch_arguments(
        "f1tenth_default.yaml", "centerline", Path("/tmp/r.jsonl")
    )
    assert arguments["planner"] == "track_centerline"
    assert arguments["speed_source"] == "curvature_schedule"
    assert arguments["recording_path"] == "/tmp/r.jsonl"
    assert arguments["vehicle"].endswith("config/vehicles/f1tenth_default.yaml")
    assert "raceline_optimizer" not in arguments


def test_check_2780_launch_arguments_committed_line() -> None:
    runs = _runs()
    arguments = runs.launch_arguments(
        "f1tenth_default.yaml", "tum_mintime", Path("/tmp/r.jsonl")
    )
    assert arguments["planner"] == "optimized_raceline"
    assert arguments["raceline_optimizer"] == "tum_mintime"
    assert arguments["speed_source"] == "trajectory"


def test_check_2780_lines_of_committed_tree() -> None:
    runs = _runs()
    assert runs.lines_of("f1tenth-default-v1") == [
        "centerline",
        "tum_mincurv_iqp",
        "tum_mintime",
    ]
