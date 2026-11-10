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


# -- CHECK-2710 ---------------------------------------------------------------


def _committed_stack() -> dict:
    import yaml  # noqa: PLC0415

    return yaml.safe_load(
        (ROOT / "config" / "reference_stack.yaml").read_text(encoding="utf-8")
    )


def _coverage_failures(stack: dict) -> list[str]:
    """Why the benchmark does not cover ``stack``'s pose sources/planners."""
    from scripts import benchmark_runs  # noqa: PLC0415
    from scripts.pose_source_report import METHODS  # noqa: PLC0415

    failures = []
    names = {
        impl.name: impl.role
        for impl in benchmark.stack_from_dict(stack, {}, {}).implementations
    }
    pose = {n for n, r in names.items() if r == "pose_source"}
    pose -= set(benchmark.NOT_BENCHMARKED["pose_source"])
    methods = {name for name, method in METHODS.items() if method}
    failures += [
        f"pose_source {n}: no METHODS entry in pose_source_report"
        for n in sorted(pose - methods)
    ]
    failures += [
        f"METHODS {n}: not a pose_source implementation in the stack"
        for n in sorted(methods - pose)
    ]
    planners = {n for n, r in names.items() if r == "planner"}
    failures += [
        f"planner {n}: the planning runner does not map it"
        for n in sorted(planners - set(benchmark.PLANNER_RUNNABLE))
    ]
    for _, (_, version) in benchmark_runs.VEHICLES.items():
        for planner, lines_for in benchmark.PLANNER_RUNNABLE.items():
            if not lines_for(version):
                failures.append(f"planner {planner}: no line for {version}")
    return failures


def test_check_2710_committed_stack_is_covered() -> None:
    assert _coverage_failures(_committed_stack()) == []


def test_check_2710_a_new_pose_source_fails_naming_it() -> None:
    stack = _committed_stack()
    stack["roles"]["pose_source"]["alternatives"].append(
        {
            "implementation": "brand_new",
            "package": "p",
            "status": "experimental",
        }
    )
    failures = _coverage_failures(stack)
    assert any("brand_new" in f for f in failures)


def test_check_2710_a_new_planner_fails_naming_it() -> None:
    stack = _committed_stack()
    stack["roles"]["planner"]["alternatives"].append(
        {"implementation": "spline", "package": "p", "status": "experimental"}
    )
    assert any("spline" in f for f in _coverage_failures(stack))


def test_check_2710_ground_truth_is_named_as_not_an_estimator() -> None:
    reason = benchmark.NOT_BENCHMARKED["pose_source"]["ground_truth"]
    assert "not an estimator" in reason


# -- CHECK-2720 ---------------------------------------------------------------


def test_check_2720_rows_keep_key_order_never_value_order() -> None:
    table = {("a", 1): 9.0, ("b", 1): 1.0, ("c", 1): 5.0}
    out = benchmark.pivot(table, ["a", "b", "c"], [1])
    lines = [line for line in out.splitlines() if line.startswith("| ")]
    assert [line.split("|")[1].strip() for line in lines[1:]] == ["a", "b", "c"]
    reversed_out = benchmark.pivot(table, ["c", "b", "a"], [1])
    assert reversed_out != out


def test_check_2720_missing_seed_is_a_dash() -> None:
    out = benchmark.pivot({("a", 1): 2.5}, ["a"], [1, 2])
    assert out.splitlines()[0].split("|")[2:4] == [" 1 ", " 2 "]
    assert "| a | 2.5 | - |" in out


def test_check_2720_tuple_row_keys_join_with_slash() -> None:
    out = benchmark.pivot(
        {(("default", "centerline"), 1): "x"}, [("default", "centerline")], [1]
    )
    assert "| default / centerline | x |" in out


def test_check_2720_lap_cell() -> None:
    done = {"outcome": "completed", "metrics": {"lap_time": 12.3412}}
    dnf = {"outcome": "dnf", "distance_m": 52.31}
    assert benchmark.lap_cell(done) == "12.34"
    assert benchmark.lap_cell(dnf) == "DNF 52.3 m"
    for word in ("latched", "infrastructure", "budget"):
        assert benchmark.lap_cell({"outcome": word}) == word
    assert benchmark.lap_cell(None) == "-"


def test_check_2720_cell_none_is_dash_and_nan_stays_nan() -> None:
    from scripts.pose_source_report import cell  # noqa: PLC0415

    assert cell(None) == "-"
    assert cell(float("nan")) == "nan"
    assert cell(0.123456) == "0.123"
