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


# -- CHECK-2730 ---------------------------------------------------------------


def _args(*argv):
    return benchmark.build_parser().parse_args(["run", *argv])


def test_check_2730_defaults() -> None:
    from scripts import benchmark_runs  # noqa: PLC0415
    from scripts.pose_source_report import SCENARIOS  # noqa: PLC0415

    matrix = benchmark.plan_matrix(_args())
    assert matrix.roles == ["localization", "planning"]
    assert matrix.seeds == list(benchmark_runs.SEEDS) == [42, 1030, 7, 2026]
    assert matrix.scenarios == list(SCENARIOS)


def test_check_2730_role_choice_and_single_role() -> None:
    assert benchmark.plan_matrix(_args("--role", "planning")).roles == [
        "planning"
    ]
    with pytest.raises(SystemExit) as stop:
        _args("--role", "controller")
    assert stop.value.code == 2


@pytest.mark.parametrize("bad", [["--seeds"], ["--seeds", "x"]])
def test_check_2730_seeds_must_be_a_nonempty_integer_list(bad) -> None:
    with pytest.raises(SystemExit) as stop:
        _args(*bad)
    assert stop.value.code == 2


def test_check_2730_a_seed_given_twice_is_refused() -> None:
    with pytest.raises(SystemExit) as stop:
        benchmark.plan_matrix(_args("--seeds", "42", "42"))
    assert stop.value.code == 2


def test_check_2730_analytic_circle_is_refused_with_the_reason(capsys) -> None:
    with pytest.raises(SystemExit) as stop:
        benchmark.plan_matrix(_args("--scenarios", "analytic_circle"))
    assert stop.value.code == 2
    assert "rotationally symmetric" in capsys.readouterr().err


def test_check_2730_unknown_scenario_is_refused(capsys) -> None:
    with pytest.raises(SystemExit) as stop:
        benchmark.plan_matrix(_args("--scenarios", "monza"))
    assert stop.value.code == 2
    assert "monza" in capsys.readouterr().err


def test_check_2730_scenario_override_reaches_expected() -> None:
    matrix = benchmark.plan_matrix(
        _args("--seeds", "7", "--scenarios", "spielberg")
    )
    assert matrix.seeds == [7]
    assert matrix.scenarios == ["spielberg"]


# -- CHECK-2740 ---------------------------------------------------------------


def _fake_stack(monkeypatch, *, latch=None, raises=False):
    """Replace the launches with fakes: ``latch`` names a role whose runs
    latch, ``raises`` makes every launch raise."""
    from scripts import benchmark_runs  # noqa: PLC0415
    from scripts.pose_source_report import record  # noqa: PLC0415

    monkeypatch.setenv("RACING_REVISION", "abc1234deadbeef")
    monkeypatch.setattr(benchmark_runs, "track_length", lambda: 100.0)
    monkeypatch.setattr(
        benchmark_runs, "launch_keys", lambda: ["slam_toolbox", "amcl", "mola"]
    )
    monkeypatch.setattr(benchmark_runs, "reference_row", lambda: {"map": "m"})
    monkeypatch.setattr(
        benchmark_runs, "static_fields", lambda version, line: {"length_m": 1.0}
    )
    monkeypatch.setattr(benchmark_runs, "regeneration_record", lambda t: {})
    state = {"loc": 0, "plan": 0}

    def localization_run(source, scenario, seed, length, tmp):
        if raises:
            raise RuntimeError("boom")
        state["loc"] += 1
        outcome = "latched" if latch == "localization" else "completed"
        run = {
            "outcome": outcome,
            "time_scale": 5.0,
            "drive_on_estimate": False,
            "metrics": {"lap_completed": True, "lap_time": 100.0},
        }
        return record(
            source,
            scenario,
            seed,
            run,
            {"ate_rmse": 0.3, "availability": 1.0},
            {"pose_rate_hz": 40.0, "tf_rate_hz": 90.0, "scan_rate_hz": 40.0},
            2.0,
            {"median": 0.04},
        )

    def planning_run(vehicle, line, seed, length, tmp):
        if raises:
            raise RuntimeError("boom")
        state["plan"] += 1
        version = benchmark_runs.VEHICLES[vehicle][1]
        run = {
            "vehicle": vehicle,
            "line": line,
            "seed": seed,
            "manifest_version": version,
        }
        if latch == "planning":
            return {**run, "outcome": "latched"}
        metrics = dict.fromkeys(
            (
                "lap_time",
                "minimum_wall_clearance",
                "maximum_tracking_error",
                "p95_tracking_error",
                "control_saturation_events",
            ),
            1.0,
        )
        metrics["vehicle_parameter_version"] = version
        return {**run, "outcome": "completed", "metrics": metrics}

    monkeypatch.setattr(benchmark_runs, "localization_run", localization_run)
    monkeypatch.setattr(benchmark_runs, "planning_run", planning_run)
    return state


def test_check_2740_clean_run_writes_both_reports(
    monkeypatch, tmp_path, capsys
) -> None:
    import json  # noqa: PLC0415

    _fake_stack(monkeypatch)
    out = tmp_path / "out"
    code = benchmark.main(
        ["run", "--role", "all", "--seeds", "42", "--out", str(out)]
    )
    assert code == 0
    shown = capsys.readouterr().out
    assert "| implementation | 42 |" in shown
    for role in ("localization", "planning"):
        assert (out / role / "report.md").is_file()
        document = json.loads((out / role / "report.json").read_text())
        block = document["benchmark"]
        assert block["role"] == role
        assert block["revision"] == "abc1234deadbeef"
        assert block["started_utc"].endswith("+00:00")
        assert "host_load" in block
        assert block["scored_against"].startswith("simulator ground truth")
        header = (out / role / "report.md").read_text()
        assert "abc1234" in header
        assert "scored against" in header
    assert "SIM-5040" in (out / "planning" / "report.md").read_text()


@pytest.mark.parametrize("role", ["localization", "planning"])
def test_check_2740_a_latched_run_exits_1_per_role(
    monkeypatch, tmp_path, capsys, role
) -> None:
    _fake_stack(monkeypatch, latch=role)
    code = benchmark.main(
        ["run", "--role", role, "--seeds", "42", "--out", str(tmp_path)]
    )
    assert code == 1
    assert "latched" in capsys.readouterr().out


@pytest.mark.parametrize("role", ["localization", "planning"])
def test_check_2740_a_missing_record_exits_1(
    monkeypatch, tmp_path, capsys, role
) -> None:
    from scripts import benchmark_runs  # noqa: PLC0415

    _fake_stack(monkeypatch)
    real = getattr(benchmark_runs, f"{role}_run")
    seen = []

    def lossy(*args):
        seen.append(args)
        return None if len(seen) == 1 else real(*args)

    monkeypatch.setattr(benchmark_runs, f"{role}_run", lossy)
    # a runner that returns nothing leaves a hole structural_failures names
    code = benchmark.main(
        ["run", "--role", role, "--seeds", "42", "--out", str(tmp_path)]
    )
    assert code == 1
    assert "missing" in capsys.readouterr().out


def test_check_2740_a_subset_run_is_not_missing_records(
    monkeypatch, tmp_path
) -> None:
    import json  # noqa: PLC0415

    state = _fake_stack(monkeypatch)
    code = benchmark.main(
        [
            "run",
            "--role",
            "localization",
            "--seeds",
            "42",
            "--scenarios",
            "spielberg",
            "--out",
            str(tmp_path),
        ]
    )
    assert code == 0
    # 3 present methods x 1 scenario x 1 seed
    assert state["loc"] == 3
    records = json.loads(
        (tmp_path / "localization" / "report.json").read_text()
    )["records"]
    assert {r["scenario"] for r in records} == {"spielberg"}


def test_check_2740_report_is_written_when_a_run_raises(
    monkeypatch, tmp_path
) -> None:
    _fake_stack(monkeypatch, raises=True)
    with pytest.raises(RuntimeError):
        benchmark.main(
            [
                "run",
                "--role",
                "planning",
                "--seeds",
                "42",
                "--out",
                str(tmp_path),
            ]
        )
    assert (tmp_path / "planning" / "report.json").is_file()
    assert (tmp_path / "planning" / "report.md").is_file()


# -- CHECK-2750 ---------------------------------------------------------------

FIXTURES = ROOT / "scripts" / "tests" / "fixtures" / "benchmark"


def _run_for_render(monkeypatch, tmp_path, role) -> Path:
    _fake_stack(monkeypatch)
    out = tmp_path / "out"
    assert (
        benchmark.main(
            ["run", "--role", role, "--seeds", "42", "--out", str(out)]
        )
        == 0
    )
    return out / role / "report.json"


@pytest.mark.parametrize("role", ["localization", "planning"])
def test_check_2750_render_reproduces_a_run_s_report(
    monkeypatch, tmp_path, capsys, role
) -> None:
    path = _run_for_render(monkeypatch, tmp_path, role)
    written = (path.parent / "report.md").read_text(encoding="utf-8")
    capsys.readouterr()
    assert benchmark.main(["render", str(path)]) == 0
    assert capsys.readouterr().out.strip() == written.strip()


def test_check_2750_legacy_pose_source_artifact(capsys) -> None:
    path = FIXTURES / "legacy_pose_source_report.json"
    assert benchmark.main(["render", str(path)]) == 0
    shown = capsys.readouterr().out
    assert (
        "scored against: simulator ground truth (legacy nightly artifact)"
        in (shown)
    )
    assert "## spielberg: ate_rmse" in shown
    assert "| implementation | 42 | 7 |" in shown
    # rows follow METHODS order, never ate_rmse order (amcl 0.31 > slam 0.22)
    assert shown.index("| slam_toolbox |") < shown.index("| amcl |")


def test_check_2750_legacy_raceline_artifact(capsys) -> None:
    path = FIXTURES / "legacy_raceline_report.json"
    assert benchmark.main(["render", str(path)]) == 0
    shown = capsys.readouterr().out
    assert "legacy nightly artifact" in shown
    assert "| default / centerline |" in shown
    assert "DNF 52.3 m" in shown


def test_check_2750_unrecognised_json_exits_2(tmp_path) -> None:
    path = tmp_path / "other.json"
    path.write_text('{"id": "SIM-3070", "numbers": {}}', encoding="utf-8")
    with pytest.raises(SystemExit) as stop:
        benchmark.main(["render", str(path)])
    assert stop.value.code == 2


# -- CHECK-2760 ---------------------------------------------------------------


def _report(
    folder: Path, name: str, report_id: str, seed, impl, revision, numbers
) -> None:
    import json  # noqa: PLC0415

    folder.mkdir(parents=True, exist_ok=True)
    (folder / f"{name}.json").write_text(
        json.dumps(
            {
                "id": report_id,
                "numbers": numbers,
                "baseline": None,
                "seed": seed,
                "scenario": "spielberg",
                "implementation": impl,
                "revision": revision,
                "host_load": {"load1": 1.0},
            }
        ),
        encoding="utf-8",
    )


def _history_tree(tmp_path: Path) -> list[Path]:
    one, two = tmp_path / "run-1", tmp_path / "run-2"
    slam = "reports-slam-seed-42"
    _report(
        one / slam, "SIM-5020-amcl", "SIM-5020", 42, "amcl", "aaaaaaa1234",
        {"live": {"ate_rmse": 0.9, "availability": 0.99}, "n": 3},
    )  # fmt: skip
    _report(
        one / slam, "SIM-5020-slam", "SIM-5020", 42, "slam_toolbox", "aaaaaaa1234",
        {"live": {"ate_rmse": 0.2, "availability": 0.95}, "n": 3},
    )  # fmt: skip
    _report(
        two / slam, "SIM-5020-amcl", "SIM-5020", 42, "amcl", "bbbbbbb5678",
        {"live": {"ate_rmse": 0.5, "availability": 1.0}, "n": 3},
    )  # fmt: skip
    _report(
        two / slam, "SIM-3110", "SIM-3110", 42, "optimized_raceline", "bbbbbbb5678",
        {"lap_time": 111.0, "series": [1, 2]},
    )  # fmt: skip
    (two / slam / "other.json").write_text('{"hello": 1}', encoding="utf-8")
    return [one, two]


def test_check_2760_groups_by_id_one_row_per_run(tmp_path, capsys) -> None:
    dirs = _history_tree(tmp_path)
    assert benchmark.main(["history", *map(str, dirs)]) == 0
    shown = capsys.readouterr()
    assert shown.out.count("## SIM-5020") == 1
    assert shown.out.count("## SIM-3110") == 1
    section = shown.out.split("## SIM-5020")[1].split("## SIM-3110")[0]
    rows = [r for r in section.splitlines() if r.startswith("| run-")]
    assert len(rows) == 3
    # run dir, short revision, seed, implementation, then the leaves
    assert rows[0].startswith("| run-1 | aaaaaaa | 42 | amcl | ")
    assert "live.ate_rmse" in section
    assert "series" not in shown.out  # lists are not scalar leaves


def test_check_2760_never_orders_by_value(tmp_path, capsys) -> None:
    dirs = _history_tree(tmp_path)
    benchmark.main(["history", *map(str, dirs)])
    section = (
        capsys.readouterr().out.split("## SIM-5020")[1].split("## SIM-3110")[0]
    )
    order = [
        r.split("|")[4].strip() for r in section.splitlines() if "| run-" in r
    ]
    # run-1 amcl (0.9), run-1 slam (0.2), run-2 amcl (0.5): by run dir
    assert order == ["amcl", "slam_toolbox", "amcl"]


def test_check_2760_field_restricts_the_columns(tmp_path, capsys) -> None:
    dirs = _history_tree(tmp_path)
    benchmark.main(["history", *map(str, dirs), "--field", "live.ate_rmse"])
    out = capsys.readouterr().out
    assert "live.ate_rmse" in out
    assert "live.availability" not in out


def test_check_2760_non_report_json_is_skipped_and_named(
    tmp_path, capsys
) -> None:
    dirs = _history_tree(tmp_path)
    benchmark.main(["history", *map(str, dirs)])
    assert "other.json" in capsys.readouterr().err
