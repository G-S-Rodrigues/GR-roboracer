"""Tests for the quality-report helper and the catastrophic bounds
(ADR 0008): CHECK-2260..2290 here; the static guards follow."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

LIB_ROOT = Path(__file__).resolve().parents[2] / "tests" / "lib"
if str(LIB_ROOT) not in sys.path:
    sys.path.insert(0, str(LIB_ROOT))

from racing_test_keywords import report  # noqa: E402


@pytest.fixture
def reports(tmp_path, monkeypatch):
    monkeypatch.setattr(report, "REPORT_ROOT", tmp_path / "reports")
    monkeypatch.setenv("RACING_REVISION", "abc123")
    return tmp_path / "reports"


def _write(report_id="SIM-9999", numbers=None, **kwargs):
    kwargs.setdefault("seed", 42)
    kwargs.setdefault("scenario", "config/scenarios/spielberg.yaml")
    kwargs.setdefault("implementation", "amcl")
    return report.write_report(
        report_id, numbers if numbers is not None else {"x": 1.5}, **kwargs
    )


def test_check_2260_report_file_and_line(reports, capsys):
    """CHECK-2260: the report carries every field and is printed."""
    path = _write(baseline={"POSITION_RMSE": 1.0})
    assert path == reports / "SIM-9999.json"
    data = json.loads(path.read_text())
    for key in (
        "id",
        "numbers",
        "baseline",
        "seed",
        "scenario",
        "implementation",
        "revision",
        "host_load",
    ):
        assert key in data, key
    assert data["id"] == "SIM-9999"
    assert data["numbers"] == {"x": 1.5}
    assert data["baseline"] == {"POSITION_RMSE": 1.0}
    assert data["revision"] == "abc123"
    out = capsys.readouterr().out.strip().splitlines()
    assert len(out) == 1
    prefix, report_id, body = out[0].split(" ", 2)
    assert (prefix, report_id) == ("REPORT", "SIM-9999")
    assert json.loads(body)["numbers"] == {"x": 1.5}


@pytest.mark.parametrize(
    "bad", ["../x", "SIM-1", "sim-3080", "SIM-3080/a", "", "SIM-3080 x"]
)
def test_check_2270_id_must_be_path_safe(reports, bad):
    """CHECK-2270: an ID that could leave the directory is refused."""
    with pytest.raises(ValueError):
        _write(bad)


def test_check_2270_suffixed_id_is_accepted(reports):
    assert _write("SIM-5020-amcl-seed-42").name == "SIM-5020-amcl-seed-42.json"


def test_check_2270_nan_is_null_and_json_is_strict(reports):
    """CHECK-2270: NaN and inf are written as null, nested too."""
    path = _write(numbers={"a": math.nan, "b": [math.inf, 1.0]})
    text = path.read_text()
    assert "NaN" not in text and "Infinity" not in text
    data = json.loads(text, parse_constant=lambda c: pytest.fail(c))
    assert data["numbers"] == {"a": None, "b": [None, 1.0]}


def test_check_2280_revision_prefers_env_then_sha(monkeypatch):
    """CHECK-2280: RACING_REVISION, then GITHUB_SHA."""
    monkeypatch.setenv("GITHUB_SHA", "sha1")
    monkeypatch.setenv("RACING_REVISION", "rev1")
    assert report.revision() == "rev1"
    monkeypatch.delenv("RACING_REVISION")
    assert report.revision() == "sha1"


def test_check_2280_revision_never_raises(monkeypatch, tmp_path):
    """CHECK-2280: no env and no git anywhere is "unknown", not a raise."""
    monkeypatch.delenv("RACING_REVISION", raising=False)
    monkeypatch.delenv("GITHUB_SHA", raising=False)
    monkeypatch.setattr(report, "REPOSITORY_ROOT", tmp_path)
    assert report.revision() == "unknown"


def test_check_2280_revision_from_git_when_no_env(monkeypatch):
    monkeypatch.delenv("RACING_REVISION", raising=False)
    monkeypatch.delenv("GITHUB_SHA", raising=False)
    value = report.revision()
    assert value == "unknown" or len(value) == 40


BOUND_CASES = [
    ("ate_rmse", 5.0, 5.01),
    ("availability", 0.90, 0.89),
    ("map_median", 1.0, 1.01),
    ("wall_cells", 21800, 21799),
]


@pytest.mark.parametrize("metric,edge,past", BOUND_CASES)
def test_check_2290_bound_edges_and_reasons(metric, edge, past):
    """CHECK-2290: the bound itself is accepted, one step past it and NaN are
    refused, and each bound states why."""
    bound = report.CATASTROPHIC_BOUNDS[metric]
    assert bound["measurement"].strip()
    kind = "maximum" if "maximum" in bound else "minimum"
    assert bound[kind] == edge
    key = "v"
    fields = {key: metric}
    assert report.catastrophic_violations({key: edge}, fields) == []
    assert len(report.catastrophic_violations({key: past}, fields)) == 1
    assert len(report.catastrophic_violations({key: math.nan}, fields)) == 1
    # a metric with no value (an empty map gives None) is outside too
    assert len(report.catastrophic_violations({key: None}, fields)) == 1


@pytest.mark.parametrize("metric,edge,past", BOUND_CASES[:2])
def test_check_2290_catastrophic_dict_for_the_comparator(metric, edge, past):
    """CHECK-2290: `catastrophic` builds a compare_localization bounds dict."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from compare_localization import compare_localization

    bounds = report.catastrophic(metric)
    assert set(bounds) == {metric}
    base = {
        "ate_rmse": 0.3,
        "availability": 0.99,
    }
    assert compare_localization(bounds, {**base, metric: edge}) == []
    assert compare_localization(bounds, {**base, metric: past})
    assert compare_localization(bounds, {**base, metric: math.nan})


def test_check_2290_working_values_pass_and_message_names_things():
    ok = {"position_rmse": 0.4, "availability": 0.99}
    fields = {"position_rmse": "ate_rmse", "availability": "availability"}
    assert report.catastrophic_violations(ok, fields) == []
    (message,) = report.catastrophic_violations(
        {"position_rmse": 7.0, "availability": 0.99}, fields
    )
    assert "position_rmse" in message and "7.0" in message
    assert "ate_rmse" in message and "5.0" in message


# --- CHECK-2300 / CHECK-2310: the inventory of tests that report ----------

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
_LOCALIZATION_NAMES = (
    "POSITION_RMSE_MAXIMUM",
    "POSITION_MAXIMUM",
    "AVAILABILITY_MINIMUM",
    "OFFLINE_BOUNDS",
)
_LAP_NAMES = ("LAP_TIME", "LAP_TIME_TOLERANCE")

# (path, superseded names, report ID prefix, catastrophic metrics it checks)
INVENTORY = [
    (
        "tests/system/test_sim_3070_slam_toolbox_localizes.py",
        _LOCALIZATION_NAMES,
        "SIM-3070",
        ("ate_rmse", "availability"),
    ),
    (
        "tests/system/test_sim_3080_amcl_localizes.py",
        _LOCALIZATION_NAMES,
        "SIM-3080",
        ("ate_rmse", "availability"),
    ),
    (
        "tests/system/test_sim_3510_mola_localizes.py",
        _LOCALIZATION_NAMES,
        "SIM-3510",
        ("ate_rmse", "availability"),
    ),
    (
        "tests/system/test_sim_3550_slam_toolbox_1080_localizes.py",
        (*_LOCALIZATION_NAMES, "MAP_MEDIAN_MAXIMUM", "MAP_P95_MAXIMUM"),
        "SIM-3550",
        ("ate_rmse", "availability", "map_median"),
    ),
    (
        "tests/system/test_sim_3530_mola_maps.py",
        ("MIN_CELLS", "P95_BOUND", "MEDIAN_BOUND", "SIGNED_BOUND"),
        "SIM-3530",
        ("wall_cells", "map_median"),
    ),
    (
        "tests/system/test_sim_3110_optimized_raceline_lap.py",
        _LAP_NAMES,
        "SIM-3110",
        (),
    ),
    (
        "tests/system/test_sim_3120_mintime_raceline_lap.py",
        _LAP_NAMES,
        "SIM-3120",
        (),
    ),
    (
        "tests/nightly/test_sim_5020_cross_product.py",
        ("BOUNDS",),
        "SIM-5020",
        ("ate_rmse", "availability"),
    ),
    (
        "tests/nightly/test_sim_5030_amcl_spread.py",
        ("violated",),
        "SIM-5030",
        (),
    ),
]
_INVENTORY_IDS = [row[2] for row in INVENTORY]


def _tree(path: str):
    import ast

    source = (REPOSITORY_ROOT / path).read_text()
    return ast.parse(source, filename=path)


def _call_name(node) -> str:
    import ast

    func = node.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return ""


def _names_in(node) -> list[tuple[str, int]]:
    import ast

    return [
        (inner.id, inner.lineno)
        for inner in ast.walk(node)
        if isinstance(inner, ast.Name)
    ]


def _strings_in(node) -> set[str]:
    import ast

    return {
        inner.value
        for inner in ast.walk(node)
        if isinstance(inner, ast.Constant) and isinstance(inner.value, str)
    }


def _asserted_empty_names(tree) -> set[str]:
    """Names an `assert` requires empty: `assert x == []` or `assert not x`."""
    import ast

    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assert):
            continue
        test = node.test
        if (
            isinstance(test, ast.Compare)
            and len(test.ops) == 1
            and isinstance(test.ops[0], ast.Eq)
            and isinstance(test.left, ast.Name)
            and isinstance(test.comparators[0], ast.List)
            and not test.comparators[0].elts
        ):
            names.add(test.left.id)
        elif (
            isinstance(test, ast.UnaryOp)
            and isinstance(test.op, ast.Not)
            and isinstance(test.operand, ast.Name)
        ):
            names.add(test.operand.id)
    return names


@pytest.mark.parametrize(
    "path,superseded,report_id,metrics", INVENTORY, ids=_INVENTORY_IDS
)
def test_check_2300_no_superseded_bound_and_catastrophic_kept(
    path, superseded, report_id, metrics
):
    """CHECK-2300: no assert (nor a compare_localization argument) reads a
    superseded bound or a `*_BASELINE` name, and the file still checks each
    of its catastrophic metrics through the catastrophic helpers and asserts
    each helper result empty (`assert x == []`)."""
    import ast

    tree = _tree(path)
    found = []
    for node in ast.walk(tree):
        scanned = []
        if isinstance(node, ast.Assert):
            scanned.append(node)
        elif isinstance(node, ast.Call) and (
            _call_name(node) == "compare_localization"
        ):
            scanned.extend([*node.args, *(k.value for k in node.keywords)])
        for part in scanned:
            for name, line in _names_in(part):
                if name in superseded or name.endswith("_BASELINE"):
                    found.append(f"{name} :{line}")
    assert not found, f"{path} still gates on: {sorted(set(found))}"

    checked: set[str] = set()
    unasserted: list[str] = []
    asserted = _asserted_empty_names(tree)
    helpers = ("catastrophic", "catastrophic_violations")

    def has_helper(part) -> bool:
        return any(
            isinstance(inner, ast.Call) and _call_name(inner) in helpers
            for inner in ast.walk(part)
        )

    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and has_helper(node.value):
            targets = [t.id for t in node.targets if isinstance(t, ast.Name)]
            if not set(targets) & asserted:
                unasserted.append(f"{targets} :{node.lineno}")
            checked |= _strings_in(node.value)
        elif isinstance(node, ast.Assert) and has_helper(node.test):
            checked |= _strings_in(node.test)
    assert not unasserted, (
        f"{path} computes catastrophic violations but never asserts them "
        f"empty: {unasserted}"
    )
    missing = [metric for metric in metrics if metric not in checked]
    assert not missing, f"{path} no longer checks catastrophic {missing}"


@pytest.mark.parametrize(
    "path,superseded,report_id,metrics", INVENTORY, ids=_INVENTORY_IDS
)
def test_check_2310_writes_its_report(path, superseded, report_id, metrics):
    """CHECK-2310: the file calls write_report with its own ID first."""
    import ast

    tree = _tree(path)
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call) and _call_name(node) == "write_report"
        ):
            continue
        if not node.args:
            continue
        first = node.args[0]
        if isinstance(first, ast.JoinedStr) and first.values:
            first = first.values[0]
        if (
            isinstance(first, ast.Constant)
            and isinstance(first.value, str)
            and first.value.startswith(report_id)
        ):
            return
    pytest.fail(f"{path}: no write_report('{report_id}', ...) call")


# --- CHECK-2330: racing.robot carries no localization bound column --------

ROBOT = REPOSITORY_ROOT / "tests" / "acceptance" / "racing.robot"


def _robot_sections() -> tuple[str, str]:
    text = ROBOT.read_text()
    cases, _, keywords_text = text.partition("*** Keywords ***")
    return cases, keywords_text


def _template_block() -> str:
    _, keywords_text = _robot_sections()
    start = keywords_text.index("Pose Source Should Track Ground Truth")
    return keywords_text[start:]


def test_check_2330_template_takes_no_bound() -> None:
    """CHECK-2330: the localization template has no bound arguments, reports
    before it asserts, and asserts the catastrophic bounds."""
    block = _template_block()
    arguments = next(
        line for line in block.splitlines() if "[Arguments]" in line
    )
    assert "${error_bound}" not in block
    assert "${availability_bound}" not in block
    assert "${test_id}" in arguments
    report_at = block.index("Write Localization Report")
    lap_at = block.index("Lap Completion Should Be")
    bounds_at = block.index(
        "Localization Should Stay Within Catastrophic Bounds"
    )
    assert report_at < lap_at < bounds_at


def test_check_2330_rows_carry_no_numeric_bound() -> None:
    """CHECK-2330: a row is an ID, a pose source and optional scenario and
    time scale; the old 4.0 / 0.65 / 2.5 and 0.99 columns are gone."""
    import re

    cases, _ = _robot_sections()
    rows = [
        line.strip()
        for line in cases.splitlines()
        if re.match(r"\s+(ACC-\d{4}\s+)?(slam_toolbox|amcl|mola)(\s|$)", line)
        and not line.strip().startswith(("#", "..."))
    ]
    assert len(rows) == 3, rows
    for row in rows:
        assert row.startswith("ACC-"), row
        assert not re.search(r"\s(4\.0|0\.65|2\.5|0\.99)(\s|$)", row), row


def test_check_2330_acc_literals_unchanged() -> None:
    """CHECK-2330: the clearance (0.08) and p95 tracking (1.0) asserts stay
    literals, as ADR 0008's table has them."""
    cases, _ = _robot_sections()
    assert cases.count("Minimum Wall Clearance Should Exceed    0.08") == 2
    assert cases.count("P95 Tracking Error Should Be Below    1.0") == 1


# --- CHECK-2360: the gate prints what the run reported --------------------


def test_check_2360_print_reports_newer_than_since_sorted(tmp_path, capsys):
    """CHECK-2360: one REPORT line per file at or after `since`, by name."""
    import os

    for name, mtime in (
        ("SIM-3110", 1000),
        ("SIM-3080", 2000),
        ("ACC-4010", 3000),
    ):
        path = tmp_path / f"{name}.json"
        path.write_text(json.dumps({"id": name}))
        os.utime(path, (mtime, mtime))
    (tmp_path / "notes.txt").write_text("not a report")
    report.print_reports(tmp_path, 2000)
    lines = capsys.readouterr().out.strip().splitlines()
    assert [line.split(" ", 2)[:2] for line in lines] == [
        ["REPORT", "ACC-4010"],
        ["REPORT", "SIM-3080"],
    ]
    assert json.loads(lines[0].split(" ", 2)[2]) == {"id": "ACC-4010"}


def test_check_2360_absent_directory_prints_nothing(tmp_path, capsys):
    report.print_reports(tmp_path / "nowhere", 0)
    assert capsys.readouterr().out == ""


def test_check_2372_diagnostics_only_when_given(reports):
    """CHECK-2372: a report carries `diagnostics` only when a caller gives
    some, so every existing report is byte-identical to today's."""
    plain = json.loads(_write().read_text())
    assert "diagnostics" not in plain
    given = json.loads(
        _write(
            diagnostics={"shutdown_escalations": [], "frozen_at": None}
        ).read_text()
    )
    assert given["diagnostics"] == {
        "shutdown_escalations": [],
        "frozen_at": None,
    }
    assert given["numbers"] == plain["numbers"]
