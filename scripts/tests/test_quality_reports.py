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


@pytest.mark.parametrize(
    "path,superseded,report_id,metrics", INVENTORY, ids=_INVENTORY_IDS
)
def test_check_2300_no_superseded_bound_and_catastrophic_kept(
    path, superseded, report_id, metrics
):
    """CHECK-2300: no assert (nor a compare_localization argument) reads a
    superseded bound or a `*_BASELINE` name, and the file still checks each
    of its catastrophic metrics through the catastrophic helpers."""
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
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and _call_name(node) in (
            "catastrophic",
            "catastrophic_violations",
        ):
            for part in [*node.args, *(k.value for k in node.keywords)]:
                checked |= _strings_in(part)
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
