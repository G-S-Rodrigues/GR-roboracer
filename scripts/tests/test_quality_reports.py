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
