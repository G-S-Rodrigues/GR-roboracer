"""Tests for the tier-4 localization keywords: the recording they keep and
what a failed assert says (N5, nightly 39568107208, ACC-4030)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

LIB_ROOT = Path(__file__).resolve().parents[2] / "tests" / "lib"
if str(LIB_ROOT) not in sys.path:
    sys.path.insert(0, str(LIB_ROOT))

from racing_test_keywords import keywords  # noqa: E402

SCORE = {
    "availability": 0.98,
    "maximum_gap": 2.8,
    "stale_gap_count": 1,
    "sample_count": 12211,
    "scored_count": 11984,
    "ate_rmse": 0.31,
}


class _RobotOutputDir:
    """A BuiltIn stand-in whose `${OUTPUT DIR}` is a given directory."""

    def __init__(self, directory: Path) -> None:
        self._directory = directory

    def __call__(self):
        return self

    def get_variable_value(self, name: str):
        assert name == "${OUTPUT DIR}"
        return str(self._directory)


class _NoRobot:
    """BuiltIn outside Robot raises when asked for a variable."""

    def __call__(self):
        return self

    def get_variable_value(self, name: str):
        raise RuntimeError("Cannot access execution context")


def _run(monkeypatch, **call) -> dict:
    """Call the keyword with the runner and scorer stubbed; the stub runner
    writes one line to the recording path it is handed."""
    seen: dict = {}

    def fake_run_scenario(**kwargs):
        path = Path(kwargs["launch_arguments"]["recording_path"])
        seen["run_path"] = path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}\n")
        return {"lap_completed": True}

    def fake_score(path):
        seen["score_path"] = Path(path)
        return dict(SCORE)

    monkeypatch.setattr(keywords, "run_scenario", fake_run_scenario)
    monkeypatch.setattr(keywords, "score_localization", fake_score)
    library = keywords.RacingTestKeywords()
    library.run_racing_scenario_on_pose_source("amcl", **call)
    seen["library"] = library
    return seen


@pytest.mark.parametrize(
    "drive, name",
    [("false", "amcl-open-loop.jsonl"), ("true", "amcl-closed-loop.jsonl")],
)
def test_check_1400_recording_is_kept_under_robot_output_dir(
    monkeypatch, tmp_path, drive, name
) -> None:
    """CHECK-1400: the recording lands under Robot's `${OUTPUT DIR}`, is the
    path both the launch and the scorer use, and survives the call."""
    monkeypatch.setattr(keywords, "BuiltIn", _RobotOutputDir(tmp_path))
    seen = _run(monkeypatch, drive_on_estimate=drive)
    expected = tmp_path / "localization" / name
    assert seen["run_path"] == expected
    assert seen["score_path"] == expected
    assert expected.exists()


def test_check_1400_recording_falls_back_outside_robot(
    monkeypatch, tmp_path
) -> None:
    """CHECK-1400: outside Robot the root is `<repo>/log/robot`."""
    monkeypatch.setattr(keywords, "BuiltIn", _NoRobot())
    monkeypatch.setattr(keywords, "REPOSITORY_ROOT", tmp_path)
    seen = _run(monkeypatch)
    expected = tmp_path / "log" / "robot" / "localization"
    assert seen["run_path"] == expected / "amcl-open-loop.jsonl"
    assert seen["run_path"].exists()


@pytest.mark.parametrize(
    "call, argument",
    [
        ("localization_availability_should_exceed", 0.99),
        ("localization_error_should_be_below", 0.1),
    ],
)
def test_check_1410_failed_assert_names_where_coverage_was_lost(
    monkeypatch, tmp_path, call, argument
) -> None:
    """CHECK-1410: a failed availability or error assert carries the score
    summary and the recording path, not just the value."""
    monkeypatch.setattr(keywords, "BuiltIn", _RobotOutputDir(tmp_path))
    seen = _run(monkeypatch)
    with pytest.raises(AssertionError) as raised:
        getattr(seen["library"], call)(argument)
    text = str(raised.value)
    for field in (
        "maximum_gap",
        "stale_gap_count",
        "sample_count",
        "scored_count",
        "ate_rmse",
        str(seen["run_path"]),
    ):
        assert field in text
