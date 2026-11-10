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
        seen["launch_arguments"] = dict(kwargs["launch_arguments"])
        seen["run_kwargs"] = dict(kwargs)
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


RACELINE = {
    "planner": "optimized_raceline",
    "raceline_optimizer": "tum_mincurv_iqp",
    "speed_source": "trajectory",
}


@pytest.mark.parametrize(
    "call, name",
    [
        ({"drive_on_estimate": "false"}, "amcl-open-loop.jsonl"),
        ({"drive_on_estimate": "true"}, "amcl-closed-loop.jsonl"),
        (
            {"drive_on_estimate": "true", **RACELINE},
            "amcl-closed-loop-optimized_raceline.jsonl",
        ),
    ],
)
def test_check_1400_recording_is_kept_under_robot_output_dir(
    monkeypatch, tmp_path, call, name
) -> None:
    """CHECK-1400: the recording lands under Robot's `${OUTPUT DIR}`, is the
    path both the launch and the scorer use, and survives the call. A
    planner is named in the file (two ACC-4040 cases must not overwrite one
    recording) and reaches the launch; one not given is absent from it, not
    the string "None"."""
    monkeypatch.setattr(keywords, "BuiltIn", _RobotOutputDir(tmp_path))
    seen = _run(monkeypatch, **call)
    expected = tmp_path / "localization" / name
    assert seen["run_path"] == expected
    assert seen["score_path"] == expected
    assert expected.exists()
    passed = {k: v for k, v in call.items() if k in RACELINE}
    assert seen["launch_arguments"] == {
        "recording_path": str(expected),
        **passed,
    }


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


def _library_with(
    monkeypatch, tmp_path, **score
) -> keywords.RacingTestKeywords:
    from racing_test_keywords import report

    monkeypatch.setattr(report, "REPORT_ROOT", tmp_path / "reports")
    monkeypatch.setattr(keywords, "BuiltIn", _RobotOutputDir(tmp_path))
    seen = _run(monkeypatch)
    seen["library"]._localization = {**SCORE, **score}
    return seen["library"]


@pytest.mark.parametrize(
    "score, fails",
    [
        ({"ate_rmse": 0.3, "availability": 0.995}, False),
        ({"ate_rmse": 5.01, "availability": 0.995}, True),
        ({"ate_rmse": 0.3, "availability": 0.89}, True),
    ],
)
def test_check_2320_catastrophic_keyword(
    monkeypatch, tmp_path, score, fails
) -> None:
    """CHECK-2320: the tier-4 catastrophic keyword refuses a lost estimate
    and a stalled one, and passes a working one."""
    library = _library_with(monkeypatch, tmp_path, **score)
    if fails:
        with pytest.raises(AssertionError):
            library.localization_should_stay_within_catastrophic_bounds()
    else:
        library.localization_should_stay_within_catastrophic_bounds()


def test_check_2320_localization_report_keyword(monkeypatch, tmp_path) -> None:
    """CHECK-2320: the report carries the offline score and the run."""
    import json

    library = _library_with(monkeypatch, tmp_path, ate_maximum=1.2)
    library.write_localization_report("ACC-4030-amcl")
    data = json.loads((tmp_path / "reports" / "ACC-4030-amcl.json").read_text())
    for field in ("ate_rmse", "ate_maximum", "availability"):
        assert field in data["numbers"]
    assert data["implementation"] == "amcl"
    assert data["seed"] == 42


def test_check_2320_metrics_report_keyword(monkeypatch, tmp_path) -> None:
    """CHECK-2320: the metrics report carries the three clearance and
    tracking numbers."""
    import json

    from racing_test_keywords import report

    monkeypatch.setattr(report, "REPORT_ROOT", tmp_path / "reports")
    library = keywords.RacingTestKeywords()
    library._metrics = {
        "lap_completed": True,
        "lap_time": 120.4,
        "minimum_wall_clearance": 1.7,
        "p95_tracking_error": 0.14,
    }
    library._run_info = {
        "seed": 42,
        "scenario": "config/scenarios/spielberg.yaml",
        "implementation": "ground_truth",
    }
    library.write_metrics_report("ACC-4010")
    data = json.loads((tmp_path / "reports" / "ACC-4010.json").read_text())
    for field in ("lap_time", "minimum_wall_clearance", "p95_tracking_error"):
        assert field in data["numbers"]


# ---- CHECK-2372: diagnostics beside, never inside, the report's numbers ----


def _outcome(**attributes):
    from racing_test_keywords.scenario_runner import RunOutcome

    outcome = RunOutcome(lap_completed=False, lap_time=0.0, collision_count=0)
    for name, value in attributes.items():
        setattr(outcome, name, value)
    return outcome


@pytest.fixture
def reports(tmp_path, monkeypatch):
    from racing_test_keywords import report

    monkeypatch.setattr(report, "REPORT_ROOT", tmp_path / "reports")
    monkeypatch.setenv("RACING_REVISION", "abc123")
    return tmp_path / "reports"


def test_check_2372_reports_carry_the_run_diagnostics(reports):
    import json

    library = keywords.RacingTestKeywords()
    escalation = [{"name": "racing_recording-4", "signal": "SIGKILL"}]
    library._metrics = _outcome(
        shutdown_escalations=escalation, frozen_at=(110.51, 173.8, -0.88)
    )
    library._localization = dict(SCORE)
    library._run_info = {"seed": 7, "scenario": "s", "implementation": "amcl"}
    library.write_metrics_report("ACC-9001")
    library.write_localization_report("ACC-9002")
    metrics = json.loads((reports / "ACC-9001.json").read_text())
    localization = json.loads((reports / "ACC-9002.json").read_text())
    assert metrics["diagnostics"]["shutdown_escalations"] == escalation
    assert metrics["diagnostics"]["frozen_at"] == [110.51, 173.8, -0.88]
    assert localization["diagnostics"]["shutdown_escalations"] == escalation
    # numbers stay exactly what they were: the metrics, and the score
    assert set(metrics["numbers"]) == {
        "lap_completed",
        "lap_time",
        "collision_count",
    }
    assert localization["numbers"] == SCORE


def test_check_2372_a_plain_dict_outcome_still_reports(reports):
    """CHECK-1400/1410 stub the runner with a plain dict."""
    import json

    library = keywords.RacingTestKeywords()
    library._metrics = {"lap_completed": True}
    library._run_info = {"seed": 1}
    library.write_metrics_report("ACC-9003")
    record = json.loads((reports / "ACC-9003.json").read_text())
    assert record["numbers"] == {"lap_completed": True}
    assert record["diagnostics"]["shutdown_escalations"] == []


def test_check_2372_failure_text_names_the_escalations():
    library = keywords.RacingTestKeywords()
    library._metrics = _outcome(
        shutdown_escalations=[{"name": "stubborn-1", "signal": "SIGKILL"}]
    )
    with pytest.raises(AssertionError) as raised:
        library.lap_completion_should_be("true")
    text = str(raised.value)
    assert text.startswith("expected lap_completed=True, was False")
    assert "shutdown escalations: stubborn-1 SIGKILL" in text
    # none seen: the text is exactly today's
    library._metrics = _outcome()
    with pytest.raises(AssertionError) as raised:
        library.lap_completion_should_be("true")
    assert str(raised.value) == "expected lap_completed=True, was False"


# ---- CHECK-2373: the pose-source keyword stops on a frozen pose ----

REPO = Path(__file__).resolve().parents[2]


def _scenario_track_length(scenario: str) -> float:
    import racing_common
    from racing_bringup.scenario_parameters import track_parameters

    parameters = track_parameters(Path(scenario))
    path = parameters["racing_metrics"]["track_path"]
    return racing_common.Track.from_yaml(str(path)).length()


@pytest.mark.parametrize(
    "pose_source,call",
    [
        ("amcl", {"drive_on_estimate": "true"}),
        (
            "mola",
            {
                "scenario": "config/scenarios/spielberg_1080.yaml",
                "time_scale": "1.0",
            },
        ),
    ],
)
def test_check_2373_pose_source_keyword_stops_on_frozen_pose(
    monkeypatch, pose_source, call
):
    from scripts import frozen_pose

    monkeypatch.chdir(REPO)
    seen: dict = {}

    def fake_run_scenario(**kwargs):
        seen.update(kwargs)
        Path(kwargs["launch_arguments"]["recording_path"]).parent.mkdir(
            parents=True, exist_ok=True
        )
        Path(kwargs["launch_arguments"]["recording_path"]).write_text("{}\n")
        return {"lap_completed": True}

    monkeypatch.setattr(keywords, "run_scenario", fake_run_scenario)
    monkeypatch.setattr(keywords, "score_localization", lambda p: dict(SCORE))
    monkeypatch.setattr(keywords, "_output_root", lambda: REPO / "log" / "t")
    library = keywords.RacingTestKeywords()
    library.run_racing_scenario_on_pose_source(pose_source, **call)
    assert seen["stop_on_frozen_pose"] == 2 * frozen_pose.FROZEN_WINDOW_SECONDS
    scenario = call.get("scenario", keywords.LOCALIZATION_SCENARIO)
    assert seen["track_length"] == _scenario_track_length(scenario)


def test_check_2373_the_track_length_is_the_metrics_node_s():
    """Gotcha 15: one definition of a lap. Spielberg's is 343.3 m."""
    from racing_test_keywords.scenario_runner import scenario_track_length

    length = scenario_track_length(
        str(REPO / "config/scenarios/spielberg.yaml")
    )
    assert length == pytest.approx(343.3, abs=0.1)
    with pytest.raises(TypeError):
        scenario_track_length(None)


# ---- CHECK-2375: the frozen pose gets a named verdict ----


def _freeze_library(tmp_path, **attributes):
    import json

    recording = tmp_path / "amcl.jsonl"
    event = {
        "record_type": "event",
        "stamp": {"sec": 110, "nanosec": 400_000_000},
        "topic": "/evaluation/localization_error",
        "payload": {
            "estimate_topic": "/localization/odom",
            "longitudinal_error": 1.38,
            "lateral_error": 0.02,
        },
    }
    recording.write_text(json.dumps(event) + "\n")
    library = keywords.RacingTestKeywords()
    library._metrics = _outcome(**attributes)
    library._recording = recording
    return library


def _lap_text(library) -> str:
    with pytest.raises(AssertionError) as raised:
        library.lap_completion_should_be("true")
    return str(raised.value)


def test_check_2375_a_frozen_pose_is_named_a_collision(tmp_path):
    library = _freeze_library(
        tmp_path,
        frozen_at=(110.51, 173.8, -0.88),
        stop_context={"text": "observer text", "latched": False},
    )
    text = _lap_text(library)
    assert text.startswith("expected lap_completed=True, was False")
    assert (
        "ground truth frozen at sim 110.51 s, s 173.8, d -0.88: "
        "the gym holds its state on a collision"
    ) in text
    assert "estimate lon +1.38 m, lat +0.02 m at the freeze" in text
    assert "observer text" in text


def test_check_2375_a_latch_is_not_called_a_collision(tmp_path):
    library = _freeze_library(
        tmp_path,
        frozen_at=(4.95, 12.0, 0.1),
        stop_context={
            "text": "first non-NONE status: REASON_TRACK_LIMIT",
            "latched": True,
        },
    )
    text = _lap_text(library)
    assert "ground truth frozen at sim 4.95 s" in text
    assert "a latch, not a collision" in text
    assert "holds its state on a collision" not in text
    assert "REASON_TRACK_LIMIT" in text


def test_check_2375_not_frozen_keeps_todays_text(tmp_path):
    library = _freeze_library(tmp_path)
    assert _lap_text(library) == "expected lap_completed=True, was False"


def test_check_2375_a_diagnostic_never_hides_the_assertion(tmp_path):
    library = _freeze_library(
        tmp_path,
        frozen_at=(110.51, 173.8, -0.88),
        stop_context={"text": "x", "latched": False},
    )
    library._recording = tmp_path / "missing.jsonl"
    text = _lap_text(library)
    assert text.startswith("expected lap_completed=True, was False")
    assert "ground truth frozen at sim 110.51 s" in text
