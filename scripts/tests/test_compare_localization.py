"""Tests for scripts.compare_localization's N1 queries: the estimate's error
at a moment (CHECK-2375) and when the estimator started (CHECK-2376)."""

from __future__ import annotations

import math

import pytest

from scripts import compare_localization

ERROR_TOPIC = "/evaluation/localization_error"


def _stamp(t: float) -> dict:
    return {"sec": int(t), "nanosec": round((t % 1) * 1e9)}


def error_event(t, lon, lat, topic="/localization/odom"):
    return {
        "record_type": "event",
        "stamp": _stamp(t),
        "topic": ERROR_TOPIC,
        "payload": {
            "estimate_topic": topic,
            "longitudinal_error": lon,
            "lateral_error": lat,
        },
    }


def test_check_2375_error_at_is_the_last_error_at_or_before_t() -> None:
    events = [
        error_event(110.30, 0.9, 0.01),
        error_event(110.40, 1.38, 0.02),
        error_event(110.60, 2.0, 0.03),
    ]
    found = compare_localization.error_at(events, 110.51)
    assert found["longitudinal_error"] == 1.38
    assert found["lateral_error"] == 0.02
    assert compare_localization.error_at(events, 110.0) is None


def test_check_2375_error_at_restores_nan_and_never_raises() -> None:
    events = [error_event(1.0, None, None)]
    found = compare_localization.error_at(events, 2.0)
    assert math.isnan(found["longitudinal_error"])
    assert compare_localization.error_at([], 2.0) is None


# ---- CHECK-2376: when the estimator started ----


def odom_event(t, x, topic):
    return {
        "record_type": "event",
        "stamp": _stamp(t),
        "topic": topic,
        "payload": {
            "x": x,
            "y": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        },
    }


def synthetic(first_estimate=1.03, with_error_event=True, speed=2.35):
    """Truth at 100 Hz from t=0 along x; the estimate from `first_estimate`."""
    events = []
    steps = 500
    for i in range(steps):
        t = i / 100
        events.append(
            odom_event(t, speed * t, compare_localization.GROUND_TRUTH_TOPIC)
        )
        if first_estimate is not None and t >= first_estimate - 1e-9:
            events.append(odom_event(t, speed * t, "/localization/odom"))
    if with_error_event:
        events.append(error_event(2.0, 0.0, 0.0))
    return events


def test_check_2376_estimator_start_is_the_first_estimate_and_travel():
    start = compare_localization.estimator_start(synthetic())
    assert start["first_estimate_stamp"] == pytest.approx(1.03)
    assert start["travel_before_first_estimate"] == pytest.approx(
        2.42, abs=0.03
    )
    assert set(start) == {
        "first_estimate_stamp",
        "travel_before_first_estimate",
    }


def test_check_2376_an_estimate_from_t0_has_no_travel():
    start = compare_localization.estimator_start(synthetic(first_estimate=0.0))
    assert start["first_estimate_stamp"] == 0.0
    assert start["travel_before_first_estimate"] == 0.0


def test_check_2376_no_estimate_is_none_not_prompt():
    """A dead estimator must not look like a prompt one (EVAL-1030)."""
    start = compare_localization.estimator_start(synthetic(first_estimate=None))
    assert start == {
        "first_estimate_stamp": None,
        "travel_before_first_estimate": None,
    }


def test_check_2376_it_never_raises_without_an_error_event():
    events = synthetic(with_error_event=False)
    start = compare_localization.estimator_start(events)
    assert start == {
        "first_estimate_stamp": None,
        "travel_before_first_estimate": None,
    }
    named = compare_localization.estimator_start(events, "/localization/odom")
    assert named["first_estimate_stamp"] == pytest.approx(1.03)


def test_check_2376_score_localization_merges_both_keys(monkeypatch):
    import sys
    from pathlib import Path

    lib = Path(__file__).resolve().parents[2] / "tests" / "lib"
    if str(lib) not in sys.path:
        sys.path.insert(0, str(lib))
    from racing_test_keywords import scenario_runner

    events = synthetic()
    monkeypatch.setattr(
        compare_localization, "read_recording", lambda p: events
    )
    score = scenario_runner.score_localization(
        "x.jsonl", estimate_topic="/localization/odom"
    )
    assert score["first_estimate_stamp"] == pytest.approx(1.03)
    assert "travel_before_first_estimate" in score
    assert "ate_rmse" in score
