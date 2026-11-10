"""Tests for scripts.compare_localization's N1 queries: the estimate's error
at a moment (CHECK-2375) and when the estimator started (CHECK-2376)."""

from __future__ import annotations

import math

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
