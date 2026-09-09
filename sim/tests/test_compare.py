from __future__ import annotations

from copy import deepcopy

import pytest

from scripts.compare_metrics import compare_metrics, format_report


def metrics() -> dict:
    return {
        "header": {
            "stamp": {"sec": 0, "nanosec": 0},
            "frame_id": "map",
        },
        "source": "headless_sim",
        "scenario_id": "baseline",
        "seed": 42,
        "timestep_ratio": 1,
        "lap_completed": True,
        "lap_time": 12.0,
        "collision_count": 0,
        "minimum_wall_clearance": 0.2,
        "maximum_tracking_error": 0.1,
        "p95_tracking_error": 0.08,
        "control_saturation_events": 2,
        "code_revision": "abc1234",
        "container_image_digest": "sha256:expected",
        "track_version": "track-v1",
        "vehicle_parameter_version": "vehicle-v1",
        "platform": {
            "system": "Linux",
            "machine": "x86_64",
            "python": "3.12.0",
            "jax_backend": "gpu",
            "jax_version": "0.7.2",
        },
    }


def test_metrics_1020_honours_per_field_tolerances() -> None:
    expected = metrics()
    actual = deepcopy(expected)
    actual["lap_time"] += 0.049
    actual["minimum_wall_clearance"] -= 0.009
    actual["p95_tracking_error"] += 0.009

    assert compare_metrics(expected, actual) == []

    actual["p95_tracking_error"] = expected["p95_tracking_error"] + 0.011
    differences = compare_metrics(expected, actual)

    assert [difference.field for difference in differences] == [
        "p95_tracking_error"
    ]
    report = format_report(expected, actual, differences)
    assert "sha256:expected" in report
    assert "Linux/x86_64" in report


def test_metrics_1020_rejects_exact_float_comparison() -> None:
    with pytest.raises(ValueError, match="positive tolerance"):
        compare_metrics(metrics(), metrics(), {"lap_time": 0.0})


def test_metrics_1020_rejects_non_finite_and_missing_metrics() -> None:
    expected = metrics()
    actual = metrics()
    actual["lap_time"] = float("nan")
    del actual["maximum_tracking_error"]

    difference_fields = {
        difference.field for difference in compare_metrics(expected, actual)
    }
    assert difference_fields == {
        "lap_time",
        "maximum_tracking_error",
    }
