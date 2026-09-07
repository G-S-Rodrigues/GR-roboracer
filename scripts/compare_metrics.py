#!/usr/bin/env python3
"""Compare two ScenarioMetrics JSON records without exact float equality."""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

FLOAT_TOLERANCES = {
    "lap_time": 0.05,
    "minimum_wall_clearance": 0.01,
    "maximum_tracking_error": 0.01,
    "p95_tracking_error": 0.01,
}

EXACT_FIELDS = (
    "source",
    "scenario_id",
    "seed",
    "timestep_ratio",
    "lap_completed",
    "collision_count",
    "control_saturation_events",
    "track_version",
    "vehicle_parameter_version",
)

_MISSING = object()


@dataclass(frozen=True)
class Difference:
    field: str
    expected: Any
    actual: Any
    tolerance: float | None = None


def _validated_tolerances(
    overrides: Mapping[str, float] | None,
) -> dict[str, float]:
    tolerances = dict(FLOAT_TOLERANCES)
    if overrides:
        unknown = set(overrides) - set(tolerances)
        if unknown:
            names = ", ".join(sorted(unknown))
            raise ValueError(f"unknown float tolerance fields: {names}")
        tolerances.update(overrides)
    for field, tolerance in tolerances.items():
        if tolerance <= 0.0:
            raise ValueError(f"{field} requires a positive tolerance")
    return tolerances


def compare_metrics(
    expected: Mapping[str, Any],
    actual: Mapping[str, Any],
    tolerances: Mapping[str, float] | None = None,
) -> list[Difference]:
    """Return semantic differences; provenance fields are report context."""
    float_tolerances = _validated_tolerances(tolerances)
    differences: list[Difference] = []

    for field in EXACT_FIELDS:
        expected_value = expected.get(field, _MISSING)
        actual_value = actual.get(field, _MISSING)
        if expected_value is _MISSING or actual_value is _MISSING:
            differences.append(
                Difference(
                    field,
                    (
                        "<missing>"
                        if expected_value is _MISSING
                        else expected_value
                    ),
                    "<missing>" if actual_value is _MISSING else actual_value,
                )
            )
        elif expected_value != actual_value:
            differences.append(Difference(field, expected_value, actual_value))

    for field, tolerance in float_tolerances.items():
        expected_value = expected.get(field, _MISSING)
        actual_value = actual.get(field, _MISSING)
        if expected_value is _MISSING or actual_value is _MISSING:
            differences.append(
                Difference(
                    field,
                    (
                        "<missing>"
                        if expected_value is _MISSING
                        else expected_value
                    ),
                    "<missing>" if actual_value is _MISSING else actual_value,
                    tolerance,
                )
            )
            continue
        expected_float = float(expected_value)
        actual_float = float(actual_value)
        if (
            not math.isfinite(expected_float)
            or not math.isfinite(actual_float)
            or abs(actual_float - expected_float) > tolerance
        ):
            differences.append(
                Difference(field, expected_value, actual_value, tolerance)
            )

    return differences


def _platform_label(record: Mapping[str, Any]) -> str:
    platform = record.get("platform", {})
    return (
        f"{platform.get('system', '?')}/{platform.get('machine', '?')} "
        f"python={platform.get('python', '?')} "
        f"jax={platform.get('jax_version', '?')} "
        f"backend={platform.get('jax_backend', '?')}"
    )


def format_report(
    expected: Mapping[str, Any],
    actual: Mapping[str, Any],
    differences: Sequence[Difference],
) -> str:
    lines = [
        "expected provenance: "
        f"image={expected.get('container_image_digest', '?')} "
        f"platform={_platform_label(expected)}",
        "actual provenance: "
        f"image={actual.get('container_image_digest', '?')} "
        f"platform={_platform_label(actual)}",
    ]
    if not differences:
        lines.append("metrics match within tolerance")
        return "\n".join(lines)

    for difference in differences:
        suffix = (
            ""
            if difference.tolerance is None
            else f" tolerance={difference.tolerance}"
        )
        lines.append(
            f"{difference.field}: expected={difference.expected!r} "
            f"actual={difference.actual!r}{suffix}"
        )
    return "\n".join(lines)


def _parse_tolerance(value: str) -> tuple[str, float]:
    try:
        field, raw_tolerance = value.split("=", 1)
        return field, float(raw_tolerance)
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected FIELD=VALUE") from error


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("expected", type=Path)
    parser.add_argument("actual", type=Path)
    parser.add_argument(
        "--tolerance",
        action="append",
        default=[],
        type=_parse_tolerance,
        metavar="FIELD=VALUE",
    )
    args = parser.parse_args()

    expected = json.loads(args.expected.read_text(encoding="utf-8"))
    actual = json.loads(args.actual.read_text(encoding="utf-8"))
    differences = compare_metrics(expected, actual, dict(args.tolerance))
    print(format_report(expected, actual, differences))
    return 1 if differences else 0


if __name__ == "__main__":
    raise SystemExit(main())
