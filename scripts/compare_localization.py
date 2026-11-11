#!/usr/bin/env python3
"""Score a recorded run's pose estimate against its ground truth, offline.

The live evaluator (racing_evaluation) and racing_recording's JSONL carry the
same streams, so a lap run once can be rescored here in milliseconds, as many
ways as needed - the cheap half of the live/offline split that keeps long
runs affordable.

Mirrors ``compare_metrics.py``'s shape without sharing its code: the
semantics differ. A metrics record is held *near* a golden; a localization
score is held *inside a bound* - an error below a maximum, availability
above a minimum - because an estimate that beats its bound is not a
regression. Bounds are per method and belong beside the method (PR 3), so
this file carries no tolerance table of its own. What it does enforce is
repo-gotchas #14's rule: every bound states the measurement that justifies
it, or it is rejected.
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from racing_evaluation.localization import (
    DEFAULT_WINDOW,
    Pose,
    absolute_trajectory_error,
    availability,
    relative_pose_error,
)

GROUND_TRUTH_TOPIC = "/ground_truth/odom"
ERROR_TOPIC = "/evaluation/localization_error"
# RPE's interval: one second is ~3 m of travel at the reference stack's
# 3.0 m/s, several scan-matching updates for any estimator.
DEFAULT_RPE_DELTA = 1.0

SCORE_FIELDS = (
    "ate_rmse",
    "ate_mean",
    "ate_maximum",
    "rpe_rmse",
    "rpe_maximum",
    "availability",
    "maximum_gap",
    "stale_gap_count",
    "sample_count",
    "scored_count",
)


@dataclass(frozen=True)
class Violation:
    field: str
    bound: str
    limit: float
    actual: float


def read_recording(path: Path) -> list[dict[str, Any]]:
    """Every event record of a racing_recording JSONL file."""
    events = []
    with path.open(encoding="utf-8") as recording:
        for line in recording:
            record = json.loads(line)
            if record.get("record_type") == "event":
                events.append(record)
    return events


def _stamp(event: Mapping[str, Any]) -> float:
    return event["stamp"]["sec"] + event["stamp"]["nanosec"] * 1e-9


def poses(events: Iterable[Mapping[str, Any]], topic: str) -> list[Pose]:
    """The recorded nav_msgs/Odometry stream on ``topic``, as planar poses."""
    result = []
    for event in events:
        if event["topic"] != topic:
            continue
        payload = event["payload"]
        qx, qy, qz, qw = (payload[key] for key in ("qx", "qy", "qz", "qw"))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
        result.append(Pose(_stamp(event), payload["x"], payload["y"], yaw))
    return result


def recorded_estimate_topic(events: Iterable[Mapping[str, Any]]) -> str:
    """The estimate topic the live evaluator scored during the run."""
    for event in events:
        if event["topic"] == ERROR_TOPIC:
            return event["payload"]["estimate_topic"]
    raise ValueError(f"no {ERROR_TOPIC} events; pass --estimate-topic")


def last_live_error(events: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    """The live evaluator's final running totals, NaN restored from null."""
    last = None
    for event in events:
        if event["topic"] == ERROR_TOPIC:
            last = event["payload"]
    if last is None:
        raise ValueError(f"no {ERROR_TOPIC} events in the recording")
    return {
        key: math.nan if value is None else value for key, value in last.items()
    }


def estimator_start(
    events: Iterable[Mapping[str, Any]], estimate_topic: str | None = None
) -> dict[str, float | None]:
    """When the estimate stream began and how far the car had driven by then.

    ``first_estimate_stamp`` is the sim time of the first estimate;
    ``travel_before_first_estimate`` the ground-truth path length (m) up to
    it. Both are ``None`` when there was no estimate (or no topic to look
    for), so a dead estimator does not look like a prompt one (EVAL-1030).
    Never raises. Nightly 2026-11-10: MOLA's first estimate came 1.03 s
    after the car left, about 2.5 m on, and ATE RMSE was 21.5 m."""
    events = list(events)
    none = {"first_estimate_stamp": None, "travel_before_first_estimate": None}
    try:
        topic = estimate_topic or recorded_estimate_topic(events)
    except ValueError:
        return none
    estimates = poses(events, topic)
    if not estimates:
        return none
    first = min(pose.stamp for pose in estimates)
    travel = 0.0
    previous = None
    for pose in poses(events, GROUND_TRUTH_TOPIC):
        if pose.stamp > first:
            break
        if previous is not None:
            travel += math.hypot(pose.x - previous.x, pose.y - previous.y)
        previous = pose
    return {
        "first_estimate_stamp": first,
        "travel_before_first_estimate": travel,
    }


def error_at(
    events: Iterable[Mapping[str, Any]], t: float
) -> dict[str, Any] | None:
    """The live evaluator's last error at or before sim time ``t`` (NaN
    restored from null), or ``None`` when it had published none by then."""
    last = None
    for event in events:
        if event["topic"] == ERROR_TOPIC and _stamp(event) <= t:
            last = event["payload"]
    if last is None:
        return None
    return {
        key: math.nan if value is None else value for key, value in last.items()
    }


def score(
    estimates: list[Pose],
    truth: list[Pose],
    window: float = DEFAULT_WINDOW,
    rpe_delta: float = DEFAULT_RPE_DELTA,
) -> dict[str, float]:
    """One estimate stream against its truth, over truth's span."""
    if len(truth) < 2:
        raise ValueError("ground truth needs at least two samples")
    ate = absolute_trajectory_error(estimates, truth, window)
    rpe = relative_pose_error(estimates, truth, rpe_delta, window)
    coverage = availability(
        estimates,
        window,
        start=min(p.stamp for p in truth),
        end=max(p.stamp for p in truth),
    )
    return {
        "ate_rmse": ate.rmse,
        "ate_mean": ate.mean,
        "ate_maximum": ate.maximum,
        "rpe_rmse": rpe.rmse,
        "rpe_maximum": rpe.maximum,
        "availability": coverage.availability,
        "maximum_gap": coverage.maximum_gap,
        "stale_gap_count": coverage.stale_gap_count,
        "sample_count": ate.sample_count,
        "scored_count": ate.scored_count,
    }


def score_recording(
    events: list[dict[str, Any]],
    estimate_topic: str | None = None,
    truth_topic: str = GROUND_TRUTH_TOPIC,
    window: float = DEFAULT_WINDOW,
    rpe_delta: float = DEFAULT_RPE_DELTA,
) -> dict[str, Any]:
    topic = estimate_topic or recorded_estimate_topic(events)
    result: dict[str, Any] = {"estimate_topic": topic}
    result.update(
        score(
            poses(events, topic),
            poses(events, truth_topic),
            window,
            rpe_delta,
        )
    )
    return result


def compare_localization(
    bounds: Mapping[str, Mapping[str, Any]], actual: Mapping[str, Any]
) -> list[Violation]:
    """Bounds are {field: {"maximum"|"minimum": value, "measurement": why}}.

    A NaN score violates every bound: nothing scored is never in bounds.
    """
    violations: list[Violation] = []
    for field, bound in bounds.items():
        if field not in SCORE_FIELDS:
            raise ValueError(f"unknown localization score field: {field}")
        if not str(bound.get("measurement", "")).strip():
            raise ValueError(f"{field}: a bound must state its measurement")
        limits = {
            key: bound[key] for key in ("maximum", "minimum") if key in bound
        }
        if not limits:
            raise ValueError(f"{field}: a bound needs a maximum or minimum")
        value = float(actual[field])
        for kind, limit in limits.items():
            inside = (
                value <= limit if kind == "maximum" else value >= limit
            ) and not math.isnan(value)
            if not inside:
                violations.append(Violation(field, kind, float(limit), value))
    return violations


def format_report(
    actual: Mapping[str, Any], violations: Iterable[Violation]
) -> str:
    lines = [f"estimate_topic: {actual['estimate_topic']}"]
    lines.extend(f"{field}: {actual[field]!r}" for field in SCORE_FIELDS)
    violations = list(violations)
    if not violations:
        lines.append("localization within bounds")
    for violation in violations:
        lines.append(
            f"{violation.field}: {violation.actual!r} outside "
            f"{violation.bound} {violation.limit!r}"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bounds", type=Path)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--estimate-topic")
    parser.add_argument("--truth-topic", default=GROUND_TRUTH_TOPIC)
    parser.add_argument("--window", type=float, default=DEFAULT_WINDOW)
    parser.add_argument("--rpe-delta", type=float, default=DEFAULT_RPE_DELTA)
    args = parser.parse_args()

    bounds = json.loads(args.bounds.read_text(encoding="utf-8"))
    actual = score_recording(
        read_recording(args.recording),
        args.estimate_topic,
        args.truth_topic,
        args.window,
        args.rpe_delta,
    )
    violations = compare_localization(bounds, actual)
    print(format_report(actual, violations))
    return 1 if violations else 0


if __name__ == "__main__":
    raise SystemExit(main())
