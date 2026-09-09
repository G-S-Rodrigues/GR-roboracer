"""Robot Framework keyword library wrapping the scenario runner for tier 4.

Kept a thin adapter over :mod:`racing_test_keywords.scenario_runner` — the
scenario-running logic itself is exercised directly (and more cheaply, with
per-field assertions) by the tier-3 pytest suite in ``tests/system/``. This
library only adds the Robot Framework keyword surface tier-4 acceptance specs
read like plain English.
"""

from __future__ import annotations

from typing import Any

from racing_interfaces.msg import SafetyStatus

from .scenario_runner import (
    run_scenario,
    run_scenario_with_track_limit_violation,
)


class RacingTestKeywords:
    """Robot Framework library: run a racing scenario, assert on its metrics."""

    ROBOT_LIBRARY_SCOPE = "TEST"

    def __init__(self) -> None:
        self._metrics: dict[str, Any] | None = None
        self._violation: dict[str, Any] | None = None

    def run_racing_scenario(
        self, scenario: str | None = None, seed: int = 0
    ) -> None:
        """Run the full stack headlessly for one seeded scenario."""
        self._metrics = run_scenario(seed=int(seed), scenario=scenario)

    def violate_track_limits_during_scenario(
        self, scenario: str | None = None, seed: int = 0
    ) -> None:
        """Run the full stack, then force the vehicle off-track."""
        self._violation = run_scenario_with_track_limit_violation(
            seed=int(seed), scenario=scenario
        )

    def _require_metrics(self) -> dict[str, Any]:
        if self._metrics is None:
            raise RuntimeError("Run Racing Scenario has not been called yet")
        return self._metrics

    def _require_violation(self) -> dict[str, Any]:
        if self._violation is None:
            raise RuntimeError(
                "Violate Track Limits During Scenario has not been called yet"
            )
        return self._violation

    def lap_completion_should_be(self, expected: bool) -> None:
        actual = self._require_metrics()["lap_completed"]
        expected_bool = str(expected).strip().lower() in ("true", "1", "yes")
        if actual != expected_bool:
            raise AssertionError(
                f"expected lap_completed={expected_bool}, was {actual}"
            )

    def collision_count_should_be(self, expected: int) -> None:
        actual = self._require_metrics()["collision_count"]
        if actual != int(expected):
            raise AssertionError(
                f"expected collision_count={expected}, was {actual}"
            )

    def minimum_wall_clearance_should_exceed(self, threshold: float) -> None:
        actual = self._require_metrics()["minimum_wall_clearance"]
        if not actual > float(threshold):
            raise AssertionError(
                f"expected minimum_wall_clearance > {threshold}, was {actual}"
            )

    def p95_tracking_error_should_be_below(self, threshold: float) -> None:
        actual = self._require_metrics()["p95_tracking_error"]
        if not actual < float(threshold):
            raise AssertionError(
                f"expected p95_tracking_error < {threshold}, was {actual}"
            )

    def safety_clamp_should_include_track_limit(self) -> None:
        active_clamps = self._require_violation()["active_clamps"]
        if not active_clamps & SafetyStatus.CLAMP_TRACK_LIMIT:
            raise AssertionError(
                "expected CLAMP_TRACK_LIMIT in active_clamps, "
                f"was {active_clamps}"
            )

    def commanded_speed_should_be_zero(self) -> None:
        speed = self._require_violation()["speed"]
        if speed != 0.0:
            raise AssertionError(f"expected commanded speed 0.0, was {speed}")
