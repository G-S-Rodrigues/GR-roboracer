"""Robot Framework keyword library wrapping the scenario runner for tier 4.

Kept a thin adapter over :mod:`racing_test_keywords.scenario_runner` — the
scenario-running logic itself is exercised directly (and more cheaply, with
per-field assertions) by the tier-3 pytest suite in ``tests/system/``. This
library only adds the Robot Framework keyword surface tier-4 acceptance specs
read like plain English.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from racing_interfaces.msg import SafetyStatus
from robot.libraries.BuiltIn import BuiltIn

from .report import CATASTROPHIC_BOUNDS, write_report
from .scenario_runner import (
    REPOSITORY_ROOT,
    run_scenario,
    run_scenario_with_track_limit_violation,
    score_localization,
)

# Every localization case runs on Spielberg: a rotationally symmetric
# track leaves position along the centerline unobservable, so the same
# case on analytic_circle would pass while measuring nothing
# (repo-gotchas #18).
LOCALIZATION_SCENARIO = "config/scenarios/spielberg.yaml"
# The scale SIM-3070/3080/3090 run Spielberg at, and the scale every bound
# these keywords are given was measured at (ADR 0006).
LOCALIZATION_TIME_SCALE = 5.0
LOCALIZATION_TIMEOUT_SECONDS = 400.0


def _output_root() -> Path:
    """Where this run's artifacts are kept: Robot's `${OUTPUT DIR}`, which
    `check.sh` and CI point at `log/robot`, else `<repo>/log/robot` when a
    keyword is called outside Robot."""
    try:
        directory = BuiltIn().get_variable_value("${OUTPUT DIR}")
    except Exception:
        directory = None
    if directory:
        return Path(directory)
    return REPOSITORY_ROOT / "log" / "robot"


class RacingTestKeywords:
    """Robot Framework library: run a racing scenario, assert on its metrics."""

    ROBOT_LIBRARY_SCOPE = "TEST"

    def __init__(self) -> None:
        self._metrics: dict[str, Any] | None = None
        self._violation: dict[str, Any] | None = None
        self._localization: dict[str, Any] | None = None
        self._recording: Path | None = None
        # Seed, scenario and implementation of the last run, for reports.
        self._run_info: dict[str, Any] = {}

    def run_racing_scenario(
        self, scenario: str | None = None, seed: int = 0
    ) -> None:
        """Run the full stack headlessly for one seeded scenario."""
        self._metrics = run_scenario(seed=int(seed), scenario=scenario)
        self._run_info = {
            "seed": int(seed),
            "scenario": scenario or "default",
            "implementation": "ground_truth",
        }

    def violate_track_limits_during_scenario(
        self, scenario: str | None = None, seed: int = 0
    ) -> None:
        """Run the full stack, then force the vehicle off-track."""
        self._violation = run_scenario_with_track_limit_violation(
            seed=int(seed), scenario=scenario
        )

    def run_racing_scenario_on_pose_source(
        self,
        pose_source: str,
        seed: int = 42,
        drive_on_estimate: str = "false",
        scenario: str = LOCALIZATION_SCENARIO,
        time_scale: str = str(LOCALIZATION_TIME_SCALE),
        planner: str | None = None,
        raceline_optimizer: str | None = None,
        speed_source: str | None = None,
    ) -> None:
        """Run one Spielberg lap with `pose_source` owning map -> odom.

        Open loop by default: the controller keeps ground truth and the
        estimator is scored beside it, which is the only way a pose source
        the car cannot yet drive on (slam_toolbox, plan D7) is measurable
        at all. `drive_on_estimate=true` closes the loop. `scenario` and
        `time_scale` default to the amcl/slam_toolbox lap; MOLA runs the
        1080-beam scenario at 1x (its scan matching is wall-clock work).
        `planner`, `raceline_optimizer` and `speed_source` are passed to the
        launch only when given; a planner is also named in the recording
        (`<source>-<kind>-loop-<planner>.jsonl`) so two cases on one pose
        source keep separate evidence.
        """
        scale = float(time_scale)
        closed_loop = str(drive_on_estimate).strip().lower() in (
            "true",
            "1",
            "yes",
        )
        # Kept, not temporary: a failed availability or error assert is
        # only explicable from the recording's stamps (nightly 39568107208
        # lost about 2.27 s of sim time beyond the 0.5 s window, i.e. one
        # ~2.8 s hole or several, and the evidence with it).
        kind = "closed" if closed_loop else "open"
        suffix = f"-{planner}" if planner else ""
        recording = (
            _output_root()
            / "localization"
            / f"{pose_source}-{kind}-loop{suffix}.jsonl"
        )
        recording.parent.mkdir(parents=True, exist_ok=True)
        recording.unlink(missing_ok=True)
        self._recording = recording
        launch_arguments = {"recording_path": str(recording)}
        for key, value in (
            ("planner", planner),
            ("raceline_optimizer", raceline_optimizer),
            ("speed_source", speed_source),
        ):
            if value is not None:
                launch_arguments[key] = value
        self._metrics = run_scenario(
            seed=int(seed),
            scenario=scenario,
            time_scale=scale,
            timeout=LOCALIZATION_TIMEOUT_SECONDS * max(1.0, 5.0 / scale),
            pose_source=pose_source,
            drive_on_estimate=closed_loop,
            launch_arguments=launch_arguments,
        )
        self._localization = score_localization(recording)
        self._run_info = {
            "seed": int(seed),
            "scenario": scenario,
            # the planner too, when one is chosen: ACC-4040's raceline case
            # exercises the planner on top of the pose source
            "implementation": (
                pose_source if planner is None else f"{pose_source}/{planner}"
            ),
        }

    def _require_localization(self) -> dict[str, Any]:
        if self._localization is None:
            raise RuntimeError(
                "Run Racing Scenario On Pose Source has not been called yet"
            )
        return self._localization

    def _localization_summary(self) -> str:
        """Where the score lost its coverage, for a failed assert's text."""
        score = self._require_localization()
        fields = (
            "maximum_gap",
            "stale_gap_count",
            "sample_count",
            "scored_count",
            "ate_rmse",
        )
        detail = ", ".join(f"{name}={score.get(name)}" for name in fields)
        return f" ({detail}; recording {self._recording})" + (
            self._escalation_note()
        )

    def localization_error_should_be_below(self, threshold: float) -> None:
        """The estimate's ATE RMSE over the lap, scored offline."""
        actual = self._require_localization()["ate_rmse"]
        if not actual < float(threshold):
            raise AssertionError(
                f"expected ate_rmse < {threshold}, was {actual}"
                + self._localization_summary()
            )

    def localization_availability_should_exceed(self, threshold: float) -> None:
        """A stalled estimator scores as unavailable, never as zero error
        (EVAL-1030), so availability is asserted before any error is."""
        actual = self._require_localization()["availability"]
        if not actual > float(threshold):
            raise AssertionError(
                f"expected availability > {threshold}, was {actual}"
                + self._localization_summary()
            )

    def localization_should_stay_within_catastrophic_bounds(self) -> None:
        """The only quality assert on a localization case (ADR 0008): the
        estimate is not lost (ATE RMSE) and not stalled (availability). The
        strict `<` / `>` of the two keywords it calls are kept."""
        self.localization_availability_should_exceed(
            CATASTROPHIC_BOUNDS["availability"]["minimum"]
        )
        self.localization_error_should_be_below(
            CATASTROPHIC_BOUNDS["ate_rmse"]["maximum"]
        )

    def write_localization_report(self, report_id: str) -> None:
        """Report the offline score; called before any assert, so a red
        case keeps its numbers."""
        write_report(
            str(report_id),
            self._require_localization(),
            baseline=None,
            diagnostics=self._run_diagnostics(),
            **self._report_context(),
        )

    def write_metrics_report(self, report_id: str) -> None:
        """Report the scenario metrics (lap time, clearance, tracking)."""
        write_report(
            str(report_id),
            self._require_metrics(),
            baseline=None,
            diagnostics=self._run_diagnostics(),
            **self._report_context(),
        )

    def _run_diagnostics(self) -> dict[str, Any]:
        """What the run itself learned, for a report's `diagnostics`. Read
        with getattr: a stubbed runner returns a plain dict (CHECK-1400).
        Never raises: a diagnostic must not turn a green run red."""
        try:
            metrics = self._metrics
            frozen_at = getattr(metrics, "frozen_at", None)
            return {
                "shutdown_escalations": list(
                    getattr(metrics, "shutdown_escalations", None) or []
                ),
                "frozen_at": None if frozen_at is None else list(frozen_at),
            }
        except Exception:  # noqa: BLE001
            return {"shutdown_escalations": [], "frozen_at": None}

    def _escalation_note(self) -> str:
        """`; shutdown escalations: <name> SIGKILL, ...` for a failure's text,
        or nothing when launch escalated nothing (the text stays today's)."""
        try:
            escalations = (
                getattr(self._metrics, "shutdown_escalations", None) or []
            )
            if not escalations:
                return ""
            listed = ", ".join(
                f"{e['name']} {e['signal']}" for e in escalations
            )
            return f"; shutdown escalations: {listed}"
        except Exception:  # noqa: BLE001
            return "; shutdown escalations: (diagnostic unavailable)"

    def _report_context(self) -> dict[str, Any]:
        info = self._run_info
        return {
            "seed": info.get("seed"),
            "scenario": info.get("scenario", "unknown"),
            "implementation": info.get("implementation", "unknown"),
        }

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
                + self._escalation_note()
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
        violation = self._require_violation()
        active_clamps = violation["active_clamps"]
        if not active_clamps & SafetyStatus.CLAMP_TRACK_LIMIT:
            raise AssertionError(
                "expected CLAMP_TRACK_LIMIT in active_clamps, "
                f"was {active_clamps}; "
                f"safety: {violation.get('safety_record', 'none')}"
            )

    def commanded_speed_should_be_zero(self) -> None:
        violation = self._require_violation()
        speed = violation["speed"]
        if speed != 0.0:
            raise AssertionError(
                f"expected commanded speed 0.0, was {speed}; "
                f"safety: {violation.get('safety_record', 'none')}"
            )
