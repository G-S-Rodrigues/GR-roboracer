"""SIM-3100: the noise model actually degrades dead reckoning - /odom scored
against /ground_truth/odom over a lap diverges past a stated floor.

The guard against noise being silently disabled. With `noise:` removed from
a scenario, or its odometry sigmas set to zero, /odom is ground truth under
another frame name, every localization test in PR 3 becomes vacuous, and all
of them stay green. This is the only test that notices.

Scored with racing_evaluation itself: `estimate_topic:=/odom` makes the live
evaluator score dead reckoning, the recording carries both streams, and
`scripts/compare_localization.py` rescores them offline. Dead reckoning
starts at the true pose (odom == map at t=0), so its position in `odom` is
directly comparable with truth in `map`, and the difference is the drift.

On analytic_circle: dead-reckoning drift involves no scan matching, so
repo-gotchas #18's symmetry does not apply, and the lap is ~6.5x cheaper.
"""

from pathlib import Path

from racing_test_keywords.scenario_runner import run_scenario

from scripts.compare_localization import (
    compare_localization,
    format_report,
    read_recording,
    score_recording,
)

DEAD_RECKONING_TOPIC = "/odom"
SEED = 42

DRIFT_BOUNDS = {
    "ate_rmse": {
        "minimum": 0.1,
        "measurement": (
            "2026-09-21, contract_test.yaml, one lap at 1x: ATE RMSE 0.755 m "
            "(seed 42), 1.170 m (seed 7), 0.403 m (seed 1030); with the "
            "noise block removed, seed 42 scores 9.5e-15 m. 0.1 m sits 4x "
            "under the smallest seed and 13 orders of magnitude over the "
            "noise-free run."
        ),
    },
    "availability": {
        "minimum": 1.0,
        "measurement": (
            "2026-09-21: /odom publishes every tick; availability 1.0 on "
            "all four runs above, so drift is not a scoring gap"
        ),
    },
}


def test_sim_3100_dead_reckoning_drifts_from_ground_truth(
    tmp_path: Path,
) -> None:
    recording = tmp_path / "dead_reckoning.jsonl"

    metrics = run_scenario(
        seed=SEED,
        launch_arguments={
            "recording_path": str(recording),
            "estimate_topic": DEAD_RECKONING_TOPIC,
        },
    )
    assert metrics["lap_completed"]

    drift = score_recording(
        read_recording(recording), estimate_topic=DEAD_RECKONING_TOPIC
    )
    violations = compare_localization(DRIFT_BOUNDS, drift)
    assert violations == [], format_report(drift, violations)
