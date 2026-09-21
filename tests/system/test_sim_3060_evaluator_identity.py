"""SIM-3060: racing_evaluation fed ground truth *as* the estimate reports
error == 0 and full availability - the check that the ruler is straight.

It gates PR 3: an evaluator that misaligns time, mixes frames or drops
samples would put a bent ruler under every estimator measured after it,
and every number would look plausible. With the estimate identical to the
truth, anything but zero is the harness's own error.

Both halves are checked: the live node's running totals, as recorded, and
`scripts/compare_localization.py` rescoring the same recording offline.

Run on analytic_circle deliberately. repo-gotchas #18 forbids measuring
*localization* there, because scan matching cannot observe position along a
rotationally symmetric track; no scan matching happens here, and the
circle's lap is ~6.5x cheaper than Spielberg's.
"""

from pathlib import Path

from racing_test_keywords.scenario_runner import run_scenario

from scripts.compare_localization import (
    compare_localization,
    format_report,
    last_live_error,
    read_recording,
    score_recording,
)

GROUND_TRUTH_TOPIC = "/ground_truth/odom"
SEED = 42

# Bounds for an identity estimate are not measurements of a method: zero
# error and complete coverage are the definition of scoring a stream
# against itself.
IDENTITY_BOUNDS = {
    "ate_maximum": {
        "maximum": 0.0,
        "measurement": "identity: an estimate equal to truth has no error",
    },
    "rpe_maximum": {
        "maximum": 0.0,
        "measurement": "identity: an estimate equal to truth has no error",
    },
    "availability": {
        "minimum": 1.0,
        "measurement": "identity: truth covers its own span",
    },
}


def test_sim_3060_ground_truth_as_estimate_scores_zero(tmp_path: Path) -> None:
    recording = tmp_path / "identity.jsonl"

    metrics = run_scenario(
        seed=SEED,
        launch_arguments={
            "recording_path": str(recording),
            "estimate_topic": GROUND_TRUTH_TOPIC,
        },
    )
    assert metrics["lap_completed"]

    events = read_recording(recording)
    live = last_live_error(events)
    assert live["estimate_topic"] == GROUND_TRUTH_TOPIC
    # The lap is 22.8 s of 100 Hz truth: the evaluator must have scored it.
    assert live["sample_count"] > 2000
    assert live["available_count"] == live["sample_count"], live
    assert live["position_rmse"] == 0.0, live
    assert live["position_maximum"] == 0.0, live

    offline = score_recording(events, estimate_topic=GROUND_TRUTH_TOPIC)
    assert offline["scored_count"] == offline["sample_count"] > 2000
    violations = compare_localization(IDENTITY_BOUNDS, offline)
    assert violations == [], format_report(offline, violations)
