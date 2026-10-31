"""SIM-3510: `pose_source:=mola` - MOLA-LO owns `map -> odom` for a real
Spielberg lap against the map it built itself, and the pose it puts under
the stack stays available and bounded, scored live by racing_evaluation and
again offline by `scripts/compare_localization.py`.

This measures localization **on MOLA's own map** (owner ruling 2026-10-25):
`config/scenarios/maps/Spielberg/mola/`, from one seed-7 mapping lap with no
loop closure (`scripts/mola_map_track.sh`). The error therefore includes
MOLA's own map error (drift where the lap closes, wall offsets) and the
alignment T from the map frame to the world frame (its residual is in the
provenance yaml). It is not comparable term for term with SIM-3080, which
localizes on a survey. `/initialpose` comes from truth, as amcl's does.

Open loop (`drive_on_estimate=False`) on the 1080-beam scenario, at the
scale MOLA was measured at (1x: its scan matching is wall-clock work inside
a simulated-time run). Spielberg, never analytic_circle (repo-gotchas #18).
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

SCENARIO = "config/scenarios/spielberg_1080.yaml"
ESTIMATE_TOPIC = "/localization/odom"
SEED = 42
TIME_SCALE = 1.0
TIMEOUT_SECONDS = 400.0

MEASUREMENT = (
    "SIM-3510, Spielberg 1080 beams, seed 42, 1x, open loop, on "
    "maps/Spielberg/mola/Spielberg.mm (mm_sha256 5b4ec52..., mapping seed 7), "
    "localization icp_prior_weight 5.0, 5 full laps on the dev host at "
    "1-minute load 7.7-8.0 at each start (2026-10-30): ate_rmse 0.704, "
    "1.944, 2.071, 2.091, 2.131 m, ate_maximum 1.414-4.149 m, availability "
    "0.99751-0.99792; every lap complete. Includes MOLA's own map error and "
    "the T alignment, so a regenerated map is a reason to re-measure "
    "(SIM-5520 flags drift)"
)
# Headroom: rmse 1.17x and maximum 1.08x of the worst run, availability's
# floor 0.99 (worst 0.9975). The bounds were set on the previous map
# (rmse 1.547-1.556 m, maximum 3.076-3.097 m, five laps at load 1.9-4.2) and
# are unchanged. On this map the outcome is not near-deterministic: at host
# load under 4 the same configuration gave rmse 0.742-0.752 m (3 runs), at
# load near 8 it gave 0.70 once and 1.94-2.13 four times, so the margin to the
# bound is thin under load. At icp_prior_weight 1.0 (the default) this map
# passed 2 of 5 and diverged (37.7 m) in 3.
POSITION_RMSE_MAXIMUM = 2.5
POSITION_MAXIMUM = 4.5
AVAILABILITY_MINIMUM = 0.99
OFFLINE_BOUNDS = {
    "ate_maximum": {"maximum": POSITION_MAXIMUM, "measurement": MEASUREMENT},
    "availability": {
        "minimum": AVAILABILITY_MINIMUM,
        "measurement": MEASUREMENT,
    },
}


def test_sim_3510_mola_tracks_ground_truth(tmp_path: Path) -> None:
    recording = tmp_path / "mola.jsonl"

    metrics = run_scenario(
        seed=SEED,
        scenario=SCENARIO,
        time_scale=TIME_SCALE,
        timeout=TIMEOUT_SECONDS,
        pose_source="mola",
        drive_on_estimate=False,
        launch_arguments={"recording_path": str(recording)},
    )
    assert metrics["lap_completed"], metrics
    assert metrics["collision_count"] == 0, metrics

    events = read_recording(recording)
    live = last_live_error(events)
    assert live["estimate_topic"] == ESTIMATE_TOPIC
    # A lap of 100 Hz truth at 1x: ~12 000 samples.
    assert live["sample_count"] > 10000, live
    availability = live["available_count"] / live["sample_count"]
    assert availability >= AVAILABILITY_MINIMUM, live
    assert live["position_rmse"] <= POSITION_RMSE_MAXIMUM, live
    assert live["position_maximum"] <= POSITION_MAXIMUM, live

    offline = score_recording(events, estimate_topic=ESTIMATE_TOPIC)
    violations = compare_localization(OFFLINE_BOUNDS, offline)
    assert violations == [], format_report(offline, violations)
