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
    "maps/Spielberg/mola/Spielberg.mm (mm_sha256 4a89722..., mapping seed 7), "
    "5 full laps on the dev host (load 1.9-4.2): ate_rmse 1.547-1.556 m "
    "(median 1.552), ate_maximum 3.076-3.097 m, availability "
    "0.99743-0.99925; every lap complete, 0 collisions. Includes MOLA's "
    "own map error and the T alignment, so a regenerated map is a reason "
    "to re-measure (SIM-5520 flags drift)"
)
# Headroom: rmse 1.61x and maximum 1.45x of the worst run, availability's
# floor 0.99 (worst 0.9974). The spread over five laps is 0.009 m: the run
# is near-deterministic, so the headroom covers machine and map differences.
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
