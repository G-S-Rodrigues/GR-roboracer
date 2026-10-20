"""SIM-3080: `pose_source:=amcl` - nav2_amcl owns `map -> odom` for a real
Spielberg lap against the track's committed occupancy grid, and the pose it
puts under the stack stays available and bounded, scored live by
racing_evaluation and again offline by `scripts/compare_localization.py`.

The same claim as SIM-3070, the same harness, the same comparator, a
different implementation underneath - which is what makes the pair evidence
that the harness is method-neutral rather than evidence about one
estimator (ADR 0005).

Open loop (`drive_on_estimate=False`), for the same reason SIM-3070 is: the
lap is then the reference stack's own 120.44 s and the estimator is
measured beside it, so this test's numbers are comparable to SIM-3070's and
to SIM-3090's lap. That the car *can* drive on this estimate is ACC-4040's
claim, not this one's.

Spielberg, never analytic_circle: a rotationally symmetric track makes
position along the centerline unobservable to any scan matcher or particle
filter, and this test would pass on it while measuring nothing
(repo-gotchas #18).
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

SCENARIO = "config/scenarios/spielberg.yaml"
ESTIMATE_TOPIC = "/localization/odom"
SEED = 42
# The scale SIM-3090's Spielberg lap runs at (ADR 0006, plan D1), and the
# scale every bound here was measured at: AMCL's update is wall-clock work
# inside a simulated-time run, so a bound measured at 1x would not hold.
TIME_SCALE = 5.0
TIMEOUT_SECONDS = 400.0

# This is the SEED-42 bound (D4). SIM-5030 runs this same lap RUNS_PER_SEED
# times per seed at this scale and reports the spread - a particle filter
# has no seed, so a bound from a single run would be a bound on that run's
# RNG. SIM-5030's own multi-seed bounds are wider (it includes the bad
# odometry-draw seeds 3 and 123) and are not shared with this test.
# SIM_5030_MEASUREMENT below is the seed-42 sample the 0.65 rests on.
SIM_5030_MEASUREMENT = (
    "SIM-5030 seed-42 slice, Spielberg, 5x, open loop: original ten runs "
    "ate_rmse 0.215-0.437 m (spread 0.223, mean 0.314), ate_maximum "
    "0.736-1.834 m, availability 0.99411-0.99900; post-3.1b, 5 runs each: "
    "dev host 0.179-0.306 (median 0.259), CI 39423332965 0.141-0.279 "
    "(median 0.220), CI 39423349401 0.264-0.356 (median 0.286)"
)
# SIM-5030's worst seed-42 run was 0.437 in the original sample, 0.570 in the
# N1 CI samples and 0.356 post-3.1b; this bound's headroom is 1.14x at worst.
POSITION_RMSE_MAXIMUM = 0.65
POSITION_MAXIMUM = 2.6  # SIM-5030's worst of ten: 1.834
AVAILABILITY_MINIMUM = 0.99  # SIM-5030's worst of ten: 0.99411
OFFLINE_BOUNDS = {
    "ate_maximum": {
        "maximum": POSITION_MAXIMUM,
        "measurement": SIM_5030_MEASUREMENT,
    },
    "availability": {
        "minimum": AVAILABILITY_MINIMUM,
        "measurement": SIM_5030_MEASUREMENT,
    },
}


def test_sim_3080_amcl_tracks_ground_truth(tmp_path: Path) -> None:
    recording = tmp_path / "amcl.jsonl"

    metrics = run_scenario(
        seed=SEED,
        scenario=SCENARIO,
        time_scale=TIME_SCALE,
        timeout=TIMEOUT_SECONDS,
        pose_source="amcl",
        drive_on_estimate=False,
        launch_arguments={"recording_path": str(recording)},
    )
    # The lap is the reference stack's, unchanged by the estimator beside
    # it: the same 120.44 s SIM-3090 and SIM-3070 measure.
    assert metrics["lap_completed"], metrics
    assert metrics["collision_count"] == 0, metrics

    events = read_recording(recording)
    live = last_live_error(events)
    assert live["estimate_topic"] == ESTIMATE_TOPIC
    # A lap of 100 Hz truth at 5x: ~12 100 samples.
    assert live["sample_count"] > 10000, live
    # An estimator that stalls scores as unavailable, never as zero error
    # (EVAL-1030): availability is the first thing this test asserts on.
    availability = live["available_count"] / live["sample_count"]
    assert availability >= AVAILABILITY_MINIMUM, live
    assert live["position_rmse"] <= POSITION_RMSE_MAXIMUM, live
    assert live["position_maximum"] <= POSITION_MAXIMUM, live

    offline = score_recording(events, estimate_topic=ESTIMATE_TOPIC)
    violations = compare_localization(OFFLINE_BOUNDS, offline)
    assert violations == [], format_report(offline, violations)
