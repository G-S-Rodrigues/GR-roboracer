"""SIM-3070: `pose_source:=slam_toolbox` — slam_toolbox owns `map -> odom`
for a real Spielberg lap, and the pose it puts under the stack stays
available and bounded, scored live by racing_evaluation and again offline
by `scripts/compare_localization.py`.

What this does *not* claim is that the estimate is good. Karto's scan
matcher builds a same-sign yaw drift on this track's straights even from
exact odometry and an exact scan (plan D7 has the investigation), and the
car cannot complete a lap driving on it: closing the loop puts it off
track and the supervisor - which reads ground truth - latches TRACK_LIMIT,
which is the honest failure that asymmetry exists to produce. So the run
is open loop (`drive_on_estimate=False`): the controller keeps ground
truth, the lap is the reference stack's own, and slam_toolbox is measured
beside it. The bounds below are that measurement, not a target.

Spielberg, never analytic_circle: a rotationally symmetric track makes
position along the centerline unobservable to any scan matcher, and this
test would pass on it while measuring nothing (repo-gotchas #18).
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
# scale every bound here was measured at: slam_toolbox's latency is not
# gated by simulated time, so a bound measured at 1x would not hold at 5x.
TIME_SCALE = 5.0
# 120.44 simulated seconds of lap at 5x, plus startup, JIT warm-up and
# slam_toolbox's own Ceres threads.
TIMEOUT_SECONDS = 400.0

# Measured: three runs at seed 42, time_scale 5.0, on this host.
# position_rmse 2.76 / 2.88 / 2.43 m, position_maximum 4.42 / 4.99 / 3.67 m,
# availability 12236/12241, 12197/12203, 12113/12117 (>= 0.9995).
# The bounds sit ~40 % above the worst of the three, which is what a
# multithreaded Ceres-backed matcher's run-to-run spread needs (it has no
# seed; ADR-0005-style tolerances carry their measurement, and this one is
# a divergence guard, not a quality claim).
POSITION_RMSE_MAXIMUM = 4.0
POSITION_MAXIMUM = 7.0
AVAILABILITY_MINIMUM = 0.99
OFFLINE_BOUNDS = {
    "ate_maximum": {
        "maximum": POSITION_MAXIMUM,
        "measurement": "3 runs at 5x, seed 42: live maximum 3.67-4.99 m",
    },
    "availability": {
        "minimum": AVAILABILITY_MINIMUM,
        "measurement": "3 runs at 5x, seed 42: >= 0.9995 of truth's span",
    },
}


def test_sim_3070_slam_toolbox_tracks_ground_truth(tmp_path: Path) -> None:
    recording = tmp_path / "slam_toolbox.jsonl"

    metrics = run_scenario(
        seed=SEED,
        scenario=SCENARIO,
        time_scale=TIME_SCALE,
        timeout=TIMEOUT_SECONDS,
        pose_source="slam_toolbox",
        drive_on_estimate=False,
        launch_arguments={"recording_path": str(recording)},
    )
    # The lap is the reference stack's, unchanged by the estimator beside
    # it: the same 120.44 s SIM-3090 measures.
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
