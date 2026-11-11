"""SIM-3550: `pose_source:=slam_toolbox` on the 1080-beam lidar - slam_toolbox,
with the parameter file made for that lidar, owns `map -> odom` for a real
Spielberg lap, and the pose and the map it builds stay inside the
catastrophic bounds (ADR 0008); how well they track and match is reported
(`log/reports/SIM-3550.json`), against the baseline measured on that file.

What this claims: on `spielberg_1080`, the launch's default slam_toolbox
configuration tracks ground truth and builds a map of the gym's walls, neither
catastrophically wrong. What it does not claim: closed loop (the run
is open loop, as SIM-3070's, because the controller keeps ground truth);
other tracks; the 64-beam configuration (SIM-3070 covers it); or that
the baseline is a target - it is the worst of six measured laps times 1.25
(owner rule), read against the report, not asserted.

Spielberg, never analytic_circle: a rotationally symmetric track makes
position along the centerline unobservable to any scan matcher (gotcha #18).
"""

import dataclasses
from pathlib import Path

import yaml
from racing_evaluation.map_accuracy import load_grid, map_to_wall
from racing_test_keywords.probes import LastMapProbe
from racing_test_keywords.report import (
    catastrophic,
    catastrophic_violations,
    write_report,
)
from racing_test_keywords.scenario_runner import run_scenario

from scripts.compare_localization import (
    compare_localization,
    format_report,
    last_live_error,
    read_recording,
    score_recording,
)

REPO = Path(__file__).resolve().parents[2]
SCENARIO = "config/scenarios/spielberg_1080.yaml"
ESTIMATE_TOPIC = "/localization/odom"
SEED = 42
# Measured and run at 5x: at 1x the 64 file gave 7.54 / 7.21 m (part 8
# diagnosis A1), so its failure was never latency.
TIME_SCALE = 5.0
# Six laps took 77-87 s of wall clock from launch to metrics (x2 = 174 s,
# rounded up to the next 50 s is 200 s); never below SIM-3070's 400 s.
TIMEOUT_SECONDS = 400.0

# Part 8, 2026-10-30, 6 laps (3 each at seeds 42 and 7), time_scale 5, open
# loop, the committed slam_toolbox_online_1080.yaml, host load 4.8-8.0 at
# start and 7.9-10.5 at end (other sessions share the host):
#   seed 42: ATE rmse 1.249 / 1.168 / 1.201 m, maximum 2.108 / 2.138 / 2.239 m,
#            map median 0.522 / 0.564 / 0.541 m, p95 1.934 / 1.476 / 1.651 m
#   seed  7: ATE rmse 0.813 / 1.098 / 0.865 m, maximum 1.529 / 1.758 / 1.857 m,
#            map median 0.284 / 0.504 / 0.303 m, p95 0.916 / 1.358 / 1.039 m
#   availability >= 0.99959 on all six; every lap complete, 0 collisions.
# The 64 file on the same scenario gave 6.93-8.17 m (seeds 7 and 42); the
# old bounds, kept as the baseline a report is read against (ADR 0008) and
# read by no assert, are the worst of the six times 1.25, loose enough for
# Ceres' and Karto's unseeded multithreading, and deliberately the looser of
# the two seeds because this test runs seed 42 only. Wall times 77.2-87.1 s.
POSITION_RMSE_BASELINE = 1.57
POSITION_MAXIMUM_BASELINE = 2.80
MAP_MEDIAN_BASELINE = 0.71
MAP_P95_BASELINE = 2.42
AVAILABILITY_BASELINE = 0.99
BASELINE = {
    "position_rmse": POSITION_RMSE_BASELINE,
    "position_maximum": POSITION_MAXIMUM_BASELINE,
    "map_median": MAP_MEDIAN_BASELINE,
    "map_p95": MAP_P95_BASELINE,
    "availability": AVAILABILITY_BASELINE,
    "measurement": "6 laps at 5x, seeds 42 and 7: maximum 1.529-2.239 m, "
    "availability >= 0.99959",
}


def truth_grid():
    """The scenario's gym map image: what the sim lidar hits."""
    path = REPO / SCENARIO
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    name = document["environment"]["map"]
    base = path.parent / document["map_directory"] / name
    return load_grid(base / f"{name}.yaml")


def test_sim_3550_slam_toolbox_1080_tracks_ground_truth(
    tmp_path: Path,
) -> None:
    recording = tmp_path / "slam_toolbox_1080.jsonl"
    last_map = LastMapProbe()

    metrics = run_scenario(
        seed=SEED,
        scenario=str(REPO / SCENARIO),
        time_scale=TIME_SCALE,
        timeout=TIMEOUT_SECONDS,
        pose_source="slam_toolbox",
        drive_on_estimate=False,
        launch_arguments={"recording_path": str(recording)},
        probes=[last_map],
    )
    assert metrics["lap_completed"], metrics
    assert metrics["collision_count"] == 0, metrics

    events = read_recording(recording)
    live = last_live_error(events)
    assert live["estimate_topic"] == ESTIMATE_TOPIC
    # A lap of 100 Hz truth: ~12 000 samples.
    assert live["sample_count"] > 10000, live
    availability = live["available_count"] / live["sample_count"]

    grid = last_map.grid()
    assert grid is not None and grid.occupied.any(), "slam_toolbox built no map"
    accuracy = dataclasses.asdict(map_to_wall(grid, truth_grid()))

    offline = score_recording(events, estimate_topic=ESTIMATE_TOPIC)
    # Reported before anything quality-related is asserted, so a red run
    # keeps its record (ADR 0008).
    write_report(
        "SIM-3550",
        {
            "live": {**live, "availability": availability},
            "offline": offline,
            "map": accuracy,
        },
        seed=SEED,
        scenario=SCENARIO,
        implementation="slam_toolbox",
        baseline=BASELINE,
    )
    # An estimator that stalls scores as unavailable, never as zero error
    # (EVAL-1030): the availability bound is part of this one.
    live_violations = catastrophic_violations(
        {**live, "availability": availability},
        {"availability": "availability", "position_rmse": "ate_rmse"},
    )
    assert live_violations == [], (live_violations, live)
    map_violations = catastrophic_violations(accuracy, {"median": "map_median"})
    assert map_violations == [], (map_violations, accuracy)

    violations = compare_localization(catastrophic("availability"), offline)
    assert violations == [], format_report(offline, violations)
