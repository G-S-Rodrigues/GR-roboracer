"""SIM-3050: at time_scale=2.0, the seeded lap's metrics still match
`tests/golden/baseline.json`.

SIM-3040 makes this comparison at the default time_scale=1.0. This is the
same comparison, run with `run_scenario`'s `time_scale` at 2.0 instead, to
prove `time_scale` changes only the run's wall-clock rate: it must never
change the physics `run_scenario` integrates or the simulated `lap_time`
`racing_metrics` reports, since both come from the same `/clock`-driven
simulated-time counter regardless of how fast the wall timer that advances
it fires (ADR 0006).
"""

import json
from pathlib import Path

from racing_test_keywords.scenario_runner import run_scenario

from scripts.compare_metrics import compare_metrics, format_report

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
GOLDEN_PATH = REPOSITORY_ROOT / "tests" / "golden" / "baseline.json"

# racing_metrics/src/node.cpp: metrics_source
ROS_METRICS_SOURCE = "racing_metrics"
# config/vehicles/f1tenth_default.yaml: racing_metrics.track_version
ROS_TRACK_VERSION = "analytic_circle-v1"
TIME_SCALE = 2.0


def test_sim_3050_simulated_time_lap_matches_golden() -> None:
    golden = json.loads(GOLDEN_PATH.read_text(encoding="utf-8"))
    expected = dict(
        golden,
        source=ROS_METRICS_SOURCE,
        track_version=ROS_TRACK_VERSION,
    )

    actual = run_scenario(seed=expected["seed"], time_scale=TIME_SCALE)

    differences = compare_metrics(expected, actual)

    assert differences == [], format_report(expected, actual, differences)
