"""SIM-3050: the reference stack still completes a seeded lap on simulated
time, and its metrics still match `tests/golden/baseline.json`.

The regression guard for the simulated-time base (ADR 0006): every node but
`sim_node` runs with `use_sim_time: true`, driven by `/clock`, and
`run_scenario` starts the sim held and releases it with `~/reset` once every
participant is discovered, so t=0 is deterministic. This is the same
comparison SIM-3040 makes, on that time base.
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


def test_sim_3050_simulated_time_lap_matches_golden() -> None:
    golden = json.loads(GOLDEN_PATH.read_text(encoding="utf-8"))
    expected = dict(
        golden,
        source=ROS_METRICS_SOURCE,
        track_version=ROS_TRACK_VERSION,
    )

    actual = run_scenario(seed=expected["seed"])

    differences = compare_metrics(expected, actual)

    assert differences == [], format_report(expected, actual, differences)
