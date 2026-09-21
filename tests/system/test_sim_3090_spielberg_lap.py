"""SIM-3090: one seeded Spielberg lap on ground-truth pose completes, through
the real launch, and matches `tests/golden/spielberg_baseline.json`.

The league track's regression guard - the one Spielberg lap `--full` carries
(ADR 0005). The golden is generated headless by `sim/rollout.py` on the GPU,
in ground-truth pose mode, exactly as `tests/golden/baseline.json` is, and
never through a run using a pose estimator. See the golden's own
regeneration command in `docs/running.md`.

`source` and `track_version` differ structurally between the two pipelines,
exactly as SIM-3040 documents; everything else is held to
`scripts/compare_metrics.py`'s ordinary tolerances.
"""

import json
from pathlib import Path

from racing_test_keywords.scenario_runner import run_scenario

from scripts.compare_metrics import compare_metrics, format_report

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
GOLDEN_PATH = REPOSITORY_ROOT / "tests" / "golden" / "spielberg_baseline.json"
SCENARIO = str(REPOSITORY_ROOT / "config" / "scenarios" / "spielberg.yaml")

# racing_metrics/src/node.cpp: metrics_source
ROS_METRICS_SOURCE = "racing_metrics"
# config/scenarios/spielberg.yaml: metrics.track_version
ROS_TRACK_VERSION = "spielberg-v1"

# 5x, ADR 0006's time_scale paying for the league lap. A run above 1x is the
# same run because the sim waits, above 1x, for the command answering the
# tick it last published (SIMJAX-1070). Before that it did not: at 5x 1998
# of 2000 steps applied a command one tick staler than at 1x, and this lap
# measured 120.47-120.50 against the golden's 120.41, p95 ~0.024 against
# 1x's ~0.030. Measured with it (seed 42): 5x 43.2 s / 42.2 s of wall time,
# lap 120.44, p95 0.029779, clearance 0.8914279 - the 1x run's metrics
# (130.3 s of wall), to the last digit but p95's 5th.
TIME_SCALE = 5.0
LAP_WALL_SECONDS = 43.2
# 3x the measured wall time: absorbs host load (it runs last in `--full`)
# without letting a hung graph cost much more than the lap itself.
TIMEOUT_SECONDS = 3.0 * LAP_WALL_SECONDS


def test_sim_3090_spielberg_lap_matches_golden() -> None:
    golden = json.loads(GOLDEN_PATH.read_text(encoding="utf-8"))
    expected = dict(
        golden,
        source=ROS_METRICS_SOURCE,
        track_version=ROS_TRACK_VERSION,
    )

    actual = run_scenario(
        seed=expected["seed"],
        scenario=SCENARIO,
        timeout=TIMEOUT_SECONDS,
        time_scale=TIME_SCALE,
    )

    differences = compare_metrics(expected, actual)

    assert differences == [], format_report(expected, actual, differences)
