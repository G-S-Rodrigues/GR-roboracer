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

# Deliberately 1x, although ADR 0006's time_scale exists to make this lap
# cheaper. Measured on this host (seed 42, the golden's own run): 1x costs
# 130.0 s / 131.3 s of wall time and reproduces lap 120.44 twice, against the
# golden's 120.41; 5x costs 53.9 s / 41.6 s / 41.6 s but laps in 120.50 /
# 120.48 / 120.49, outside lap_time's 0.05 tolerance, and moves p95 from
# ~0.030 to ~0.024 and minimum clearance from 0.89143 to 0.88928. So a run
# above 1x is not the same run, contrary to ADR 0006; the suspected cause
# (commands applied ticks stale when the wall period shrinks) is an open
# issue PR 3 must resolve before any metric from a faster run is trusted.
TIME_SCALE = 1.0
LAP_WALL_SECONDS = 131.3
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
