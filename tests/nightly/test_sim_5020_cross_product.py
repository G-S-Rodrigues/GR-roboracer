"""SIM-5020: every pose source, on every seed, scored the same way.

The cross-product ADR 0005 keeps out of the gate. Tiers 3 and 4 verify each
implementation once, at one seed, inside the reference stack current when
it landed; this is the test that notices the combination - an estimator
that only converges from the start pose seed 42 draws, or a lap that only
completes for the seed its bound was measured at.

The track dimension is Spielberg alone, and deliberately: it is the only
track with a committed occupancy grid for nav2_amcl, and the only one where
a localization number means anything at all, since analytic_circle leaves
position along the centerline unobservable (repo-gotchas #18). A second
league track adds a row here the day it is surveyed.

Open loop throughout: the lap is then the reference stack's own for every
row, so a row that fails is the estimator and not the driving.

`ground_truth` is not a row. It is not an estimator: with it selected the
support node broadcasts `map -> odom` itself and publishes no composed
`/localization/odom` at all (BRINGUP-1040), so scoring it here measured
availability 0.0 over 12 050 truth samples on both seeds - a row that
fails for a reason that has nothing to do with the cross-product. The
default path's correctness is SIM-3090's and the goldens', not this
test's.

Measured, this test's own first full run (open loop, 5x, 9 min 23 s for
six rows, of which these four are kept):

    amcl          seed   42  ate_rmse 0.403  ate_maximum 1.677
                  seed 1030  ate_rmse 0.357  ate_maximum 1.744
                  availability 0.99478 / 0.99651
    slam_toolbox  seed   42  ate_rmse 2.942  ate_maximum 4.528
                  seed 1030  ate_rmse 1.883  ate_maximum 3.554
                  availability 0.99967 / 0.99975

Every row completed its lap with no collision, so seed 1030 on Spielberg
- which nothing in the suite had run before - laps on both estimators.
"""

from pathlib import Path

import pytest
from racing_test_keywords.scenario_runner import run_scenario

from scripts.compare_localization import read_recording, score_recording

SCENARIO = "config/scenarios/spielberg.yaml"
ESTIMATE_TOPIC = "/localization/odom"
TIME_SCALE = 5.0
TIMEOUT_SECONDS = 400.0
# 42 is the seed every committed tier-3 bound is measured at; 1030 is the
# other seed the suite already pins, and a second start pose is the whole
# point of this row - f110_env.reset draws the start arc length from it.
SEEDS = (42, 1030)
# (ate_rmse below, availability above). Each is the bound its own tier-3
# case already carries - SIM-3080's 0.65 from SIM-5030's ten runs,
# SIM-3070's 4.0 from three - which is the point: this test asks whether
# the bound measured at seed 42 survives a second start pose, so it must
# be the same number and never a looser one.
BOUNDS = {
    "slam_toolbox": (4.0, 0.99),
    "amcl": (0.65, 0.99),
}


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("pose_source", sorted(BOUNDS))
def test_sim_5020_pose_source_across_seeds(
    pose_source: str, seed: int, tmp_path: Path
) -> None:
    recording = tmp_path / f"{pose_source}-{seed}.jsonl"

    metrics = run_scenario(
        seed=seed,
        scenario=SCENARIO,
        time_scale=TIME_SCALE,
        timeout=TIMEOUT_SECONDS,
        pose_source=pose_source,
        drive_on_estimate=False,
        launch_arguments={"recording_path": str(recording)},
    )
    assert metrics["lap_completed"], metrics
    assert metrics["collision_count"] == 0, metrics

    scored = score_recording(
        read_recording(recording), estimate_topic=ESTIMATE_TOPIC
    )
    print(f"SIM-5020 {pose_source} seed {seed}: {scored}")

    rmse_maximum, availability_minimum = BOUNDS[pose_source]
    # A stalled estimator scores as unavailable, never as zero error
    # (EVAL-1030), so availability comes first.
    assert scored["availability"] >= availability_minimum, scored
    assert scored["ate_rmse"] <= rmse_maximum, scored
