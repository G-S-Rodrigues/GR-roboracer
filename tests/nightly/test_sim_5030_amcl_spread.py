"""SIM-5030: `nav2_amcl`'s run-to-run spread at one seed.

A particle filter has no seed here: `nav2_amcl` draws its particles from
its own RNG, and two runs of the same scenario with the same simulator
seed are the same trajectory observed by two different filters. Nothing in
the stack makes that spread visible - every other localization test scores
one run - so the spread is measured here, over N identical runs, and the
tier-3 bound SIM-3080 asserts is derived from this, never from a single
run.

Tier 5 because that is what it costs: RUN_COUNT Spielberg laps at 5x is
~25 minutes, which belongs in `--nightly` and not in the gate (ADR 0005).
What it buys nightly is that a regression in AMCL's *stability* - a
configuration change that leaves the mean untouched and doubles the
spread - is visible rather than inferred from a tier-3 test that happens
to still pass.

Spielberg, never analytic_circle: a rotationally symmetric track leaves
position along the centerline unobservable, and every number here would be
measuring nothing (repo-gotchas #18).

The sample, from this test's own first full execution (9 min 43 s):

    ate_rmse      0.215 0.246 0.275 0.289 0.301
                  0.306 0.316 0.323 0.429 0.437
                  spread 0.223, mean 0.314
    ate_maximum   0.736 .. 1.834
    availability  0.99411 .. 0.99900
    lap           120.44 s, 0 collisions, on all ten

The spread is narrow - the plan's first branch - so SIM-3080 gets a real
error bound rather than a convergence-only assertion. The tails are what
the bounds here have to survive: the distribution is not symmetric, and
two of ten runs sit ~40 % above the median.
"""

import statistics
from pathlib import Path

from racing_test_keywords.scenario_runner import run_scenario

from scripts.compare_localization import (
    read_recording,
    score_recording,
)

SCENARIO = "config/scenarios/spielberg.yaml"
ESTIMATE_TOPIC = "/localization/odom"
SEED = 42
# The scale SIM-3080 and SIM-3090 run at, and the scale these numbers are
# measured at: AMCL's update rate is wall-clock bound work inside a
# simulated-time run, so a spread measured at 1x would not transfer.
TIME_SCALE = 5.0
TIMEOUT_SECONDS = 400.0
# The plan's N >= 10: enough that the maximum of the sample is a usable
# bound, few enough that the suite stays inside a nightly window.
RUN_COUNT = 10

# Measured: the sample in this module's docstring. Each bound is ~1.5x
# the sample's own worst value - more headroom than SIM-3070's 1.4x,
# because what varies here is a particle filter's unseeded RNG and ten
# draws do not bound its tail. A bound that has to be widened is a
# regression to explain, never a number to raise (repo-gotchas #14).
ATE_RMSE_MAXIMUM = 0.65  # sample maximum 0.437
ATE_RMSE_SPREAD_MAXIMUM = 0.35  # sample spread 0.223
AVAILABILITY_MINIMUM = 0.99  # sample minimum 0.99411


def test_sim_5030_amcl_run_to_run_spread(tmp_path: Path) -> None:
    scores = []
    for run in range(RUN_COUNT):
        recording = tmp_path / f"amcl-{run}.jsonl"
        metrics = run_scenario(
            seed=SEED,
            scenario=SCENARIO,
            time_scale=TIME_SCALE,
            timeout=TIMEOUT_SECONDS,
            pose_source="amcl",
            drive_on_estimate=False,
            launch_arguments={"recording_path": str(recording)},
        )
        # Open loop: the lap is the reference stack's own, identical every
        # run, and AMCL is measured beside it. A lap that differs means
        # the estimator leaked into the driven path, not that AMCL moved.
        assert metrics["lap_completed"], metrics
        assert metrics["collision_count"] == 0, metrics

        scored = score_recording(
            read_recording(recording), estimate_topic=ESTIMATE_TOPIC
        )
        print(f"SIM-5030 run {run}: {scored}")
        scores.append(scored)

    rmses = [score["ate_rmse"] for score in scores]
    availabilities = [score["availability"] for score in scores]
    spread = max(rmses) - min(rmses)
    print(
        f"SIM-5030 ate_rmse: {sorted(round(value, 3) for value in rmses)} "
        f"spread {spread:.3f} "
        f"mean {statistics.fmean(rmses):.3f} "
        f"availability min {min(availabilities):.5f}"
    )

    assert min(availabilities) >= AVAILABILITY_MINIMUM, availabilities
    assert max(rmses) <= ATE_RMSE_MAXIMUM, rmses
    assert spread <= ATE_RMSE_SPREAD_MAXIMUM, rmses
