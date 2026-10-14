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
What it buys nightly is that a shift in AMCL's typical error (the median
of the runs) or a single-run blow-up (the maximum) is visible rather than
inferred from a tier-3 test that happens to still pass. It does not see
a wider spread or more frequent tail runs around an unchanged median;
the bound comment below says what that costs.

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
two of ten runs sit ~40 % above the median. The bound is therefore on the
median and on each run's maximum, never on max-min of ten (see the
constants below, which carry every sample).
"""

import json
import shutil
import statistics
import time
from pathlib import Path

from racing_test_keywords.scenario_runner import (
    REPOSITORY_ROOT,
    host_load_delta,
    read_host_load,
    run_scenario,
)

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

# Measured, in three steps. (1) The docstring sample above: maximum 0.437,
# median 0.3035. Each bound was first set ~1.5x the sample's worst, with a
# max-min spread bound of 0.35 (sample 0.223). (2) That spread bound failed
# the nightly 38381183034 on a healthy filter (rmse 0.172 .. 0.570, spread
# 0.398, median 0.2345, availability 1.0), and the N1 investigation drew 70
# more runs on CI: the spread of ten is set by ONE run in the heavy upper
# tail (single runs of 0.476, 0.497, 0.541, 0.570 in ~80 CI runs, about 5 %),
# so it passes only ~85-90 % of the time and is not a stability measure.
# (3) The statistic was changed, not the number. Per-sample medians over
# the 10 runs, every sample we hold:
#
#     docstring (dev host)     median 0.3035  IQR 0.043  max 0.437
#     red nightly 38381183034  median 0.2345  IQR 0.117  max 0.570
#     CI 38616809490 leg 1/2/3 median 0.234 / 0.285 / 0.253
#                              IQR    0.164 / 0.113 / 0.076
#                              max    0.410 / 0.407 / 0.369
#     CI 38618967101 leg 1-4   median 0.251 / 0.275 / 0.217 / 0.220
#                              IQR    0.046 / 0.066 / 0.109 / 0.078
#                              max    0.541 / 0.497 / 0.362 / 0.289
#     (local dev run of this branch: median in the 0.23-0.28 band, max
#     0.364; per-run values not kept. One CI leg aborted at run 5 on a
#     startup failure, not a spread: not a sample.)
#
# The median spans 0.217 .. 0.3035 over those nine samples (0.09 wide);
# the IQR spans 0.043 .. 0.164 (nearly 4x wide), because one or two tail
# runs landing inside the quartiles move it. The median is the statistic
# the data separates: it ignores a one-off tail run (changing one of ten
# runs cannot move it past its neighbours) and moves when the typical
# error does. The bound is 0.40: 1.3x the highest observed median
# (0.3035) and ~1.6x the typical 0.25, which a shift of the typical
# error to 0.425 crosses (tested below on a constructed sample) while its
# maximum stays under 0.65. A maximum bound alone would miss that shift;
# the median bound is what makes the test see a worse filter. The
# per-run 0.65 stays: the observed single-run tail tops out at 0.570.
# What the median gives up: a wider spread or more frequent tail runs
# around an unchanged median. Bootstrapped over the 80 pooled CI runs
# (10-run draws), old spread bound vs this one: false red 21 % vs 0.01 %;
# spread doubled 82 % vs 32 %; tail rate 5 -> 20 % 71 % vs 3 %; whole
# distribution x1.5 44 % vs 45 %; x2 91 % vs 99 %. Fewer false reds and a
# better view of a worse typical error, bought with near-blindness to a
# fatter tail.
# A bound that has to be widened is a regression to explain, never a
# number to raise (repo-gotchas #14).
ATE_RMSE_MAXIMUM = 0.65  # sample maximum 0.570 (red nightly), 0.541 (CI)
ATE_RMSE_MEDIAN_MAXIMUM = 0.40  # highest sample median 0.3035
AVAILABILITY_MINIMUM = 0.99  # sample minimum 0.99411


def amcl_violations(
    rmses: list[float], availabilities: list[float]
) -> list[str]:
    """Names of the bounds this sample of N runs breaks, in order."""
    violated = []
    if min(availabilities) < AVAILABILITY_MINIMUM:
        violated.append("availability")
    if max(rmses) > ATE_RMSE_MAXIMUM:
        violated.append("maximum")
    if statistics.median(rmses) > ATE_RMSE_MEDIAN_MAXIMUM:
        violated.append("median")
    return violated


def test_sim_5030_amcl_run_to_run_spread(tmp_path: Path) -> None:
    scores = []
    load_before = read_host_load()
    started = time.monotonic()
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
    median = statistics.median(rmses)
    print(
        f"SIM-5030 ate_rmse: {sorted(round(value, 3) for value in rmses)} "
        f"spread {spread:.3f} "
        f"median {median:.3f} "
        f"mean {statistics.fmean(rmses):.3f} "
        f"availability min {min(availabilities):.5f}"
    )

    # Written before the asserts, so a red run keeps its sample (and the
    # worst run's recording): a passing run prints nothing, and until this
    # existed no distribution of the spread had ever been kept.
    log_dir = REPOSITORY_ROOT / "log"
    log_dir.mkdir(exist_ok=True)
    (log_dir / "sim-5030.json").write_text(
        json.dumps(
            {
                "scores": scores,
                "ate_rmse_sorted": sorted(rmses),
                "spread": spread,
                "median": median,
                "mean": statistics.fmean(rmses),
                "availability_min": min(availabilities),
                "wall_seconds": time.monotonic() - started,
                "host_load": host_load_delta(load_before, read_host_load()),
            },
            indent=2,
        )
    )
    violated = amcl_violations(rmses, availabilities)
    if violated:
        worst = rmses.index(max(rmses))
        shutil.copyfile(
            tmp_path / f"amcl-{worst}.jsonl", log_dir / "sim-5030-max.jsonl"
        )

    assert not violated, (violated, sorted(rmses), availabilities)


# The red nightly 38381183034's own ten (sorted), and the ten of the
# original docstring sample: the bound must pass both and reject a shift.
RED_NIGHTLY_SAMPLE = [
    0.172,
    0.178,
    0.191,
    0.232,
    0.234,
    0.235,
    0.268,
    0.335,
    0.385,
    0.570,
]
DOCSTRING_SAMPLE = [
    0.215,
    0.246,
    0.275,
    0.289,
    0.301,
    0.306,
    0.316,
    0.323,
    0.429,
    0.437,
]
AVAILABLE = [1.0] * RUN_COUNT


def test_sim_5030_bound_passes_the_red_nightly_sample() -> None:
    assert amcl_violations(RED_NIGHTLY_SAMPLE, AVAILABLE) == []
    assert amcl_violations(DOCSTRING_SAMPLE, AVAILABLE) == []


def test_sim_5030_bound_rejects_a_shifted_median() -> None:
    # Typical error ~1.7x the red nightly's (median 0.425 against 0.2345)
    # with no run above 0.65: the old max-min spread (0.25) passes it too.
    shifted = [0.35, 0.38, 0.40, 0.41, 0.42, 0.43, 0.45, 0.48, 0.52, 0.60]
    assert max(shifted) <= ATE_RMSE_MAXIMUM
    assert amcl_violations(shifted, AVAILABLE) == ["median"]


def test_sim_5030_bound_rejects_a_single_run_past_the_maximum() -> None:
    assert amcl_violations([*RED_NIGHTLY_SAMPLE[:-1], 0.66], AVAILABLE) == [
        "maximum"
    ]
