"""EVAL-1030: a stalled estimator scores as unavailable, never as zero error.

An estimator that stops publishing produces no error samples, and a naive
error over zero samples is zero - a dead estimator would look perfect. So
availability is a first-class output, and the error summary itself carries
how much of the ground truth it actually scored.
"""

import math

from racing_evaluation.localization import (
    Pose,
    absolute_trajectory_error,
    availability,
)

PERIOD = 0.01


def _stream(start: float, end: float) -> list[Pose]:
    count = round((end - start) / PERIOD)
    return [
        Pose(stamp=start + index * PERIOD, x=start + index * PERIOD, y=0.0)
        for index in range(count + 1)
    ]


def test_eval_1030_a_two_second_gap_scores_as_unavailable() -> None:
    truth = _stream(0.0, 10.0)
    # The estimate is exact while it publishes, then stalls for 2 s.
    estimates = [pose for pose in truth if not 4.0 < pose.stamp < 6.0]

    summary = availability(estimates, window=0.5, start=0.0, end=10.0)

    assert math.isclose(summary.maximum_gap, 2.0, abs_tol=PERIOD)
    assert summary.stale_gap_count == 1
    # Covered: everything but the 1.5 s of the gap beyond the window.
    assert math.isclose(summary.availability, 0.85, abs_tol=0.002)

    error = absolute_trajectory_error(estimates, truth, window=0.5)

    # The gap's truth samples are not scored at all - not scored as zero.
    assert error.sample_count == len(truth)
    assert error.scored_count < error.sample_count
    assert math.isclose(error.availability, 0.8, abs_tol=0.002)
    assert error.rmse == 0.0


def test_eval_1030_a_dead_estimator_has_no_error_rather_than_zero() -> None:
    truth = _stream(0.0, 10.0)

    error = absolute_trajectory_error([], truth, window=0.5)
    summary = availability([], window=0.5, start=0.0, end=10.0)

    assert error.scored_count == 0
    assert error.availability == 0.0
    assert math.isnan(error.rmse)
    assert math.isnan(error.maximum)
    assert summary.availability == 0.0
    assert summary.maximum_gap == 10.0
