"""EVAL-1040: an estimate and ground truth arriving at different rates.

Truth runs at the simulator's 100 Hz; an estimator runs at its own rate and
its stamps fall between truth's. Each truth sample is scored against the
estimate interpolated to its own stamp - never against whichever estimate
happens to be latest, which would score an exact estimator's own latency
as position error.
"""

import math

import pytest
from racing_evaluation.localization import (
    Aligner,
    Pose,
    absolute_trajectory_error,
    align,
)

SPEED = 3.0


def _moving(stamps: list[float]) -> list[Pose]:
    """A vehicle driving along +x at SPEED, turning at 0.5 rad/s."""
    return [Pose(t, SPEED * t, 0.0, 0.5 * t) for t in stamps]


def test_eval_1040_a_slower_exact_estimate_interpolates_to_zero_error() -> None:
    truth = _moving([index * 0.01 for index in range(301)])
    # 15 Hz, phase-shifted so no stamp coincides with a truth stamp.
    estimates = _moving([0.003 + index / 15.0 for index in range(45)])

    pairs = align(estimates, truth, window=0.5)
    scored = [pair for pair in pairs if pair.estimate is not None]

    # Only truth samples outside the estimate stream's span are unscored.
    assert len(pairs) == len(truth)
    assert all(0.003 <= p.truth.stamp <= estimates[-1].stamp for p in scored)
    assert len(scored) == sum(
        0.003 <= p.stamp <= estimates[-1].stamp for p in truth
    )
    for pair in scored:
        assert pair.estimate.stamp == pair.truth.stamp
        assert pair.estimate.x == pytest.approx(pair.truth.x)
        assert pair.estimate.yaw == pytest.approx(pair.truth.yaw)
    error = absolute_trajectory_error(estimates, truth, window=0.5)
    assert error.maximum == pytest.approx(0.0, abs=1e-12)


def test_eval_1040_heading_interpolates_the_short_way_round() -> None:
    estimates = [
        Pose(0.0, 0.0, 0.0, math.pi - 0.1),
        Pose(1.0, 0.0, 0.0, -math.pi + 0.1),
    ]
    truth = [Pose(0.5, 0.0, 0.0, math.pi)]

    (pair,) = align(estimates, truth, window=2.0)

    assert abs(math.remainder(pair.estimate.yaw - math.pi, math.tau)) < 1e-12


def test_eval_1040_live_alignment_waits_for_a_late_estimate() -> None:
    """Live, truth arrives first: a sample waits for the estimate after it,
    and is declared unavailable only once truth has run a window past it."""
    aligner = Aligner(window=0.1)

    assert aligner.add_truth(Pose(0.00, 0.0, 0.0)) == []
    assert aligner.add_truth(Pose(0.05, 0.5, 0.0)) == []
    (first,) = aligner.add_estimate(Pose(0.00, 0.0, 0.0))
    assert first.truth.stamp == 0.00
    assert first.estimate is not None
    (second,) = aligner.add_estimate(Pose(0.08, 0.8, 0.0))
    assert second.estimate.x == pytest.approx(0.5)

    # The estimator stalls: truth at 0.10 waits until truth passes 0.20.
    assert aligner.add_truth(Pose(0.10, 1.0, 0.0)) == []
    assert aligner.add_truth(Pose(0.20, 2.0, 0.0)) == []
    stale = aligner.add_truth(Pose(0.21, 2.1, 0.0))
    assert [pair.truth.stamp for pair in stale] == [0.10]
    assert stale[0].estimate is None
    assert [pair.estimate for pair in aligner.flush()] == [None, None]
