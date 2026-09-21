"""EVAL-1010 / EVAL-1020: the error measures against hand-computed answers."""

import math
from pathlib import Path

import pytest
import racing_common
from racing_evaluation.localization import (
    Pose,
    absolute_trajectory_error,
    lateral_longitudinal_heading_error,
    relative_pose_error,
)

SQUARE_TRACK = """\
format_version: 1
metadata:
  name: square
  closed: true
  units: meters
centerline:
  - [0.0, 0.0, 0.0, 1.0, 1.0]
  - [10.0, 0.0, 0.0, 1.0, 1.0]
  - [10.0, 10.0, 0.0, 1.0, 1.0]
  - [0.0, 10.0, 0.0, 1.0, 1.0]
"""


def test_eval_1010_absolute_trajectory_error_by_hand() -> None:
    truth = [Pose(float(t), float(t), 0.0) for t in range(4)]
    # Offsets 0, 3-4-5, 0, 1: errors 0, 5, 0, 1.
    estimates = [
        Pose(0.0, 0.0, 0.0),
        Pose(1.0, 4.0, 4.0),
        Pose(2.0, 2.0, 0.0),
        Pose(3.0, 3.0, 1.0),
    ]

    error = absolute_trajectory_error(estimates, truth)

    assert error.scored_count == 4
    assert error.availability == 1.0
    assert error.rmse == pytest.approx(math.sqrt((25.0 + 1.0) / 4.0))
    assert error.mean == pytest.approx(1.5)
    assert error.maximum == pytest.approx(5.0)


def test_eval_1010_relative_pose_error_ignores_a_constant_offset() -> None:
    """RPE scores drift, not placement: an estimate displaced by a constant
    (1, 1) has zero RPE, while one whose steps are 10% long has 0.1 m per
    1 m step - while its ATE grows without bound."""
    truth = [Pose(float(t), float(t), 0.0) for t in range(6)]
    shifted = [Pose(p.stamp, p.x + 1.0, p.y + 1.0) for p in truth]
    stretched = [Pose(p.stamp, 1.1 * p.x, p.y) for p in truth]

    constant = relative_pose_error(shifted, truth, delta=1.0)
    drifting = relative_pose_error(stretched, truth, delta=1.0)

    assert constant.scored_count == 5
    assert constant.rmse == pytest.approx(0.0, abs=1e-12)
    assert drifting.rmse == pytest.approx(0.1)
    assert drifting.maximum == pytest.approx(0.1)
    assert absolute_trajectory_error(stretched, truth).maximum == (
        pytest.approx(0.5)
    )


def test_eval_1010_relative_pose_error_is_in_the_vehicle_frame() -> None:
    """A heading error rotates every later step: truth drives 1 m along +x
    per step facing +x, the estimate faces +y but reports the same x, y. In
    the vehicle frame the estimate stepped 1 m sideways, not forwards."""
    truth = [Pose(float(t), float(t), 0.0, 0.0) for t in range(3)]
    estimates = [Pose(p.stamp, p.x, p.y, math.pi / 2) for p in truth]

    error = relative_pose_error(estimates, truth, delta=1.0)

    assert error.rmse == pytest.approx(math.sqrt(2.0))


def test_eval_1020_lateral_longitudinal_heading_by_hand(
    tmp_path: Path,
) -> None:
    track_file = tmp_path / "square.yaml"
    track_file.write_text(SQUARE_TRACK, encoding="utf-8")
    track = racing_common.Track.from_yaml(track_file)

    # First edge runs along +x, so s = x and d = y (d positive left).
    component = lateral_longitudinal_heading_error(
        Pose(0.0, 4.0, 0.2, 0.1), Pose(0.0, 3.0, 0.5, 0.0), track
    )

    assert component.lateral == pytest.approx(-0.3)
    assert component.longitudinal == pytest.approx(1.0)
    assert component.heading == pytest.approx(0.1)


def test_eval_1020_longitudinal_error_wraps_across_the_start_line(
    tmp_path: Path,
) -> None:
    """Truth 1 m before the line (s = 39 of 40), estimate 1 m after it
    (s = 1): 2 m ahead, not 38 m behind."""
    track_file = tmp_path / "square.yaml"
    track_file.write_text(SQUARE_TRACK, encoding="utf-8")
    track = racing_common.Track.from_yaml(track_file)

    component = lateral_longitudinal_heading_error(
        Pose(0.0, 1.0, 0.0, 0.0),
        Pose(0.0, 0.0, 1.0, -math.pi / 2),
        track,
    )

    assert component.longitudinal == pytest.approx(2.0)
    assert component.lateral == pytest.approx(0.0, abs=1e-9)
    assert component.heading == pytest.approx(math.pi / 2)
