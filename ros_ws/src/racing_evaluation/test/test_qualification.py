"""EVAL-1640/1650: the qualification score (#44 part 10)."""

import pytest
from racing_evaluation.qualification import Score, score

LAPS = [30.0 + k for k in range(12)]  # lap 1 is the 30.0 s warm-up


def test_eval_1640_ten_middle_laps_plus_ten_seconds_per_collision():
    """EVAL-1640: 12 laps, the first a warm-up and the last a cool-down;
    the ten between are summed and each collision adds 10 s."""
    got = score(LAPS, collisions=2)
    assert got == Score(
        total=sum(LAPS[1:11]) + 20.0, scored_laps=LAPS[1:11], dq=False
    )
    assert score(LAPS, collisions=0).total == sum(LAPS[1:11])
    assert score(LAPS, collisions=10).dq is False


def test_eval_1650_more_than_ten_collisions_disqualify_and_short_runs_raise():
    """EVAL-1650: 11 collisions is a DQ (no total); fewer than 12 laps is a
    ValueError, because a score of an incomplete run would be a guess."""
    out = score(LAPS, collisions=11)
    assert out.dq is True
    assert out.total is None
    with pytest.raises(ValueError, match="12"):
        score(LAPS[:11], collisions=0)
