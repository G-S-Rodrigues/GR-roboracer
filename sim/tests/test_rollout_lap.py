"""The golden generator's lap rule, against a non-circular track."""

import math
from pathlib import Path

import racing_common

from sim.rollout import CenterlineLap

SPIELBERG = Path(__file__).parents[2] / "config" / "tracks" / "spielberg.yaml"
# Where seed 42 starts on Spielberg (the golden's own start).
START_S = 182.05
# About one 10 ms step at 3 m/s.
STEP_M = 0.03


def _winding_lap_distance(track: racing_common.Track) -> float:
    """Centerline distance after which f1tenth_gym_jax counts a lap.

    Its rule (`envs/f110_env.py` `check_done`): accumulate the angle swept
    around a winding point 1.5 m to the side of s=0, and call it a lap once
    that angle reaches 2 pi. On a track that is not star-shaped around that
    point the swept angle overshoots before the car is back.
    """
    length = track.length()
    point = track.to_cartesian(racing_common.FrenetPoint(0.0, -1.5, 0.0))
    previous = None
    swept = 0.0
    travelled = 0.0
    while travelled < 1.2 * length:
        s = (START_S + travelled) % length
        pose = track.to_cartesian(racing_common.FrenetPoint(s, 0.0, 0.0))
        vector = (pose.x - point.x, pose.y - point.y)
        if previous is not None:
            swept += math.atan2(
                previous[0] * vector[1] - previous[1] * vector[0],
                previous[0] * vector[0] + previous[1] * vector[1],
            )
        previous = vector
        if int(abs(swept) / (2.0 * math.pi)) >= 1:
            return travelled
        travelled += STEP_M
    raise AssertionError("the winding rule never counted a lap")


def test_sim_1010_lap_ends_on_centerline_distance_not_winding() -> None:
    """SIM-1010: `sim/rollout.py` ends a lap when the centerline distance
    travelled from the first sample reaches `track.length()` -
    `MetricsAccumulator`'s rule, which the live graph uses.

    The gym's winding-number lap agreed with it on analytic_circle and ends
    11.7 m (4.7 s) early on Spielberg, so a golden that used it scored a
    shorter lap than the live stack drives (repo-gotchas #15).
    """
    track = racing_common.Track.from_yaml(SPIELBERG)
    length = track.length()
    assert _winding_lap_distance(track) < length - 10.0

    lap = CenterlineLap(length)
    travelled = 0.0
    step = 0
    while not lap.update((START_S + travelled) % length, 0.01 * (step + 1)):
        assert travelled < length + STEP_M, "lap never completed"
        travelled += STEP_M
        step += 1

    assert length <= travelled < length + STEP_M
    assert lap.completed
    assert math.isclose(lap.lap_time, 0.01 * step, abs_tol=1e-9)


def test_sim_1010_lap_ignores_a_backwards_sample_across_the_seam() -> None:
    """SIM-1010: progress wraps at s=0 and reversing subtracts distance."""
    lap = CenterlineLap(10.0)

    assert not lap.update(9.0, 0.0)
    assert not lap.update(1.0, 1.0)  # +2 across the seam
    assert not lap.update(0.5, 2.0)  # -0.5
    for time, s in enumerate((3.0, 6.0, 8.5), start=3):
        assert not lap.update(s, float(time))
    assert lap.update(9.0, 6.0)  # 2 - 0.5 + 8.5 = 10


def test_sim_1010_lap_starts_at_the_first_sample_it_is_given() -> None:
    """SIM-1010: like `MetricsAccumulator`, distance and the lap clock start
    at the first sample, and the lap time is measured from that sample's
    stamp - so the rollout feeds it the first post-step state, as the live
    graph's first /ground_truth sample, never the reset pose.
    """
    lap = CenterlineLap(10.0)

    assert not lap.update(4.0, 0.01)
    assert not lap.update(7.0, 0.34)  # +3
    assert not lap.update(10.0, 0.67)  # +3, at the seam
    assert lap.update(14.0, 1.01)  # +4: 10 from the first sample
    assert math.isclose(lap.lap_time, 1.0, abs_tol=1e-12)
    # Once complete, later samples change nothing.
    assert lap.update(15.0, 2.0)
    assert math.isclose(lap.lap_time, 1.0, abs_tol=1e-12)
