import math

import pytest
from racing_bringup.support_node import _planar_correction


def _compose(first, second):
    x, y, yaw = first
    return (
        x + math.cos(yaw) * second[0] - math.sin(yaw) * second[1],
        y + math.sin(yaw) * second[0] + math.cos(yaw) * second[1],
        yaw + second[2],
    )


@pytest.mark.parametrize(
    ("truth", "dead_reckoning"),
    [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((10.0, -2.0, 0.3), (10.0, -2.0, 0.3)),
        ((10.0, -2.0, 0.3), (10.4, -1.7, 0.25)),
        ((-3.0, 5.0, -2.9), (-2.1, 4.2, 3.0)),
    ],
)
def test_bringup_1020_map_odom_composes_dead_reckoning_to_truth(
    truth, dead_reckoning
) -> None:
    """BRINGUP-1020: the support node's map -> odom, composed with the sim's
    odom -> base_link dead reckoning, is exactly the ground-truth pose, and
    identity when the two agree.

    The numeric half of repo-gotchas #16: identity here would leave the
    vehicle on its drifting estimate, and the ground-truth pose here would
    double-count it; both look like a working TF tree.
    """
    correction = _planar_correction(truth, dead_reckoning)
    composed = _compose(correction, dead_reckoning)

    assert composed[0] == pytest.approx(truth[0], abs=1e-12)
    assert composed[1] == pytest.approx(truth[1], abs=1e-12)
    assert math.remainder(composed[2] - truth[2], 2 * math.pi) == (
        pytest.approx(0.0, abs=1e-12)
    )
    if truth == dead_reckoning:
        assert correction == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
