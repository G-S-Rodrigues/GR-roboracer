"""The golden generator's Pure Pursuit reference point follows the selector."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import racing_common
import racing_controller_baseline

from sim.rollout import pure_pursuit_action_provider, reference_pose

CIRCLE = (
    Path(__file__).parents[2] / "config" / "tracks" / "analytic_circle.yaml"
)
LR = 0.17145


def _cartesian() -> np.ndarray:
    return np.array([2.0, -1.0, 0.1, 3.0, math.pi / 2, 0.2, 0.05], np.float32)


def test_sim_1020_rollout_reference_pose_follows_the_selector():
    """SIM-1020: `cog_link` is the gym's floats untouched, `base_link` the
    rear axle, anything else an error.

    The default must be bit-identical to what `provide` read before part
    3.1 (`cartesian[0], [1], [4]`), or every golden moves; `==`, not
    approx. `base_link` is racing_common's translate, the same arithmetic
    the sim backend and the Pure Pursuit node use.
    """
    cartesian = _cartesian()
    pose = reference_pose(cartesian, "cog_link", LR)
    assert (pose.x, pose.y, pose.psi) == (
        float(cartesian[0]),
        float(cartesian[1]),
        float(cartesian[4]),
    )

    rear = reference_pose(cartesian, "base_link", LR)
    expected = racing_common.translate_along_heading(
        racing_common.CartesianPose(
            float(cartesian[0]), float(cartesian[1]), float(cartesian[4])
        ),
        -LR,
    )
    assert rear.x == pytest.approx(expected.x, abs=1e-12)
    assert rear.y == pytest.approx(expected.y, abs=1e-12)
    assert rear.psi == expected.psi

    with pytest.raises(ValueError, match="front_axle"):
        reference_pose(cartesian, "front_axle", LR)


def test_sim_1020_provider_steers_the_selected_point():
    """The provider with `base_link` equals the law on the rear-axle state."""
    kwargs = dict(
        lookahead_distance=1.5,
        wheelbase=0.33,
        minimum_speed=1.0,
        maximum_speed=3.0,
        curvature_speed_gain=5.0,
    )
    provider = pure_pursuit_action_provider(
        CIRCLE, reference="base_link", cog_to_rear_axle=LR, **kwargs
    )
    state = SimpleNamespace(cartesian_states=_cartesian()[None, :])
    steering, speed = provider({}, state)

    import yaml

    document = yaml.safe_load(CIRCLE.read_text(encoding="utf-8"))
    trajectory = [
        racing_controller_baseline.TrajectoryPoint(float(r[0]), float(r[1]))
        for r in document["centerline"]
    ]
    params = racing_controller_baseline.ControllerParams()
    for name, value in kwargs.items():
        setattr(params, name, value)
    params.closed_trajectory = bool(document["metadata"]["closed"])
    rear = reference_pose(_cartesian(), "base_link", LR)
    command = racing_controller_baseline.PurePursuit.compute(
        racing_controller_baseline.VehicleState(rear.x, rear.y, rear.psi),
        trajectory,
        params,
    )
    assert (steering, speed) == (
        float(command.steering_angle),
        float(command.speed),
    )


@pytest.mark.parametrize("missing", [None, 0.0])
def test_sim_1020_base_link_without_an_offset_is_refused(missing):
    """SIM-1020: `base_link` with no `cog_to_rear_axle` raises.

    Without the offset the rear axle is the CoG, so a caller who forgot it
    would get a plausible lap steered from the wrong point.
    """
    kwargs = dict(
        lookahead_distance=1.5,
        wheelbase=0.33,
        minimum_speed=1.0,
        maximum_speed=3.0,
        curvature_speed_gain=5.0,
    )
    with pytest.raises(ValueError, match="cog_to_rear_axle"):
        pure_pursuit_action_provider(
            CIRCLE,
            reference="base_link",
            cog_to_rear_axle=missing,
            **kwargs,
        )
    with pytest.raises(ValueError, match="cog_to_rear_axle"):
        pure_pursuit_action_provider(CIRCLE, reference="base_link", **kwargs)
