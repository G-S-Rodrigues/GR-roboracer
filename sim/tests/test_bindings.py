"""Parity tests for the Python view of the shared racing_common core."""

import math
from pathlib import Path

import racing_common

TRACK = Path(__file__).parents[2] / "config" / "tracks" / "analytic_circle.yaml"


def test_common_1050_python_uses_the_cpp_frenet_implementation() -> None:
    """COMMON-1050: Python exposes the exact C++ Track implementation."""
    track = racing_common.Track.from_yaml(TRACK)
    original = racing_common.FrenetPoint(5.0, 0.75, -0.2)

    pose = track.to_cartesian(original)
    actual = track.to_frenet(pose)

    assert math.isclose(actual.s, original.s, abs_tol=1e-9)
    assert math.isclose(actual.d, original.d, abs_tol=1e-9)
    assert math.isclose(
        actual.heading_error, original.heading_error, abs_tol=1e-9
    )
    assert math.isclose(track.curvature_at(17.0), 0.1, abs_tol=1e-12)


def test_sim_1080_vehicle_state_carries_the_measured_speed() -> None:
    """SIM-1080: the controller's Python `VehicleState` takes `speed` (the
    measured twist the lookahead reads), defaulting to 0.0, and keeps it."""
    import racing_controller_baseline

    assert racing_controller_baseline.VehicleState().speed == 0.0
    assert racing_controller_baseline.VehicleState(x=1.0).speed == 0.0

    state = racing_controller_baseline.VehicleState(
        x=1.0, y=2.0, yaw=0.3, speed=2.5
    )
    assert (state.x, state.y, state.yaw, state.speed) == (1.0, 2.0, 0.3, 2.5)
    state.speed = 4.0
    assert state.speed == 4.0
