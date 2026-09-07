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
