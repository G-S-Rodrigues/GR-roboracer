"""SIMAD-1480: the devkit domain rule of the part-9 test harness."""

import _devkit_launch as dl
import pytest


@pytest.mark.parametrize(
    "stack,devkit", [(0, 1), (45, 46), (100, 101), (101, 100)]
)
def test_simad_1480_devkit_domain_is_s_plus_1_or_s_minus_1_past_101(
    stack, devkit
):
    """SIMAD-1480: the devkit domain is S+1, or S-1 when S+1 is past the last
    valid domain (101), and never the stack domain itself."""
    assert dl.devkit_domain(stack) == devkit
    assert dl.devkit_domain(stack) != stack
