"""Pure tests for the sim node's simulated-time and wall-timer arithmetic."""

import pytest
from racing_sim_gym_jax.clock import (
    simulated_clock_message,
    wall_timer_period,
)


def test_simjax_1030_simulated_clock_message_splits_sec_and_nanosec():
    """SIMJAX-1030: sec/nanosec split, including the ns rollover boundary."""
    zero = simulated_clock_message(0)
    assert zero.sec == 0
    assert zero.nanosec == 0

    mid_second = simulated_clock_message(1_500_000_000)
    assert mid_second.sec == 1
    assert mid_second.nanosec == 500_000_000

    just_below_rollover = simulated_clock_message(999_999_999)
    assert just_below_rollover.sec == 0
    assert just_below_rollover.nanosec == 999_999_999


def test_simjax_1040_wall_timer_period_scales_by_time_scale():
    """SIMJAX-1040: wall_timer_period scales, and rejects time_scale <= 0."""
    assert wall_timer_period(0.01, 1.0) == pytest.approx(0.01)
    assert wall_timer_period(0.01, 5.0) == pytest.approx(0.002)


@pytest.mark.parametrize("time_scale", [0.0, -1.0])
def test_simjax_1040_wall_timer_period_rejects_non_positive_time_scale(
    time_scale,
):
    with pytest.raises(ValueError, match="time_scale"):
        wall_timer_period(0.01, time_scale)
