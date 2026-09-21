"""Pure tests for the sim node's simulated-time and wall-timer arithmetic."""

import pytest
from racing_sim_gym_jax.clock import (
    simulated_clock_message,
    step_due,
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


# One 10 ms control period.
PERIOD = 0.01


def test_simjax_1070_above_1x_a_step_waits_for_the_answering_command():
    """SIMJAX-1070: above 1x the next step waits for a command answering the
    tick the sim last published, so a command is applied the same number of
    ticks after its state at every time_scale.

    Measured before this existed (circle, seed 42): at 1x 1997 of 2000 steps
    applied the command answering the last tick; at 5x 1998 of 2000 applied
    the one before it, which moved p95 0.1365 -> 0.1332 (Spielberg: 0.0297 ->
    0.0237).
    """
    assert not step_due(5.0, True, False, 0.002, PERIOD)
    assert step_due(5.0, True, True, 0.002, PERIOD)


def test_simjax_1070_at_1x_a_step_never_waits():
    """SIMJAX-1070: 1x is the real-time rehearsal (ADR 0006); a late command
    there is the graph's own lateness and stays visible, never absorbed."""
    assert step_due(1.0, True, False, 0.0, PERIOD)


def test_simjax_1070_without_a_commander_a_step_never_waits():
    """SIMJAX-1070: a sim nobody drives (tier 2) runs at its time_scale."""
    assert step_due(5.0, False, False, 0.0, PERIOD)


def test_simjax_1070_a_step_waits_at_most_one_1x_tick():
    """SIMJAX-1070: a commander that stops answering slows the run to 1x at
    worst, never stalls it."""
    assert not step_due(5.0, True, False, 0.0099, PERIOD)
    assert step_due(5.0, True, False, 0.01, PERIOD)
