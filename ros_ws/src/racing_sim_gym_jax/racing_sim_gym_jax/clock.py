"""Pure simulated-time arithmetic, kept out of the ROS node for testability."""

from builtin_interfaces.msg import Time


def simulated_clock_message(simulation_time_ns: int) -> Time:
    """Split a nanosecond counter into the sec/nanosec pair every message
    stamp and ``/clock`` publish from - one expression, one rollover rule."""
    return Time(
        sec=simulation_time_ns // 1_000_000_000,
        nanosec=simulation_time_ns % 1_000_000_000,
    )


def wall_timer_period(control_period: float, time_scale: float) -> float:
    """How often the sim node's wall timer must fire to advance simulated
    time at ``time_scale`` x real time, ``control_period`` seconds per tick.

    A non-positive ``time_scale`` would stall the graph silently rather than
    raising (a zero or negative timer period never fires), so it is rejected
    here instead.
    """
    if time_scale <= 0.0:
        raise ValueError(f"time_scale must be positive, got {time_scale}")
    return control_period / time_scale


def step_due(
    time_scale: float,
    commanded: bool,
    answered: bool,
    wall_since_publish: float,
    control_period: float,
) -> bool:
    """Whether the sim may step past the tick it last published.

    ADR 0006: ``time_scale`` changes the wall rate, never the run. At 1x the
    command answering the last published tick arrives well inside the 10 ms
    wall period and is applied on the next step; above 1x the wall period
    shrinks below the graph's latency, and without waiting the next step
    would apply a staler command - a different run (SIMJAX-1070). So above
    1x the sim waits until the tick is ``answered``: a command stamped at or
    after it has arrived since it was published.

    It never waits at 1x (the real-time rehearsal: lateness stays visible),
    never without a commander (``commanded`` false - tier 2: nothing to
    wait for), and never longer than one 1x tick (a silent commander slows the
    run to real time, never stalls it).
    """
    if time_scale <= 1.0 or not commanded or answered:
        return True
    return wall_since_publish >= control_period
