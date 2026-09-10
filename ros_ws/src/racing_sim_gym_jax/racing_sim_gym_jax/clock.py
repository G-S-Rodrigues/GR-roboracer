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
