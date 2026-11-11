"""ROS-free body-frame acceleration for the sim's /imu.

The gym state carries no acceleration, so /imu's `linear_acceleration` is
derived from what the snapshots say happened (plan D5): the speed change over
the step, not the commanded acceleration.
"""

from __future__ import annotations

# The gym's own constant (f1tenth_gym_jax dynamic_models.py: `g = 9.81`, a
# local there, so it cannot be imported). One value, not a third.
GRAVITY_M_S2 = 9.81


def body_acceleration(
    previous_speed: float | None,
    speed: float,
    yaw_rate: float,
    dt: float,
) -> tuple[float, float, float]:
    """Specific force (a_x, a_y, a_z) in base_link, m/s^2.

    a_x = (v - v_previous) / dt, a_y = v * yaw_rate, a_z = +g: an
    accelerometer reads the reaction to gravity, so a car at rest reads +g up.
    `previous_speed` is None on the first step after a reset, where a_x is 0
    rather than a spike from a stale speed.

    Limitation: a_y = v * yaw_rate is the no-slip centripetal term. It ignores
    the slip-angle rate, which the Snapshot does not expose.
    """
    longitudinal = (
        0.0 if previous_speed is None else (speed - previous_speed) / dt
    )
    return longitudinal, speed * yaw_rate, GRAVITY_M_S2
