"""Pure tests for the body-frame acceleration the sim's /imu reports."""

import pytest
from racing_sim_gym_jax.imu import GRAVITY_M_S2, body_acceleration


def test_simjax_1520_body_acceleration_is_what_happened():
    """SIMJAX-1520: a_x is the speed change per step, a_y = v * yaw rate (the
    centripetal term of a body that does not slip), a_z = +g (the specific
    force of a body at rest on the ground). Three literal cases."""
    assert GRAVITY_M_S2 == 9.81
    # Accelerating straight: 1.0 -> 1.05 m/s in 10 ms is 5 m/s^2.
    ax, ay, az = body_acceleration(1.0, 1.05, 0.0, 0.01)
    assert (ax, ay, az) == (pytest.approx(5.0), 0.0, 9.81)
    # Constant speed, turning left and right: a_y = v * omega, a_x = 0.
    ax, ay, az = body_acceleration(2.0, 2.0, 0.5, 0.01)
    assert (ax, ay, az) == (0.0, pytest.approx(1.0), 9.81)
    ax, ay, az = body_acceleration(2.0, 2.0, -0.5, 0.01)
    assert (ax, ay, az) == (0.0, pytest.approx(-1.0), 9.81)


def test_simjax_1530_first_step_after_a_reset_has_no_speed_spike():
    """SIMJAX-1530: with no previous speed (the first step after a reset) a_x
    is 0.0, not (v - stale) / dt; a_y and a_z are still computed."""
    ax, ay, az = body_acceleration(None, 3.0, 0.2, 0.01)
    assert ax == 0.0
    assert ay == pytest.approx(0.6)
    assert az == 9.81
