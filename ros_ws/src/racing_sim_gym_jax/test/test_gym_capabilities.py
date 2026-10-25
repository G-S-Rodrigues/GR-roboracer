"""Tier-1: the gym backend's static capability declaration."""

import subprocess
import sys

from racing_sim_gym_jax.capabilities import CAPABILITIES


def test_simjax_1150_gym_declares_every_capability_without_jax() -> None:
    """SIMJAX-1150: the gym owns the clock, seeds its resets and has
    step_mode, time_scale and start_held; the module that says so imports
    neither jax nor rclpy, so a launch can read it without a node."""
    assert CAPABILITIES == {
        "capability.clock": "owns_clock",
        "capability.reset": "seeded_pose",
        "capability.step_mode": True,
        "capability.time_scale": True,
        "capability.start_held": True,
    }
    probe = (
        "import sys; import racing_sim_gym_jax.capabilities; "
        "bad = [m for m in ('jax', 'rclpy') if m in sys.modules]; "
        "sys.exit(1 if bad else 0)"
    )
    done = subprocess.run([sys.executable, "-c", probe], check=False)
    assert done.returncode == 0
