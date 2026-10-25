"""What the gym_jax backend can do, as the contract's capability parameters.

Standard library only: a launch can read this without starting the node or
importing JAX. ``node.py`` declares each entry as a read-only parameter.
"""

CAPABILITIES: dict[str, str | bool] = {
    # The node publishes /clock and advances it itself.
    "capability.clock": "owns_clock",
    # A seeded reset reproduces the same initial pose.
    "capability.reset": "seeded_pose",
    "capability.step_mode": True,
    "capability.time_scale": True,
    "capability.start_held": True,
}
