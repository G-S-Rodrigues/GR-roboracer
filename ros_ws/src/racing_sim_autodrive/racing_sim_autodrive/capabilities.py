"""What the AutoDRIVE backend can do, as the contract's capability parameters.

Standard library only: a launch can read this without starting the node, ROS
or numpy (the gym's `capabilities.py` has the same shape). The server node
declares each entry as a read-only parameter. Part 6's vocabulary.
"""

CAPABILITIES: dict[str, str | bool] = {
    # The Unity sim runs on its own wall clock; there is no /clock.
    "capability.clock": "free_running",
    # `V1 Reset` returns the car to its one start pose; no seed exists.
    "capability.reset": "initial_pose_only",
    # The protocol has no lock-step control, no time scaling, no hold.
    "capability.step_mode": False,
    "capability.time_scale": False,
    "capability.start_held": False,
}
