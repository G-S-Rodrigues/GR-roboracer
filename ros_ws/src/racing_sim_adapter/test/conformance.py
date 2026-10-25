"""Capability vocabulary and gating shared by the conformance launch files.

The names mirror ``contract.hpp``; ADAPT-1020 fails if they drift.
"""

CLOCK_PARAMETER = "capability.clock"
RESET_PARAMETER = "capability.reset"
STEP_MODE_PARAMETER = "capability.step_mode"
TIME_SCALE_PARAMETER = "capability.time_scale"
START_HELD_PARAMETER = "capability.start_held"

CAPABILITY_PARAMETERS = (
    CLOCK_PARAMETER,
    RESET_PARAMETER,
    STEP_MODE_PARAMETER,
    TIME_SCALE_PARAMETER,
    START_HELD_PARAMETER,
)
BOOLEAN_PARAMETERS = (
    STEP_MODE_PARAMETER,
    TIME_SCALE_PARAMETER,
    START_HELD_PARAMETER,
)

OWNS_CLOCK = "owns_clock"
FREE_RUNNING = "free_running"
CLOCK_VALUES = (OWNS_CLOCK, FREE_RUNNING)

SEEDED_POSE = "seeded_pose"
INITIAL_POSE_ONLY = "initial_pose_only"
RESET_VALUES = (SEEDED_POSE, INITIAL_POSE_ONLY)
