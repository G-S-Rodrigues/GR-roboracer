# racing_evaluation

Purpose: score the stack against ground truth — the permanent comparison surface ADR 0005 needs.
Localization is the first tenant; navigation, lap timing and controller scoring are named future ones.

Public contract: `estimate_topic` (nav_msgs/Odometry, best-effort subscription) against
`/ground_truth/odom`, to one `racing_interfaces/LocalizationError` per truth sample on
`/evaluation/localization_error`. `scripts/compare_localization.py` scores a recording offline.

Language: Python, a thin node over the ROS-free `racing_evaluation.localization`; Frenet via
`racing_common`'s binding, never a reimplementation. ament_cmake so tests carry tier labels.

Test: `colcon test --packages-select racing_evaluation` (EVAL-1010..1040 tier 1, EVAL-2010 tier 2).

Trap: a stalled estimator has no error samples, and an error over none is zero. Every measure is
taken over truth samples and carries availability; never report an error without it (EVAL-1030).
