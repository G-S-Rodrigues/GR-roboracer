# racing_evaluation

Purpose: score the stack against ground truth — the permanent comparison surface ADR 0005 needs.
Localization is the first tenant; navigation, lap timing and controller scoring are named future ones.

Public contract: `estimate_topic` (nav_msgs/Odometry, or geometry_msgs/PoseWithCovarianceStamped with
`estimate_type: pose_with_covariance`; best-effort subscription) against
`/ground_truth/odom`, to one `racing_interfaces/LocalizationError` per truth sample on
`/evaluation/localization_error`. `scripts/compare_localization.py` scores a recording offline.
`racing_evaluation.map_accuracy` (ROS-free) scores a produced grid's wall cells against the gym map image's
wall faces: raw cell-centre distance, so a perfect 5 cm map scores 2.5 cm, not 0.

Language: Python, a thin node over the ROS-free `racing_evaluation.localization`; Frenet via
`racing_common`'s binding, never a reimplementation. ament_cmake so tests carry tier labels.

Test: `colcon test --packages-select racing_evaluation` (EVAL-1010..1060, 1500..1540 tier 1, EVAL-2010..2030 tier 2).

Trap: a stalled estimator has no error samples, and an error over none is zero. Every measure is
taken over truth samples and carries availability; never report an error without it (EVAL-1030).
