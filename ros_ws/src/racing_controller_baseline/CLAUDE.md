# racing_controller_baseline

Purpose: provide the trusted Pure Pursuit baseline and its thin ROS shell.

Public contract: `/odom` + `/trajectory` to `/controller/drive`.

Keep `PurePursuit` ROS-free and authoritative; Python rollout uses its pybind module.

Test: `colcon test --packages-select racing_controller_baseline` (gtest, tier 1, and the tier-2 launch test PP-2010: `launch_test ros_ws/src/racing_controller_baseline/test/test_controller_node.py`).

Params: `speed_source` (a `speed_source:=` launch argument on sim_pure_pursuit, forwarded only when given: argument > vehicle file > node default) (`curvature_schedule` default | `trajectory`), `lookahead_gain` (default 0). Trap: `trajectory` takes `v` at the CoG's closest point, so a centerline CSV's constant `v = 1.0` drives at 1 m/s; `lookahead_gain` scales by the measured CoG speed from `/odom`'s twist, not by `v`.
