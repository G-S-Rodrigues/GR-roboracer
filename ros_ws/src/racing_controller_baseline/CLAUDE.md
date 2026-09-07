# racing_controller_baseline

Purpose: provide the trusted Pure Pursuit baseline and its thin ROS shell.

Public contract: `/odom` + `/trajectory` to `/controller/drive`.

Keep `PurePursuit` ROS-free and authoritative; Python rollout uses its pybind module.

Test: `colcon test --packages-select racing_controller_baseline`.
