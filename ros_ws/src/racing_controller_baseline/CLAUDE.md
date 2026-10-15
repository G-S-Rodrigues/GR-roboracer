# racing_controller_baseline

Purpose: provide the trusted Pure Pursuit baseline and its thin ROS shell.

Public contract: `/odom` + `/trajectory` to `/controller/drive`. Parameter `reference_offset` (m, along the heading from base_link; the launch sets it from `pure_pursuit_reference`, default `cog_link`).

Keep `PurePursuit` ROS-free and authoritative; Python rollout uses its pybind module.

Test: `colcon test --packages-select racing_controller_baseline`.
