# racing_controller_baseline

Purpose: provide the trusted Pure Pursuit baseline and its thin ROS shell.

Public contract: `/odom` + `/trajectory` to `/controller/drive`.

Keep `PurePursuit` ROS-free and authoritative; Python rollout uses its pybind module.

Test: `colcon test --packages-select racing_controller_baseline`.

Params: `speed_source` (`curvature_schedule` default | `trajectory`), `lookahead_gain` (default 0). Trap: `trajectory` takes `v` at the CoG's closest point, so a centerline CSV's constant `v = 1.0` drives at 1 m/s; `lookahead_gain` scales by the measured CoG speed from `/odom`'s twist, not by `v`.
