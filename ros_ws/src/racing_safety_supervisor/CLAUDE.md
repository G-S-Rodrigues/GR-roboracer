# racing_safety_supervisor

Purpose: independently validate controller commands before `/drive`.

Public contract: `/controller/drive` + `/ground_truth/odom` to `/drive` + `/safety/status`.

Keep `Supervisor` ROS-free; the node only translates messages, time and parameters.

Test: `colcon test --packages-select racing_safety_supervisor`.
