# racing_safety_supervisor

Purpose: independently validate controller commands before `/drive`.

Public contract: `/controller/drive` + `/ground_truth/odom` to `/drive` + `/safety/status`; `~/clock_received` (latched first `/clock`, only under `use_sim_time`; read by the t=0 wait).

Keep `Supervisor` ROS-free; the node only translates messages, time and parameters.

Test: `colcon test --packages-select racing_safety_supervisor` (gtest tier 1; tier 2 `test/test_supervisor_clock_received.py`).
