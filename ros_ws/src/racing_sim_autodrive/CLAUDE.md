# racing_sim_autodrive

Purpose: the ROS-free core both AutoDRIVE front ends share (our Socket.IO server, the official devkit bridge), so
neither duplicates a conversion. No network code, no clock, no rclpy.

Public contract: `core.decode_frame` (Socket.IO strings) and `core.frame_from_devkit(DevkitSample)` -> `SimFrame`;
`rear_axle_to_cog`, `frame_pose`, `sim_dt`, `encoder_speed`; `lidar.decode_lidar`; `control.ThrottleLoop`,
`steering_to_normalized`, `ResetLatch`. Vehicle values (lr, wheel radius, throttle gain, max steering angle, PI
gains) are arguments with no defaults. dt is the caller's. A missing key other than the lidar raises `FrameError`.

Language: Python, because the front ends are rclpy and the devkit is Python.

Test: `colcon test --packages-select racing_sim_autodrive --event-handlers console_direct+`
(tier 1, replays `test/fixtures/porto_stream.jsonl.gz`).

Trap: `V1 Position` is the rear axle; publish `rear_axle_to_cog`, never the raw pose (gotcha 26).
