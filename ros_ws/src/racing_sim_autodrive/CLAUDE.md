# racing_sim_autodrive

Purpose: the ROS-free core both AutoDRIVE front ends share (our Socket.IO server, the official devkit bridge), so
neither duplicates a conversion. No network code, no clock, no rclpy.

Public contract: `core.decode_frame` (Socket.IO strings) and `core.frame_from_devkit(DevkitSample)` -> `SimFrame`;
`rear_axle_to_cog`, `frame_pose`, `sim_dt`, `encoder_speed`; `lidar.decode_lidar`; `control.ThrottleLoop`,
`steering_to_normalized`, `ResetLatch`. Vehicle values (lr, wheel radius, throttle gain, max steering angle, PI
gains) are arguments with no defaults. dt is the caller's. A missing key other than the lidar raises `FrameError`.

Also `bridge.BridgeSession` (frame -> reply), `server_node` (script `racing_sim_autodrive_server`, node
`racing_sim`, vehicle params required) and `replay` (stand-in for the sim). Trap: gevent serves on the MAIN
thread, the rclpy executor in a daemon thread, no monkey-patching; join it before exit.

Language: Python (rclpy, devkit).
Test: `colcon test --packages-select racing_sim_autodrive --event-handlers console_direct+` (replays
`test/fixtures/porto_stream.jsonl.gz`); needs the Socket.IO closure on `PYTHONPATH` (testing.md).

Trap: `V1 Position` is the rear axle; publish `rear_axle_to_cog`, never the raw pose (gotcha 26).
SIMAD-1320 reads `racing_sim_adapter/test/conformance.py`; a change to only that file does not select it outside the nightly.
