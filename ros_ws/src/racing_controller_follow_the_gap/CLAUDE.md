# racing_controller_follow_the_gap

Purpose: a scan-only controller that needs no centerline, so it can drive a track nobody has mapped. It is
the mapping driver for AutoDRIVE's Porto (#44 part 11a), added beside Pure Pursuit, which stays the default
(ADR 0005). Selected with `controller:=follow_the_gap`.

Public contract: `/scan` (sensor QoS) to `/controller/drive` (`AckermannDriveStamped`, reliable depth 10,
stamp = the scan's, frame `base_link`), one command per scan, nothing before the first scan. The `hold`
parameter (runtime-settable; the launch argument `controller_hold:=true` starts it true) publishes zero
commands, so the supervisor never stales while the car waits for a reset. After release the speed is capped
by a launch ramp (`launch_acceleration`): a full-throttle standing start spins the wheels and the encoder
odometry then carries a ~0.9 m phantom offset.

Language: Python, a thin node over the ROS-free numpy core `gap.py`; 1081 beams at <= 60 Hz need no C++.

Test: `colcon test --packages-select racing_controller_follow_the_gap` (FTG-1010..1080 tier 1, FTG-2010..2020
tier 2).

Trap: the scan is in frame `lidar`, `inf` means no return (the gym reports 10 m), and the lidar sits 0.1180 m
ahead of the CoG (gotcha 26). Positive steering is left (REP-103).
