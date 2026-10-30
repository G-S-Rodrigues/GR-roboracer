# racing_bringup

Launch files and the one support node the slice needs but no other package owns (Python: ROS-only
scaffolding, kept out of `racing_common`).

## Contract

`support_node.py` owns `/trajectory`, `/track/boundaries`, `/visualization/track` and the
ground-truth `map -> odom` TF (the controller's `/odom` is remapped to `/ground_truth/odom`). Under
an estimator it stops broadcasting `map -> odom` (`publish_map_to_odom: false`) and publishes
`composed_odometry_topic` = latest `map -> odom` o `/odom` in `map`, which the controller drives on
(BRINGUP-1040, BRINGUP-2010). `amcl` starts `nav2_map_server` on the scenario's grid (BRINGUP-1060)
and takes its start pose from `initial_pose_topic` (BRINGUP-2020).

## Running it

Run from the repository root; paths resolve relative to the working directory.
`ros2 launch racing_bringup sim_pure_pursuit.launch.py`. Launch arguments and fields:
`docs/configuration.md`. Tests: `./scripts/check.sh --fast`.

Trap: the planner resolver fails closed (BRINGUP-1160); it never falls back to the centerline,
except `track_centerline` on a map with no `<map>_raceline.csv` (Porto), whose speed is 0, so
`speed_source:=trajectory` there is refused (BRINGUP-1680). `simulator`, `autodrive_bridge` and
`autodrive_port` pick the simulator (`docs/configuration.md`).
