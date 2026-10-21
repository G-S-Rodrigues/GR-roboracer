# Racing

This page assumes the [Quick Start](../quick-start.md). The target configuration for a racing run
is in [Configuration](../configuration.md#racing-example). Every command runs from `/ws` inside the
container, because the default vehicle path is relative.

## Simulation

`f1tenth_gym_jax` is the one simulator. Three scenarios exist:

- `contract_test.yaml`: the analytic circle, and the default. Never use it to judge localization.
- `spielberg.yaml`: the league track, with sensor noise.
- `spielberg_survey.yaml`: Spielberg with exact sensors, for [mapping](mapping.md) only.

Shared arguments: `seed:=`, `time_scale:=` (scales the wall rate, never the physics),
`use_rviz:=false` for a headless run, and `vehicle:=`. For headless metrics runs see
[Running](../running.md); for headless JAX rollouts see
[Setup](../setup.md#optional-the-gpu-simulator).

| Selection | The car drives on | Owns `map -> odom` | Section |
|---|---|---|---|
| `ground_truth` | ground truth | support node | [Ground truth](#ground-truth-reference-stack) |
| `slam_toolbox` + `drive_on_estimate:=false` | ground truth | slam_toolbox | [slam_toolbox](#online-slam-toolbox-evaluation) |
| `amcl` | the amcl estimate | amcl | [amcl](#amcl-closed-loop) |

## Ground-truth reference stack

**Drives on:** ground truth, the simulator's exact pose.
**Selected by:** `pose_source:=ground_truth`, which is the default.

```bash
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml pose_source:=ground_truth seed:=42
```

The safety supervisor always reads ground truth, whatever the pose source. With no arguments the
launch runs the analytic circle from the Quick Start.

## Online SLAM Toolbox evaluation

**Drives on:** ground truth. The slam_toolbox estimate is scored, not driven on.
**Selected by:** `pose_source:=slam_toolbox drive_on_estimate:=false`.

```bash
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml pose_source:=slam_toolbox drive_on_estimate:=false seed:=42
```

Driving on this estimate latches a `TRACK_LIMIT` stop, so keep `drive_on_estimate:=false`. Use
Spielberg: the estimate cannot be judged on the analytic circle.

## AMCL closed loop

**Drives on:** the amcl estimate (`/localization/odom`).
amcl localizes on the committed survey grid, which `nav2_map_server` serves.
**Selected by:** `pose_source:=amcl drive_on_estimate:=true`.

```bash
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml pose_source:=amcl drive_on_estimate:=true seed:=42
```

`drive_on_estimate` defaults to `true`, so `pose_source:=amcl` alone is also closed loop. For open
loop, with the car still on ground truth, pass `drive_on_estimate:=false`. amcl publishes its
particles on `/particle_cloud`; viewing them in RViz needs `nav2_rviz_plugins`, which the image does
not install.

## Planning and raceline generation

Available: `track_raceline`. It publishes the scenario's `<map>_raceline.csv` on `/trajectory`,
which today is the centerline at constant speed. It has no launch argument, because it is the only
planner.

Available, offline, not selectable by launch: map centerline extraction and the TUM
minimum-curvature wrapper in `raceline/`. `env -u PYTHONPATH uv run --project raceline --locked
python -m raceline` optimizes a line from a map or track, verifies it and writes
`config/racelines/<track>/<vehicle_version>/<optimizer>.{csv,yaml}`; min-curvature racelines for
Spielberg are committed under `config/racelines/spielberg/`. No planner follows them yet. See the
[raceline README](../../raceline/README.md#racelines-part-4) and [Path planning](../path-planning.md).

Planned: `track_centerline`, a rename of today's planner.
In development (issue #12): an optimized-raceline planner.
Neither has a launch argument, and there is nothing to run.

## Control

Available: `pure_pursuit`. By default (`speed_source=curvature_schedule`) it sets speed from path
curvature, not from trajectory speed, and the independent safety supervisor clamps its commands.
The opt-in node parameter `speed_source=trajectory` follows the trajectory's speed instead, which
on the current centerline CSV is a constant 1.0 m/s; `lookahead_gain` defaults to 0. These are node
parameters, not launch arguments. It is the only controller, so it has no launch
argument.

Exploratory: tracking and dynamic MPC, and MPCC. None has an identifier yet.

## Next

- [Mapping](mapping.md#survey-mapping)
- [Configuration](../configuration.md)
