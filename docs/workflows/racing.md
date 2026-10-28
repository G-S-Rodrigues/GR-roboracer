# Racing

This page assumes the [Quick Start](../quick-start.md). The target configuration for a racing run
is in [Configuration](../configuration.md#racing-example). Every command runs from `/ws` inside the
container, because the default vehicle path is relative.

## Simulation

`f1tenth_gym_jax` is the one simulator. Four scenarios exist:

- `contract_test.yaml`: the analytic circle, and the default. Never use it to judge localization.
- `spielberg.yaml`: the league track, with sensor noise.
- `spielberg_survey.yaml`: Spielberg with exact sensors, for [mapping](mapping.md) only.
- `porto.yaml`: AutoDRIVE's Porto practice track (32.71 m lap). No simulator runs it until `simulator:=autodrive` exists; the gym has no Porto map.

Shared arguments: `seed:=`, `time_scale:=` (scales the wall rate, never the physics),
`use_rviz:=false` for a headless run, and `vehicle:=`. For headless metrics runs see
[Running](../running.md); for headless JAX rollouts see
[Setup](../setup.md#optional-the-gpu-simulator).

| Selection | The car drives on | Owns `map -> odom` | Section |
|---|---|---|---|
| `ground_truth` | ground truth | support node | [Ground truth](#ground-truth-reference-stack) |
| `slam_toolbox` + `drive_on_estimate:=false` | ground truth | slam_toolbox | [slam_toolbox](#online-slam-toolbox-evaluation) |
| `amcl` | the amcl estimate | amcl | [amcl](#amcl-closed-loop) |
| `mola` + `drive_on_estimate:=false` | ground truth | MOLA-LO (via `mola_map`) | [mola](#mola-open-loop) |

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

## MOLA open loop

**Drives on:** ground truth; MOLA-LO's estimate is published on `/localization/odom` and scored.
MOLA localizes on the map it built itself in one mapping lap, committed in
`config/scenarios/maps/Spielberg/mola/` (`Spielberg.mm` and the provenance `Spielberg.yaml`), not on
the survey grid, so its error includes that map's own error.
**Selected by:** `pose_source:=mola drive_on_estimate:=false`.

```bash
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg_1080.yaml pose_source:=mola drive_on_estimate:=false seed:=42
```

Use the 1080-beam scenario at 1x: scan matching is wall-clock work, so a faster simulated clock drops
scans. The support node publishes no estimate until MOLA's pose is within 5 m of the start pose it is told
on `/initialpose` (a plausibility check on MOLA's own output, with no yaw check). Launching it with
`drive_on_estimate:=true`, the default, logs a warning and is allowed. No closed-loop lap is backed for `mola`: its worst error, 3.1 m, is above the
1.83 m `amcl` closed loop is accepted at.

## Planning and raceline generation

Selected by `planner:=`, which is `track_centerline` by default.

- `track_centerline` publishes the scenario's `<map>_raceline.csv` on `/trajectory`, which is the
  centerline at constant speed. (It was called `track_raceline` before part 6 of issue #12.)
- `optimized_raceline` publishes the committed minimum-curvature line
  `config/racelines/<track>/<vehicle_version>/<raceline_optimizer>.csv`, with
  `raceline_optimizer:=tum_mincurv_iqp` (the only optimizer). A missing line, an unknown name or a
  line whose sidecar `.yaml` disagrees is an error; the launch never falls back to the centerline.

```bash
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml planner:=optimized_raceline raceline_optimizer:=tum_mincurv_iqp speed_source:=trajectory seed:=42
```

`speed_source:=trajectory` makes the car follow the line's own speed, a flat 3.0 m/s for the
default vehicle. Without it the controller keeps its curvature schedule, whatever the line. On
Spielberg, ground truth, seed 42, 5x, that lap measured 113.08 s (test SIM-3110) against 120.41 s
for the centerline lap on the curvature schedule. The gym-limits vehicle's line resolves, but no
gate lap drives it.

Available, offline: map centerline extraction and the TUM minimum-curvature wrapper in
`raceline/`. `env -u PYTHONPATH uv run --project raceline --locked python -m raceline` optimizes a
line from a map or track, verifies it and writes
`config/racelines/<track>/<vehicle_version>/<optimizer>.{csv,yaml}`; min-curvature racelines for
Spielberg are committed under `config/racelines/spielberg/`. See the
[raceline README](../../raceline/README.md#racelines-part-4) and [Path planning](../path-planning.md).

Minimum-time optimization has not converged (see [Verification](../verification.md)).

## Control

Available: `pure_pursuit`. By default (`speed_source=curvature_schedule`) it sets speed from path
curvature, not from trajectory speed, and the independent safety supervisor clamps its commands.
The opt-in `speed_source=trajectory` follows the trajectory's speed instead, which in the published
`<map>_raceline.csv` (the centerline) is a constant 1.0 m/s and on a committed raceline is the
line's own speed. Set it either with the launch argument `speed_source:=trajectory`, or in the
`racing_controller_baseline` block of a vehicle file (see `config/vehicles/f1tenth_default.yaml`)
passed with `vehicle:=`. A given argument beats the vehicle file, which beats the default
`curvature_schedule`. `lookahead_gain` sets lookahead = max(`lookahead_distance`, gain x measured
speed); it defaults to 0, so it is off, and it is a node parameter only: set it in the vehicle
file. The node reads its parameters only at start-up, so `ros2 param set` has no effect.
It is the only controller, so it has no launch argument.

Exploratory: tracking and dynamic MPC, and MPCC. None has an identifier yet.

## Next

- [Mapping](mapping.md#survey-mapping)
- [Configuration](../configuration.md)
