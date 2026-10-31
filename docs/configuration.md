# Configuration

> **Target contract, not yet executable.** No launcher reads this YAML yet. Run today's command shown beside it.

Today you select a stack with `ros2 launch` arguments and two dedicated launches. This page
describes the single YAML file a future launcher will accept, and gives today's equivalent of every
field. Each field lists the values that exist now (`Available`) and the ones on the roadmap
(`Planned`). Capability status lives in the [README matrix](../README.md#capabilities-and-roadmap).

## Choose a workflow first

Pick one of two workflows. The choice decides which fields apply and which values are legal.

- `mapping`: build an occupancy-grid map of a track with the survey vehicle.
- `racing`: run a lap with a selected stack.

For example, in `mapping` the only legal `pose_source` is `ground_truth`: the mapper owns the
map, never the pose, so the car drives on exact pose. In `racing`, `slam_toolbox`, `amcl` and `mola` are
pose sources.

## Fixed services

These are always launched and have no field. Nothing selects them.

- `safety_supervisor` reads ground truth in simulation whatever the pose source, so a drifting
  estimate stops the car instead of fooling it.
- `racing_metrics` and `racing_evaluation`. The evaluation (`localization_error`) scores the
  estimate against ground truth.
- `racing_recording`, the support node and `robot_state_publisher`.

## Replaceable roles

| Role | Available (racing) | Available (mapping) | Planned | Reference default |
|---|---|---|---|---|
| `simulator` | `f1tenth_gym_jax`, `autodrive` (in development: see [AutoDRIVE](#the-autodrive-simulator)) | `f1tenth_gym_jax` | none | `f1tenth_gym_jax` |
| `pose_source` | `ground_truth`, `slam_toolbox`, `amcl`, `mola` | `ground_truth` | none | `ground_truth` |
| `mapper` | none (racing has no mapper) | `slam_toolbox` | none | n/a |
| `planner` | `track_centerline`, `optimized_raceline` | `track_centerline` | none | `track_centerline` |
| `controller` | `pure_pursuit` | `pure_pursuit` | none | `pure_pursuit` |

`track_centerline` publishes the scenario's `<map>_raceline.csv`, which holds the track centerline
at constant speed. It was called `track_raceline` before part 6 of issue #12 renamed it for what it
is. `optimized_raceline` publishes the committed minimum-curvature line
`config/racelines/<track>/<vehicle version>/<raceline_optimizer>.csv` instead. A missing or
mismatched line is an error: the launch never falls back to the centerline. The one exception runs
the other way: under `track_centerline` a map with no `<map>_raceline.csv` (Porto has none yet) is
driven on its own centerline, published at speed 0 so the controller's schedule sets the speed;
`speed_source:=trajectory` there is refused, naming the map, because the car would never move. Exploratory methods
(MPC/MPCC, learned control, FAR-LIO) have no identifier and are not listed here.

The reference default is the composition every new implementation is verified in. Its names come
from `config/reference_stack.yaml`, which is a test map, not this schema.

## The AutoDRIVE vehicle

`config/vehicles/autodrive_roboracer.yaml` is the league car's file for the AutoDRIVE simulator
backend (`simulator:=autodrive`, see [The AutoDRIVE simulator](#the-autodrive-simulator)); the gym
cannot run it (it simulates a different car). Its geometry is
read from the sim's own prefab and guide (`config/vehicles/autodrive_roboracer/source.yaml`, every
value anchored to a pinned line) and its measurements are written only by
`tools/autodrive_identify.py` (`identified.yaml`). Its speed, acceleration and steering-rate caps are
the default's, and its laser mount is the true 0.2733 m, which matches its own description,
`autodrive_roboracer.urdf` (the AutoDRIVE RoboRacer model, imported from AutoDRIVE's BSD-2 FBX at full
resolution, about 47 MB). To look at it: `ros2 launch racing_vehicle_description display.launch.py vehicle:=config/vehicles/autodrive_roboracer.yaml`. The measured values (top speed not measured, braking
depending on speed) are under [AutoDRIVE car identification](results.md#autodrive-car-identification).

Its minimum-time dynamics file, `config/vehicles/autodrive_roboracer/tum_mintime.yaml`, is derived from
the same sources and AutoDRIVE's simulator code, with each value's provenance in
`config/vehicles/tum_mintime/reference.yaml`. The car has no tyre set, so `raceline_optimizer:=tum_mintime` cannot run on it yet, and no committed
raceline of either kind (minimum-curvature or minimum-time) exists for it.

To redo the identification for a new edition, start `record` first and the sim after it (the sim
connects once and never retries):

```
env -u PYTHONPATH uv run --locked --script tools/autodrive_identify.py record \
    --config config/autodrive/simulator.yaml --out rec.jsonl.gz
env -u PYTHONPATH uv run --locked --script tools/autodrive_identify.py analyse \
    --recording rec.jsonl.gz \
    --source config/vehicles/autodrive_roboracer/source.yaml \
    --out config/vehicles/autodrive_roboracer/identified.yaml
```

## The AutoDRIVE simulator

`simulator:=autodrive` (racing launch only) starts `racing_sim_autodrive_server` in place of the
gym. It listens on a Socket.IO port that the AutoDRIVE simulator connects to once, so start the
launch first. Its arguments:

- `autodrive_bridge:=ours` (default) or `official`. `official` runs `racing_sim_autodrive_devkit`,
  which talks to the devkit's own bridge (a ROS 2 Humble container) over DDS instead of listening for the
  simulator itself, so `autodrive_port` does not apply to it. The devkit must run on its own ROS domain,
  `autodrive_devkit_domain:=<n>` (an integer 0-101, required with `official`, and different from the
  launch's `ROS_DOMAIN_ID`): the devkit publishes a global `/tf` with a `lidar` frame that would break
  `base_link -> lidar` on the stack's domain, so the launch refuses a shared domain. The contract topics
  are the same as `ours`; the commands reach the car one frame later (the devkit latches each command and
  applies it on the next frame).
- `autodrive_port:=` the port the server listens on (default `4567`, `bridge_port` in
  `config/autodrive/simulator.yaml`).

The server's vehicle values are in `config/vehicles/autodrive_roboracer/server.yaml`, one comment per
value, and where AutoDRIVE's world sits in the Porto map is in `config/autodrive/porto_frame.yaml`
(rotation 90 degrees, translation (-2.9, 1.2) m, derived from two fits and not yet measured from a
lap). The simulator has no time scaling and no held start, so `time_scale` other than 1 and
`start_held:=true` are errors; so are any vehicle but `autodrive_roboracer` and any scenario but
`porto`, the only track the build has active. Every node then runs on wall time (`use_sim_time`
false), because AutoDRIVE publishes no clock. `seed` is accepted; a reset returns to the start pose.

Status: the launch is tested against a recorded AutoDRIVE stream, through either bridge (with a
stand-in for the devkit under `official`), not yet against the live simulator, so the capability table still
lists it as in development.

## Runtime controls

- `simulator`, `seed` and `time_scale` (the ratio of simulated time to wall clock; 1 under `autodrive`).
- `recording_path`: where the run record is written.
- `drive_on_estimate` (racing only): drive on the estimator's pose instead of ground truth. It is
  ignored with `ground_truth`. `true` is acceptance-backed for `amcl`. No acceptance lap backs it
  for `slam_toolbox` or `mola`, which is measured open loop only, so use `false` there.
- `scenario` and `vehicle`: the track and sensor setup, and the vehicle parameter file.
- `planner`, `raceline_optimizer` and `speed_source` (racing): which line `/trajectory` carries
  (`track_centerline` by default, or `optimized_raceline`), which optimizer's committed line
  (`tum_mincurv_iqp`, the default, or `tum_mintime`), and how the controller picks its speed (`curvature_schedule`
  or `trajectory`). `speed_source` is forwarded only when given: a given argument beats the vehicle
  file's `speed_source`, which beats the controller's default `curvature_schedule`.

Two launch arguments have no field. `start_held` holds physics and the clock at t=0 until the
first reset, which the test harness uses to define t=0. `estimate_topic` overrides which pose is
scored, and its default already follows the pose source. Both stay today-only advanced arguments.

## Visualization

`visualization.rviz` opens RViz (`use_rviz:=`). The supported mapping script always turns it off.

## Mapping example

> **Target contract, not yet executable.** No launcher reads this YAML yet. Run today's command shown beside it.

```yaml
# Mapping: survey a track and save its occupancy grid.
schema_version: 1  # Available: 1
workflow: mapping  # Available: mapping, racing
scenario: config/scenarios/spielberg_survey.yaml  # Available: config/scenarios/spielberg_survey.yaml (exact sensors)
vehicle: config/vehicles/f1tenth_default.yaml  # Available: config/vehicles/f1tenth_default.yaml, config/vehicles/f1tenth_gym_limits.yaml. Runs on `simulator: autodrive` only: config/vehicles/autodrive_roboracer.yaml (the AutoDRIVE car; the gym refuses it). The RViz model follows the vehicle (`urdf/descriptions.yaml`); autodrive_roboracer draws the AutoDRIVE RoboRacer
stack:
  simulator: f1tenth_gym_jax  # Available: f1tenth_gym_jax, autodrive (in development; scenario porto and vehicle autodrive_roboracer only)
  pose_source: ground_truth  # Available: ground_truth (the survey drives on exact pose)
  mapper: slam_toolbox  # Available: slam_toolbox (survey mapper, never owns map -> odom)
  planner: track_centerline  # Available: track_centerline
  controller: pure_pursuit  # Available: pure_pursuit
runtime:
  seed: 42  # Available: any non-negative integer
  time_scale: 1.0  # Available: any positive number
  recording_path: log/mapping.jsonl  # Available: any writable path
mapping:
  output: log/maps/Spielberg  # Available: any path stem; writes <stem>.png and <stem>.yaml
visualization:
  rviz: false  # Available: true, false
```

Today, from `/ws` inside the container after the Quick Start:

```bash
# Mapping, supported entry point (headless; saves the map):
./scripts/map_track.sh config/scenarios/spielberg_survey.yaml log/maps/Spielberg
# Mapping with RViz (does not save; the mapping guide covers saving):
ros2 launch racing_bringup slam_toolbox_mapping.launch.py scenario:=config/scenarios/spielberg_survey.yaml seed:=42
```

The committed survey map is `config/scenarios/maps/Spielberg/slam/Spielberg`, and overwriting it
is a reviewed diff. The example writes to `log/maps/Spielberg` so following it is harmless.

## Racing example

> **Target contract, not yet executable.** No launcher reads this YAML yet. Run today's command shown beside it.

```yaml
# Racing: one lap with a selected stack.
schema_version: 1  # Available: 1
workflow: racing  # Available: mapping, racing
scenario: config/scenarios/spielberg.yaml  # Available: config/scenarios/spielberg.yaml, config/scenarios/spielberg_1080.yaml (the same track with a 1080-beam, 270 degree lidar), config/scenarios/contract_test.yaml (no localization). Runs on `simulator: autodrive` only: config/scenarios/porto.yaml (the AutoDRIVE practice track; the gym has no map for it)
vehicle: config/vehicles/f1tenth_default.yaml  # Available: config/vehicles/f1tenth_default.yaml, config/vehicles/f1tenth_gym_limits.yaml. Runs on `simulator: autodrive` only: config/vehicles/autodrive_roboracer.yaml (the AutoDRIVE car; the gym refuses it). The RViz model follows the vehicle (`urdf/descriptions.yaml`); autodrive_roboracer draws the AutoDRIVE RoboRacer
stack:
  simulator: f1tenth_gym_jax  # Available: f1tenth_gym_jax, autodrive (in development; scenario porto and vehicle autodrive_roboracer only)
  pose_source: amcl  # Available: ground_truth, slam_toolbox, amcl, mola
  planner: track_centerline  # Available: track_centerline, optimized_raceline
  raceline_optimizer: tum_mincurv_iqp  # Available: tum_mincurv_iqp (default), tum_mintime (used by optimized_raceline)
  controller: pure_pursuit  # Available: pure_pursuit
  speed_source: curvature_schedule  # Available: curvature_schedule, trajectory (unset: the vehicle file's value, else curvature_schedule)
runtime:
  seed: 42  # Available: any non-negative integer
  time_scale: 1.0  # Available: any positive number
  drive_on_estimate: true  # Available: true, false (ignored with ground_truth)
  recording_path: log/recording.jsonl  # Available: any writable path
visualization:
  rviz: true  # Available: true, false
```

This is amcl closed loop, the demonstration configuration. Today, from `/ws`:

```bash
# Ground truth (the reference stack):
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml pose_source:=ground_truth seed:=42
# slam_toolbox, open loop:
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml pose_source:=slam_toolbox drive_on_estimate:=false seed:=42
# amcl, closed loop:
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml pose_source:=amcl drive_on_estimate:=true seed:=42
# The minimum-curvature line at its own speed, on ground truth:
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml planner:=optimized_raceline raceline_optimizer:=tum_mincurv_iqp speed_source:=trajectory seed:=42
# The minimum-time line, same stack:
ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml planner:=optimized_raceline raceline_optimizer:=tum_mintime speed_source:=trajectory seed:=42
```

`contract_test.yaml` is the analytic circle. Never use it to judge localization: every point on it
looks the same to a scan matcher.

## From fields to today's arguments

| Field | Today | Notes |
|---|---|---|
| `schema_version` | none | Which command you run decides it. |
| `workflow` | none | `mapping` runs `slam_toolbox_mapping.launch.py` or `map_track.sh`; `racing` runs `sim_pure_pursuit.launch.py`. |
| `scenario` | `scenario:=` | Also supplies amcl's map grid. |
| `vehicle` | `vehicle:=` | `map_track.sh` passes no vehicle, so it always uses the launch default `config/vehicles/f1tenth_default.yaml`. Another value needs the direct mapping launch, and that launch does not save a map: the mapping guide covers saving. |
| `stack.simulator` | `simulator:=` | `f1tenth_gym_jax` (default) or `autodrive`; anything else is an error. See [The AutoDRIVE simulator](#the-autodrive-simulator). |
| `stack.pose_source` | `pose_source:=` | Only `ground_truth`, `slam_toolbox`, `amcl` and `mola` are accepted; anything else is an error. |
| `stack.mapper` | choose `slam_toolbox_mapping.launch.py` or `map_track.sh` | Instead of `sim_pure_pursuit.launch.py`. The mapping launch takes every racing argument. |
| `stack.planner` | `planner:=` | `track_centerline` (default) or `optimized_raceline`; anything else is an error. |
| `stack.raceline_optimizer` | `raceline_optimizer:=` | `tum_mincurv_iqp` (default) or `tum_mintime`; used only under `optimized_raceline`, but validated always. |
| `stack.controller` | none | One implementation today, always launched. |
| `stack.speed_source` | `speed_source:=` | Also settable in the vehicle file's `racing_controller_baseline` block; a given argument wins. |
| `runtime.seed` | `seed:=` | `map_track.sh` forces `42`. |
| `runtime.time_scale` | `time_scale:=` | `map_track.sh` passes none, so it runs at the default `1.0`. |
| `runtime.drive_on_estimate` | `drive_on_estimate:=` | Ignored with `ground_truth`. |
| `runtime.recording_path` | `recording_path:=` | `map_track.sh` forces `log/mapping.jsonl`. |
| `mapping.output` | second argument of `map_track.sh` | No launch argument. |
| `visualization.rviz` | `use_rviz:=` | `map_track.sh` forces `false`. |
| (no field) | `start_held:=` | Harness control for t=0. |
| (no field) | `estimate_topic:=` | Overrides the scored pose; default follows the pose source. |

## Next

- [Create or update a map](workflows/mapping.md#survey-mapping)
- [Run a lap with a selected stack](workflows/racing.md#simulation)
