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
| `simulator` | `f1tenth_gym_jax` | `f1tenth_gym_jax` | an AutoDRIVE simulator (no identifier yet) | `f1tenth_gym_jax` |
| `pose_source` | `ground_truth`, `slam_toolbox`, `amcl`, `mola` | `ground_truth` | none | `ground_truth` |
| `mapper` | none (racing has no mapper) | `slam_toolbox` | none | n/a |
| `planner` | `track_centerline`, `optimized_raceline` | `track_centerline` | none | `track_centerline` |
| `controller` | `pure_pursuit` | `pure_pursuit` | none | `pure_pursuit` |

`track_centerline` publishes the scenario's `<map>_raceline.csv`, which holds the track centerline
at constant speed. It was called `track_raceline` before part 6 of issue #12 renamed it for what it
is. `optimized_raceline` publishes the committed minimum-curvature line
`config/racelines/<track>/<vehicle version>/<raceline_optimizer>.csv` instead. A missing or
mismatched line is an error: the launch never falls back to the centerline. Exploratory methods
(MPC/MPCC, learned control, FAR-LIO) have no identifier and are not listed here.

The reference default is the composition every new implementation is verified in. Its names come
from `config/reference_stack.yaml`, which is a test map, not this schema.

## Runtime controls

- `seed` and `time_scale` (the ratio of simulated time to wall clock).
- `recording_path`: where the run record is written.
- `drive_on_estimate` (racing only): drive on the estimator's pose instead of ground truth. It is
  ignored with `ground_truth`. `true` is acceptance-backed for `amcl`. No acceptance lap backs it
  for `slam_toolbox` or `mola`, which is measured open loop only, so use `false` there.
- `scenario` and `vehicle`: the track and sensor setup, and the vehicle parameter file.
- `slam_toolbox_config` (`slam_toolbox` only): the parameter file slam_toolbox loads. Empty (the default) lets
  the scenario's lidar choose: `config/localization/slam_toolbox_online.yaml` for 64 beams,
  `config/localization/slam_toolbox_online_1080.yaml` for 1080, and any other beam count is an error that
  names the override. A path replaces the choice (a real car with no scenario passes one). With any other
  pose source it is an error.
- `planner`, `raceline_optimizer` and `speed_source` (racing): which line `/trajectory` carries
  (`track_centerline` by default, or `optimized_raceline`), which optimizer's committed line
  (`tum_mincurv_iqp`, the only one), and how the controller picks its speed (`curvature_schedule`
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
vehicle: config/vehicles/f1tenth_default.yaml  # Available: config/vehicles/f1tenth_default.yaml, config/vehicles/f1tenth_gym_limits.yaml
stack:
  simulator: f1tenth_gym_jax  # Available: f1tenth_gym_jax. Planned: an AutoDRIVE simulator
  pose_source: ground_truth  # Available: ground_truth (the survey drives on exact pose)
  mapper: slam_toolbox  # Available: slam_toolbox (survey mapper, never owns map -> odom)
  planner: track_centerline  # Available: track_centerline
  controller: pure_pursuit  # Available: pure_pursuit
runtime:
  seed: 42  # Available: any non-negative integer
  time_scale: 1.0  # Available: any positive number
  slam_toolbox_config: ""  # Available: empty (chosen by the scenario's num_beams, 64 or 1080) or a parameter file path; slam_toolbox only
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
scenario: config/scenarios/spielberg.yaml  # Available: config/scenarios/spielberg.yaml, config/scenarios/spielberg_1080.yaml (the same track with a 1080-beam, 270 degree lidar), config/scenarios/contract_test.yaml (no localization). Committed but not runnable: config/scenarios/porto.yaml (AutoDRIVE practice track; no simulator runs it until `simulator:=autodrive`)
vehicle: config/vehicles/f1tenth_default.yaml  # Available: config/vehicles/f1tenth_default.yaml, config/vehicles/f1tenth_gym_limits.yaml
stack:
  simulator: f1tenth_gym_jax  # Available: f1tenth_gym_jax. Planned: an AutoDRIVE simulator
  pose_source: amcl  # Available: ground_truth, slam_toolbox, amcl, mola
  planner: track_centerline  # Available: track_centerline, optimized_raceline
  raceline_optimizer: tum_mincurv_iqp  # Available: tum_mincurv_iqp (used by optimized_raceline)
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
| `stack.simulator` | none | One implementation today, always launched. |
| `stack.pose_source` | `pose_source:=` | Only `ground_truth`, `slam_toolbox`, `amcl` and `mola` are accepted; anything else is an error. |
| `stack.mapper` | choose `slam_toolbox_mapping.launch.py` or `map_track.sh` | Instead of `sim_pure_pursuit.launch.py`. The mapping launch takes every racing argument. |
| `stack.planner` | `planner:=` | `track_centerline` (default) or `optimized_raceline`; anything else is an error. |
| `stack.raceline_optimizer` | `raceline_optimizer:=` | `tum_mincurv_iqp`; used only under `optimized_raceline`, but validated always. |
| `stack.controller` | none | One implementation today, always launched. |
| `stack.speed_source` | `speed_source:=` | Also settable in the vehicle file's `racing_controller_baseline` block; a given argument wins. |
| `runtime.seed` | `seed:=` | `map_track.sh` forces `42`. |
| `runtime.time_scale` | `time_scale:=` | `map_track.sh` passes none, so it runs at the default `1.0`. |
| `runtime.slam_toolbox_config` | `slam_toolbox_config:=` | Empty by default: the scenario's `num_beams` picks the file. `slam_toolbox` only; an error with any other pose source. The mapping launch declares it too, but it runs `ground_truth`, so a value there is an error. |
| `runtime.drive_on_estimate` | `drive_on_estimate:=` | Ignored with `ground_truth`. |
| `runtime.recording_path` | `recording_path:=` | `map_track.sh` forces `log/mapping.jsonl`. |
| `mapping.output` | second argument of `map_track.sh` | No launch argument. |
| `visualization.rviz` | `use_rviz:=` | `map_track.sh` forces `false`. |
| (no field) | `start_held:=` | Harness control for t=0. |
| (no field) | `estimate_topic:=` | Overrides the scored pose; default follows the pose source. |

## Next

- [Create or update a map](workflows/mapping.md#survey-mapping)
- [Run a lap with a selected stack](workflows/racing.md#simulation)
