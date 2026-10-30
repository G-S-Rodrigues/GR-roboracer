# Mapping

This page assumes the [Quick Start](../quick-start.md): the workspace is built and you are in a
shell inside the container, in `/ws`. The target configuration for a mapping run is in
[Configuration](../configuration.md#mapping-example).

## Survey mapping

A survey is one Spielberg lap of the reference stack. The car drives on ground truth
(`pose_source: ground_truth`) on `spielberg_survey.yaml`, which is Spielberg with exact sensors.
While it drives, slam_toolbox (`mapper: slam_toolbox`) keeps a scan every 0.2 m or 0.1 rad and
places it at the dead-reckoned pose, with scan matching and loop closure turned off. That pose is
exact only because the survey scenario has no noise. It never publishes `map -> odom`.

The result is a generated survey map, not an autonomous SLAM result. It does not show how well
slam_toolbox can build a map from a drive it has to localize itself.

## Watch the map grow in RViz

From `/ws`, with RViz on (the default):

```bash
ros2 launch racing_bringup slam_toolbox_mapping.launch.py scenario:=config/scenarios/spielberg_survey.yaml seed:=42
```

RViz's `Map` display shows `/map`, which the mapper republishes every 5 s, next to the scan, the car
and the track. Let the car complete one lap. To confirm the lap finished, echo the latched
metrics from a second shell (see the next section for how to open one):

```bash
ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /scenario/metrics racing_interfaces/msg/ScenarioMetrics
```

The output includes `lap_completed: true` once the lap is done.

## Save the map

The RViz launch does not save the map. Before you stop it, open a second shell from the host, and
source the workspace there:

```bash
docker exec -it gr-roboracer-dev bash
cd /ws
source /ws/setup.sh
```

Then save the latest `/map`. The flags matter. `map_saver_cli`'s own arguments come first and
`--ros-args` last; the other order is silent and writes a default `map_<epoch>.pgm` instead.

```bash
mkdir -p log/maps
ros2 run nav2_map_server map_saver_cli -f log/maps/Spielberg --fmt png --occ 0.65 --free 0.196 --ros-args -p use_sim_time:=true -p save_map_timeout:=60.0 -p map_subscribe_transient_local:=true
```

Stop the launch with Ctrl+C afterwards. Save after the lap: the last `/map` covers the whole lap.

## Headless: one command

`map_track.sh` does the lap and the save without RViz:

```bash
./scripts/map_track.sh config/scenarios/spielberg_survey.yaml log/maps/Spielberg
```

It fixes the seed at 42, runs at 1x, and gives up if no lap completes within 240 s (the
`LAP_WAIT` environment variable changes that). Give it
`config/scenarios/maps/Spielberg/slam/Spielberg` as the stem only to replace the committed grid,
which is a reviewed change.

## Saved artifacts

The stem you give produces two files:

- `<stem>.png`: the occupancy grid, trinary, 0.05 m per cell.
- `<stem>.yaml`: resolution, origin and the thresholds 0.65 (occupied) and 0.196 (free).

No pose graph is saved. The files belong to the container user, and two runs differ slightly
because the asynchronous mapper picks its own scans. The committed survey grid is
`config/scenarios/maps/Spielberg/slam/Spielberg`, and it is what amcl localizes on in
[Racing](racing.md#amcl-closed-loop). Denser 0.025 m and 0.02 m surveys (minimum travel 2 cm over two
laps at 1.5 m/s; the mapper dropped scans throughout, so fewer were integrated) sit beside it in
`config/scenarios/maps/Spielberg/slam_0025/` and `slam_0020/`; their headers say how they were made.
`./scripts/map_track.sh` does not produce them (non-default slam_toolbox settings and a longer
scenario), and no launch selects them: amcl reads `slam/`.

## Mapping versus online SLAM

In mapping, slam_toolbox only builds the map. The car drives on ground truth and the support node
owns `map -> odom`. As a racing pose source (`pose_source:=slam_toolbox`), slam_toolbox matches
scans, owns `map -> odom`, saves no map, and its estimate is scored against ground truth. See
[online slam_toolbox evaluation](racing.md#online-slam-toolbox-evaluation).

## The MOLA map

MOLA-LO builds its own map with one mapping lap. From `/ws`, after a `colcon build`:

```bash
./scripts/mola_map_track.sh config/scenarios/spielberg_1080.yaml config/scenarios/maps/Spielberg/mola/Spielberg 7
```

The car drives one lap at 1x on ground truth while `mola_mapping.launch.py` runs MOLA-LO's lidar
pipeline beside it. MOLA reads only `/scan` and `/odom`, never ground truth. The script writes:

- `Spielberg.mm`, MOLA's native map, which `pose_source:=mola` localizes on;
- `Spielberg_grid.png` and `Spielberg_grid.yaml`, an occupancy grid in the world frame, in the
  `map_server` format that `amcl`, the raceline extractor and `racing_evaluation` read unchanged;
- `Spielberg.yaml`, the provenance: the command, seed, tool versions, file hashes and `T_map_mola`.

`T_map_mola` is the car's ground-truth pose when MOLA processed its first scan. It places the map in
the world and is used only to score the run and to export the grid. A real car has no ground truth, so
there `T` is the identity and the map stays in MOLA's own frame. Loop closure is not used: on this 2D
data MOLA accepted none.

To watch it in RViz, start the stack in one shell with `start_held:=true`, run
`ros2 launch racing_bringup mola_mapping.launch.py simplemap_output:=/tmp/lap.simplemap` in
another, then call the simulator's `~/reset`. Its arguments are in
[Configuration](../configuration.md#mola-mapping-launch).

Mapping is not reproducible: two runs of the same command give different maps (one lap with no loop
closure drifts differently each time). The committed map is one such run, reviewed as a diff. Its
quality against the track's true walls is in [Results](../results.md#mola-map-accuracy).

## Demonstration

A recorded mapping run is in [Results](../results.md#mapping-demonstration).

## Next

- [Racing](racing.md#amcl-closed-loop) on the saved map.
- [Configuration](../configuration.md).
