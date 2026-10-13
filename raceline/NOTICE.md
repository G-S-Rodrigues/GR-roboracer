# Third-party material in `raceline/`

## `tests/fixtures/tum_inputs/`

`racecar_f110.ini`, `veh_dyn_info/ggv.csv` and `veh_dyn_info/ax_max_machines.csv`
are copied unmodified from ForzaETH/race_stack at commit
`202450f` (`stack_master/config/global_planner/`), MIT licence
(`stack_master/LICENSE` at that commit). They are the vehicle inputs TUM's
`trajectory_optimizer` reads from its `input_path`; they are test fixtures,
not this stack's vehicle (raceline part 3 owns that).

## Installed, not vendored

`ForzaETH/global_racetrajectory_optimization` @ `b1b38ec` (LGPL-3.0) and
`trajectory_planning_helpers` (LGPL-3.0) are installed into `tum_env/`'s
virtual environment, not copied into this repository.

## `raceline/extract.py`

A port of the map-to-centerline part of ForzaETH/race_stack at commit
`202450f51081b618fcca71924ad6d33f44f90e44` (branch `ros2-jazzy`; `202450f` for short) (`planner/global_planner/global_planner/global_planner_utils.py`:
`extract_centerline`, `smooth_centerline`, `extract_track_bounds`,
`dist_to_bounds`, `compare_direction`; `global_planner_logic.py`:
`filter_map_occupancy_grid` and the extraction part of
`compute_global_trajectory`), MIT licence. It is a substantial portion, so the
licence text of `planner/global_planner/LICENSE` is reproduced in full. That
file names no copyright holder; the attribution is therefore "Copyright (c)
ForzaETH contributors (no holder is named in the upstream LICENSE)".

```
Copyright (c) ForzaETH contributors (no holder is named in the upstream LICENSE)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

Changed from upstream, and why:

- No ROS (`rclpy`, `f110_msgs`, `ament_index_python` removed): `raceline/` is a
  ROS-free uv project (ADR 0003), and the port is called with arrays.
- No TUM `interp_track` (numpy resampling instead): it is LGPL and lives in the
  py3.8-only `tum_env/`, which this env must not import.
- The loop is chosen by proximity to `start_pose` and enclosed area, not by a
  driven-lap length: an offline map has no driven lap, and a survey has
  contours of every length.
- A clean-up pre-pass before the skeleton (stray wall islands and corridor
  patches filled, free space closed 5x5, spurs pruned): stray islands and
  survey patches otherwise sever the loop or add branches (measured, part 2).
- Track bounds by marching squares at the cell edge: they are then judged
  against the PNG's own free-cell edges, with no half-cell offset of their own.
- Left/right by a cross-product majority vote over every centerline point
  (upstream: the traversal direction of the outer bound at the start): one
  noisy point cannot swap the sides, and a disagreement raises.
- A `ValueError` instead of a distance-transform fallback for the widths: a
  named error is better than a plausible wrong result.
- Cell-centre pixel convention (upstream: cell corner): it is the ROS map
  convention, so the half-cell is not an error of its own.
- Curvature computed from a smoothed periodic spline: upstream computes none in
  these functions, and the loader needs one.
- Corner re-centring (`_recentre`, added in the 2026-10-13 corner clean-up): a
  medial axis cuts a corner tighter than the corridor is wide, so at a corner
  the line is the outer bound moved inward by the half-width. It cut corner
  lateral p95 from 0.65 to 0.08 m and the length shortfall from 1.37 % to
  0.14 % on the gym map (`raceline/README.md`). It assumes a constant half-width.
- The opening kernel: upstream's class default is 3x3 and `raceline/extract.py`
  uses 3x3 (a 9x9 opening is `filter_kernel_size` in upstream's shipped stack config,
  `stack_master/config/global_planner/global_planner_params.yaml:8` at
  `202450f`). The reason for 3x3 is the one `extract.py` gives
  (`_binarize`): the opening would otherwise sever the thin free strips around
  stray walls and survey patches, which the pre-pass fills first.

`raceline/bench/upstream_baseline.py` fetches upstream's
`global_planner_utils.py` at the pinned SHA at run time and checks its sha256;
that file is not vendored. The script does transcribe about ten lines of
upstream's map preparation (`GlobalPlannerLogic`, `global_planner_logic.py`
:250-253 and :516-528, MIT), so the licence block above covers it too.

### Upstream changes since 202450f

Run by hand, never by the gate (the gate must not need the network):

```sh
for b in ros2-jazzy ros2-humble; do gh api repos/ForzaETH/race_stack/compare/202450f...$b \
  --jq '.files[].filename' | grep '^planner/global_planner/'; done
```

How to read it: empty output means nothing under `planner/global_planner/`
changed on that branch; a `404` on stderr means the branch or SHA is wrong, not
"no changes"; on a listed file, port the change by hand and add a row here.

Output on 2026-10-13:

- `ros2-jazzy`: identical to `202450f` (no output).
- `ros2-humble`: nine files: `README.md`, `global_planner_logic.py`,
  `global_planner_node.py`, `global_planner_utils.py`,
  `global_trajectory_publisher.py`, `mapping_node.py`,
  `launch/global_trajectory_publisher_launch.xml`, `rviz/editor.rviz`,
  `setup.py`. The `global_planner_utils.py` diff (read 2026-10-13) removes the
  `map_editor_mode` argument from `extract_centerline` and
  `extract_track_bounds` and its use (`elif map_editor_mode or cent_length ==
  0.0` becomes `elif cent_length == 0.0`; the plot guards lose `and not
  map_editor_mode`). `smooth_centerline` is untouched and the algorithm is
  unchanged; nothing to port.
