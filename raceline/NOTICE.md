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
`202450f` (`planner/global_planner/global_planner/global_planner_utils.py`:
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

Changed from upstream: no ROS (`rclpy`, `f110_msgs`, `ament_index_python`
removed); no TUM `interp_track` (numpy resampling); the loop is chosen by
proximity to `start_pose` and enclosed area instead of a driven-lap length;
a clean-up pre-pass before the skeleton (stray wall and corridor patches
filled, free space closed, spurs pruned); track bounds from marching squares at
the cell edge; left/right by a cross-product majority vote over every
centerline point (upstream: traversal direction at the start); a `ValueError`
instead of a distance-transform fallback; cell-centre pixel convention
(upstream: cell corner); curvature computed from a smoothed periodic spline.
