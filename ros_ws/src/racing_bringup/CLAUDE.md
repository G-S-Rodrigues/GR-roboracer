# racing_bringup

Launch files and the one support node the vertical slice needs but no other package owns.

## Why this package exists

Every other node in the slice is a thin shell around a ROS-free class (see each sibling package's
own `CLAUDE.md`). None of them are responsible for the three things a human watching RViz actually
needs: the raceline on `/trajectory`, the track geometry on `/track/boundaries` and
`/visualization/track`, and a `map -> base_link` TF (`racing_sim_gym_jax` publishes `/odom`, never a
transform). `support_node.py` is the ROS-only home for those three gaps — it is scaffolding, not
domain logic, and stays out of `racing_common`.

## Running it

```
ros2 launch racing_bringup sim_pure_pursuit.launch.py
```

Run from the repository root. The launch file and every node it starts resolve
`config/vehicles/f1tenth_default.yaml` and the scenario/track paths as paths relative to the current
working directory — the same convention `scripts/check.sh` and each node's own parameter defaults
already use.

Launch arguments: `scenario`, `seed`, `recording_path`, `use_rviz`.
