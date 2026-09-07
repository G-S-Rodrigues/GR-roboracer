# racing_vehicle_description

RViz-only URDF and RViz configuration. **No Gazebo, no `<plugin>`, no `<sensor>`, no
`<transmission>`** — Gazebo and sensor simulation are spec non-goals; the automated tests in this
package assert their absence.

## Manual TF-sign check — do this after any change here or to the launch file

The reason this is a manual step and not a numeric test: **a TF sign error passes every numeric
test** and is obvious to a human instantly. Flipping the sign of `base_link_to_laser`'s yaw, or of
the map -> base_link transform `racing_bringup`'s support node broadcasts, produces a robot model and
LiDAR scan that still satisfy every tier-1/2/3 assertion (frame names, message shapes, timing) while
being visibly wrong in RViz.

1. `ros2 launch racing_bringup sim_pure_pursuit.launch.py`
2. Drive (or let Pure Pursuit drive) a **left turn**.
3. Confirm the LiDAR scan (`/scan`, drawn from `laser`) sweeps consistently with the chassis's actual
   heading as it turns — the scan should not appear to lag, mirror, or rotate opposite the chassis.
4. Confirm the chassis moves left in `map`, not right, and that `/visualization/trajectory` curves the
   same way the chassis actually turns.
