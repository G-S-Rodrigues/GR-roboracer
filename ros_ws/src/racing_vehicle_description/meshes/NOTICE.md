# Third-party meshes

`chassis.stl`, `left_wheel.stl`, `right_wheel.stl`, `hinge.stl` and `hokuyo.stl` are copied
unmodified from [f1tenth-dev/simulator](https://github.com/f1tenth-dev/simulator)
(`urdf/meshes/`, `development` branch), licensed Apache License 2.0 — see
`LICENSE-f1tenth-dev-simulator` in this directory.

`urdf/f1tenth.urdf` in this package is **not** a copy of that repository's `urdf/macros.xacro`. It is
a from-scratch, RViz-only URDF written for this repo (fixed joints only, no Gazebo/sensor/plugin/
transmission elements — those are a spec non-goal here) that references these same meshes and reuses
their relative offsets so the model reads correctly, without pulling in the ROS1 Gazebo machinery the
upstream file wires around them.
