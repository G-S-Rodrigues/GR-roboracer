# racing_common

This package owns simulator-neutral track geometry and shared racing logic.

- Keep it ROS-free by design; tier 1 depends on this package having no ROS dependency.
- Keep the C++ implementation authoritative. Python consumers use the pybind11 module rather than reimplementing geometry.
- Frenet `d` is positive to the left of the centerline.
- Run `colcon test --packages-select racing_common` for the package tests.
