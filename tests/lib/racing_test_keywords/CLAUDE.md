# racing_test_keywords

Purpose: run the real `racing_bringup` graph headlessly and read back its outcome, for tiers 3 and 4.

Not an ament package — a plain `PYTHONPATH` library (`scripts/check.sh` adds `tests/lib` for both
`pytest tests/system` via `tests/system/conftest.py` and `robot --pythonpath tests/lib
tests/acceptance`). It needs `rclpy` and `racing_interfaces`, already on the path once the ROS overlay
is sourced; it needs no `colcon build` step of its own.

`scenario_runner.run_scenario` drives the actual launch file
(`racing_bringup/launch/sim_pure_pursuit.launch.py`), not a parallel test-only graph — a passing tier-3
or tier-4 test proves the same thing `ros2 launch` gives a human. `keywords.py` is a thin Robot
Framework adapter over it; put new capability in `scenario_runner`/`ros_helpers` first, since tier 3
exercises it directly and more cheaply than a `.robot` suite can.

Test: `pytest tests/system` (tier 3), `robot --pythonpath tests/lib tests/acceptance` (tier 4).
