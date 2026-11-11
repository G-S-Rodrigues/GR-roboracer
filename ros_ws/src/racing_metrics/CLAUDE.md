# racing_metrics

Purpose: turn a scenario run into one reproducible metrics record.

Public contract: sensor/state/safety topics to `/scenario/metrics` and JSON. `~/started` latches the first accepted
sample, only under `use_sim_time` (the runner's t=0 waits on it).

Keep `MetricsAccumulator` ROS-free; the node only adapts messages and parameters.

Test: `colcon test --packages-select racing_metrics`.
