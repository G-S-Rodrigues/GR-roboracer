# racing_metrics

Purpose: turn a scenario run into one reproducible metrics record.

Public contract: sensor/state/safety topics to `/scenario/metrics` and JSON.

Keep `MetricsAccumulator` ROS-free; the node only adapts messages and parameters.

Test: `colcon test --packages-select racing_metrics`.
