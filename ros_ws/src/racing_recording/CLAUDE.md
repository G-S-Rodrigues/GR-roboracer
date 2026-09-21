# racing_recording

Purpose: preserve compact, replayable evidence for every scenario run.

Public contract: run topics to a manifest-first JSONL artifact and `~/flush`, including
`/ground_truth/odom`, the `estimate_topic` pose and `/evaluation/localization_error`, so
`scripts/compare_localization.py` can rescore a run offline (NaN is written as `null`).

Keep `EventRecorder` ROS-free; message serialization stays in the node shell.

Test: `colcon test --packages-select racing_recording`.
