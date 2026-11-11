# racing_recording

Purpose: preserve compact, replayable evidence for every scenario run.

Public contract: run topics to a manifest-first JSONL artifact and `~/flush`, including
`/ground_truth/odom`, the `estimate_topic` pose and `/evaluation/localization_error`, so
`scripts/compare_localization.py` can rescore a run offline (NaN is written as `null`). Each record is written and flushed whole, so a
SIGKILLed node leaves only parseable lines (REC-1010/2010).

Keep `EventRecorder` ROS-free; message serialization stays in the node shell.

Test: `colcon test --packages-select racing_recording`; one tier-2 file:
`launch_test ros_ws/src/racing_recording/test/test_recorder_sigkill.py`.
