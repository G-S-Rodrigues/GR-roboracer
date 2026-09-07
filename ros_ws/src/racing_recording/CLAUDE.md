# racing_recording

Purpose: preserve compact, replayable evidence for every scenario run.

Public contract: run topics to a manifest-first JSONL artifact and `~/flush`.

Keep `EventRecorder` ROS-free; message serialization stays in the node shell.

Test: `colcon test --packages-select racing_recording`.
