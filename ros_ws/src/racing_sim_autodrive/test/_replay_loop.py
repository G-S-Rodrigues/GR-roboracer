"""The slice of the recorded Porto stream the conformance replay loops.

Not a test: read by `racing_sim_adapter/test/conformance.py` (what the replay
is told) and by SIMAD-1320 (what that slice does), so the two cannot differ.

`(start, end)` is a Python slice, end exclusive: frames 132..337 of
`fixtures/porto_stream.jsonl.gz`. All positions below are CoG (rear axle moved
lr = 0.15532 m along the heading, gotcha 26), as ADAPT-2110 measures them.

  0-39      at rest
  40-82     straight at ~0.9 m/s but sparse (55-90 ms periods)
  83        collision respawn, 0.38 m teleport
  131       collision respawn, 1.072 m teleport back
  132-337   continuous driving, max step 0.062 m, 0.27 to ~1 m/s
  338-~345  throttle 0, decelerating; at rest from ~345
  449-455   the reset pulse: 7.19 m jump back to spawn at frame 451

The slice drives the whole way and holds no teleport. At recorded pace every
2 s window moves the car at least 1.898 m (1.826 m for a window that crosses
the wrap, first to last as ADAPT-2110 computes it; 0.980 m with the wrap jump
taken out). The wrap itself jumps truth 3.52 m back while /odom holds (the lap
clock falls), which can neither fake "moved" (every looped frame is moving) nor
"drift" (an odom that copied truth would jump with it).
"""

PORTO_DRIVING_LOOP = (132, 338)
