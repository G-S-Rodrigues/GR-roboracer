# racing_sim_adapter

This package is the backend-neutral simulator boundary. It contains contract constants and conformance tests, not a simulator implementation.

- Keep backend-specific symbols, imports, and types out of `include/`.
- All backends publish `/scan`, `/odom`, `/imu`, and `/ground_truth/track_relative_state`, subscribe to `/drive`, and provide private `reset` and `step_mode` services. Launched with `start_held: true`, a backend publishes its t=0 snapshot without advancing `/clock` until the first `reset` (ADAPT-2080).
- Preserve the `map`, `base_link`, and `lidar` frame names and the declared QoS profiles.
- A backend declares five read-only capability parameters (`capability.clock|reset|step_mode|time_scale|start_held`, names in `contract.hpp`). Conformance tests that need one are gated in `test/conformance.py` `REQUIREMENTS`; a lacking test prints `NOT APPLICABLE <id>` and returns, never a skip marker (ADAPT-1030, ADAPT-2150).
- ADAPT-2060 is structural and must always pass.
- Conformance launches run under `/test_adapt_<row>` with every absolute name remapped, so tests name topics through `conformance.topic()`, never a literal (ADAPT-1060). Consequence: conformance cannot tell an absolute `/scan` from a relative `scan` (both land under the prefix). The prefix is per row, not per file: two test files of the same row collide if run concurrently on one domain (safe under ctest, which runs one package's targets serially). ADAPT-2170 imports `racing_evaluation/test/graph_isolation.py`, so a change to only that file does not select this package outside the nightly.
