# racing_sim_adapter

This package is the backend-neutral simulator boundary. It contains contract constants and conformance tests, not a simulator implementation.

- Keep backend-specific symbols, imports, and types out of `include/`.
- All backends publish `/scan`, `/odom`, `/imu`, and `/ground_truth/track_relative_state`, subscribe to `/drive`, and provide private `reset` and `step_mode` services. Launched with `start_held: true`, a backend publishes its t=0 snapshot without advancing `/clock` until the first `reset` (ADAPT-2080).
- Preserve the `map`, `base_link`, and `laser` frame names and the declared QoS profiles.
- A backend declares five read-only capability parameters (`capability.clock|reset|step_mode|time_scale|start_held`, names in `contract.hpp`). Conformance tests that need one are gated in `test/conformance.py` `REQUIREMENTS`; a lacking test prints `NOT APPLICABLE <id>` and returns, never a skip marker (ADAPT-1030, ADAPT-2150).
- ADAPT-2060 is structural and must always pass.
