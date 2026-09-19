# racing_sim_adapter

This package is the backend-neutral simulator boundary. It contains contract constants and conformance tests, not a simulator implementation.

- Keep backend-specific symbols, imports, and types out of `include/`.
- All backends publish `/scan`, `/odom`, `/imu`, and `/ground_truth/track_relative_state`, subscribe to `/drive`, and provide private `reset` and `step_mode` services. Launched with `start_held: true`, a backend publishes its t=0 snapshot without advancing `/clock` until the first `reset` (ADAPT-2080).
- Preserve the `map`, `base_link`, and `laser` frame names and the declared QoS profiles.
- ADAPT-2010 through ADAPT-2050 remain explicitly skipped until Step 9 supplies the first backend; Step 9 removes those markers.
- ADAPT-2060 is structural and must always pass.
