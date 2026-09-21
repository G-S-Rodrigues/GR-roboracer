# racing_sim_gym_jax

Purpose: expose one warmed `f1tenth_gym_jax` environment through the backend-neutral ROS contract.

Public contract: noisy `/scan` and `/odom` (frame `odom`, plus TF `odom -> base_link`), exact
`/ground_truth/scan` and `/ground_truth/odom` (frame `map`), `/imu`, `/drive`, `~/reset`, and `~/step_mode`.
Above 1x, while a `/drive` publisher exists, each step waits up to one 1x tick for a `/drive` stamped
at or after the tick last published (SIMJAX-1070): a commander must stamp `/drive` with the tick it answers.

Language: Python because the simulator is JAX; keep all state evolution in the ROS-free backend.

Test: `colcon test --packages-select racing_sim_adapter --event-handlers console_direct+`.

Trap: call `step_env`, not `step`; `step` silently auto-resets on termination.
