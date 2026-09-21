# racing_sim_gym_jax

Purpose: expose one warmed `f1tenth_gym_jax` environment through the backend-neutral ROS contract.

Public contract: noisy `/scan` and `/odom` (frame `odom`, plus TF `odom -> base_link`), exact
`/ground_truth/scan` and `/ground_truth/odom` (frame `map`), `/imu`, `/drive`, `~/reset`, and `~/step_mode`.

Language: Python because the simulator is JAX; keep all state evolution in the ROS-free backend.

Test: `colcon test --packages-select racing_sim_adapter --event-handlers console_direct+`.

Trap: call `step_env`, not `step`; `step` silently auto-resets on termination.
