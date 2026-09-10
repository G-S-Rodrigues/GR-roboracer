# ADR 0003 — `sim/` lives outside `ros_ws/`: two products, one physics

**Status:** accepted
**Date:** 2026-09-09
**Supersedes:** none
**Amended:** 2026-09-10 — one sentence of Context below, by ADR 0006. The decision is unaffected.

## Context

There are two ways to run this vehicle against `f1tenth_gym_jax`, and they want opposite things:

- **`ros_ws/src/racing_sim_gym_jax`** — a ROS node stepping the env on a timer at the control
  period, inside a live DDS graph, so a human can watch it in RViz and so the rest of the graph
  experiences it the way it will experience real hardware. **It runs at real time by default**
  (ADR 0006 made the rate a `time_scale` parameter; this ADR originally said "bound to real time by
  construction", which is no longer true). What matters to *this* decision is unchanged: it is
  timer-driven and graph-bound, where `sim/rollout.py` is a tight headless loop.
- **`sim/rollout.py`** — a headless batch harness stepping the env in a tight loop with no ROS at
  all, used to generate `tests/golden/baseline.json` and, later, to sweep parameters. It wants a GPU
  and as many steps per second as it can get.

The obvious move is to make `sim/` an ament package inside `ros_ws/` so there is "one workspace".
The reason not to is concrete: `sim/` is a `uv`-managed project with a CUDA JAX lockfile, and a
uv-managed venv inside a workspace ament also scans is a known fight over Python discovery. Putting
them together means either the ROS build sees a CUDA JAX environment it should not, or `sim/` gives
up its lockfile.

## Decision

**`sim/` is a separate `uv` project at the repository root, outside `ros_ws/`, and runs in its own
`fast-sim` image with no ROS in it.** `ros_ws/` builds with `colcon` in `dev`, which has no CUDA.

**The physics is not duplicated.** Both consume the same pinned `f1tenth_gym_jax` and `jax_pf`
revisions, and both consume the *same compiled* `racing_common` — `fast-sim` mounts `dev`'s
`build/`/`install/` read-only rather than growing a second Python implementation of Frenet
conversion. There is one track model in this repository, in C++, with a pybind11 binding.

## Consequences

- Two images and two dependency managers to keep pinned. Accepted; ADR 0002 already made that call
  for `dev` vs `fast-sim` and this is the same seam.
- `sim/` cannot `import rclpy`, and nothing in `ros_ws/` may import from `sim/`. Both directions are
  intentional: it is what keeps `sim/` fast and `ros_ws/` free of CUDA.
- COMMON-1050 (pybind11 parity) is the canary. If the shared binding breaks, it surfaces in `sim/` as
  an import error far from its cause, and that test is what names it.
- The golden baseline is produced by the GPU pipeline and asserted against by the ROS pipeline. That
  cross-check is only meaningful because the physics really is shared — and it has already earned its
  keep: it is what exposed `racing_metrics` completing a lap at half distance.

## Why this is written down

This is the structural decision most likely to be "simplified" away by a future contributor — or
agent — who sees two directories doing simulation and consolidates them. Consolidating them
reintroduces the uv/ament fight and puts CUDA back into the ROS image, which
`docs/agents/repo-gotchas.md` #4 records as measurably worse.
