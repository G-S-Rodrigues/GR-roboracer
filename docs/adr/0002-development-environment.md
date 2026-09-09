# ADR 0002 — Development environment: repo on ext4, underlay in the image, artifacts in named volumes

**Status:** accepted
**Date:** 2026-09-06
**Supersedes:** none

## Context

The stack is developed on Windows 11 with WSL2 and Docker Desktop, targeting ROS 2 Jazzy, and must
also run a CUDA JAX simulator. Three questions had to be answered together, because the wrong answer
to any one of them degrades the others:

1. Where does the working tree live?
2. Where do third-party ROS sources get built?
3. Where do `build/`, `install/` and `log/` go?

The binding constraint is iteration speed. A guardrail that takes four minutes is run after three
changes have been layered on a broken one, so build latency is a correctness property here, not a
convenience.

## Decision

**1. The tree lives on ext4 inside the WSL2 distro** (`~/gitroot/GR-roboracer`), never on `/mnt/c/`.

**2. Third-party ROS packages are built into `/opt/racing_underlay` at image-build time**; only our
packages build in `/ws` at runtime.

**3. `/ws/build`, `/ws/install` and `/ws/log` are Docker named volumes**, not bind-mounted paths.

**4. `uv` and `colcon` live in separate images.** `dev` (ROS, no JAX) and `fast-sim` (JAX/CUDA, no
ROS) do not overlap.

## Consequences

Underlay-in-image makes the image digest a genuine reproducibility record, which is what lets spec
item 13 put that digest in every metrics record. A `colcon build` after an edit takes seconds and
never rebuilds `slam_toolbox`.

Named volumes keep tens of thousands of generated files out of the editor's indexer and stop `dev`
and `fast-sim` clobbering each other's artifacts. The cost is that the artifacts are not directly
visible from Windows, which is the intent.

The `dev`/`fast-sim` split means a change to the pinned gym SHA rebuilds only `fast-sim`, and that
`sim/` cannot accidentally acquire a ROS import — the interpreter it runs under has no ROS on it.

The cost of the split is that anything wanting both ROS and JAX in one process has nowhere to run.
That is deliberate: `racing_sim_gym_jax` is the single exception, and it runs in `dev` against a
CPU-resident env, which is why Step 9 carries a warm-up requirement rather than a GPU one.

## Alternatives rejected

**Tree on `/mnt/c/`, edited natively from Windows.** Rejected: through the 9p bridge `colcon build`
is roughly an order of magnitude slower, and `inotify` does not propagate, so file watchers stop
firing *with no error*. The symptom is "my rebuild didn't pick up the change" — a silent failure,
which is the worst kind. Verify with `df -T .`; it must read `ext4`.

**Cloning third-party sources into the workspace at container start.** Iterates faster on a
third-party change and needs no image rebuild, but makes "which code produced this metric"
unanswerable, because the sources are not in the image digest. Rejected on reproducibility.

**`docker-ce` inside Ubuntu instead of Docker Desktop.** Leaner, but NVIDIA container-toolkit
plumbing then becomes manual, and the GPU is load-bearing for `fast-sim`. Rejected on setup cost.

**One image containing both `uv` and `colcon`.** Mixing a uv-managed venv with ament's Python
discovery is a known fight, and losing it produces import errors far from their cause.

## Notes earned while implementing this

These are the corrections the plan did not anticipate; each is also recorded as a gotcha.

- **`f1tenth_system` is a submodule repository, and `vcs import` does not recurse submodules.** Its
  submodules must be listed as siblings in `third_party.repos`, or the import silently yields empty
  directories that `colcon` then ignores.
- **Its recorded submodule pins are Foxy-era and do not compile on Jazzy.** `f1tenth/vesc@foxy` uses
  the single-argument `declare_parameter(name)` overload, removed after Foxy. `f1tenth/vesc@humble`
  builds clean and is what we pin.
- **`rosdep` does not resolve `asio_cmake_module`**, a transitive export dependency of
  `serial_driver`. `vesc_driver` fails to configure without an explicit apt install.
- **Pinning the gym SHA does not pin the dependency tree.** `f1tenth_gym_jax` declares `jax-pf` with
  no rev, and `jax_pf` supplies the ray marching behind `/scan`.
- **Upstream's `[tool.uv] override-dependencies` does not apply to consumers.** `jax_pf` declares
  `jax[cuda12]>=0.6.1,<0.7`, unsatisfiable against the gym's `jax>=0.7.2,<0.8`; upstream resolves it
  with a pyproject override that only takes effect when uv operates *on that project*. Installing the
  gym as a dependency requires reproducing that override explicitly.
