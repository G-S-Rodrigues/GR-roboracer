# ADR 0002 — Development environment: repo on ext4, underlay in the image, artifacts in named volumes

**Status:** accepted
**Date:** 2026-09-06
**Supersedes:** none
**Amended:** 2026-10-13 — §4, by issue #12 part 1 (a ROS-free `raceline/` uv project may run in `dev`).
**Amended:** 2026-10-25 — §4, by issue #47 part 3 (a `gpu` stage).

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

*Amendment (2026-10-13, issue #12 part 1).* A ROS-free uv project may run in `dev` when (a) it lives
outside `ros_ws/`, so `colcon build --base-paths ros_ws/src` never scans it; (b) it runs with
`env -u PYTHONPATH`, so it never shares an interpreter path with `rclpy` (RACELINE-1020, gotcha 24);
and (c) it needs no CUDA. `raceline/` (offline raceline generation) is the one such project. The
image is unchanged: "nothing here creates a venv" in the Dockerfile stays true. The cost is that its
venvs (`raceline/.venv`, `raceline/tum_env/.venv`) sit on the bind mount, root-owned, outside the
named volumes §3 uses for generated files.

*Amendment (2026-10-25, issue #47 part 3).* A `gpu` stage exists: **`dev` plus a pinned CUDA 12.8
toolkit subset (nvcc, cudart)**, for GPU ROS nodes, starting with FAR-LIO's CUDA build in part 5. It
derives from `dev` by owner ruling (2026-10-25) so the owner can develop, test and run the simulators
in it. Conditions: (a) `dev` stays CPU-only and never derives from `gpu`, and the gym's JAX stays on
CPU inside `gpu` too. It is inherited from `dev`, pinned by `docker/gym-cpu-overrides.txt`, and `gpu`
installs no Python package. The reason is gotcha 4's measurement: on the 64-beam scenario a GPU step
takes 8.36 ms against 1.62 ms on the CPU, close enough to the 10 ms period to drop `/odom`. So the GPU
serves nodes whose kernels are large enough to win, never the sim step. (b) `fast-sim` stays ROS-free
(ADR 0003 unchanged). (c) It is built and tested on the owner's machine, never in PR CI (no GPU
runner); CI holds its structure (CHECK-1900 to CHECK-1940). (d) Its apt installs are a full pinned
closure (`docker/gpu-apt-pins.txt`) with an install-time diff (gotcha 27). (e) It has its own named
volumes (§3, gotcha 21) and its own `ROS_DOMAIN_ID`, because it shares the compose network with `dev`.
The costs: `gpu` carries everything in `dev`, including its uv and the CPU gym, and adds CUDA; a CUDA
bug in a GPU node is caught by the owner's hand run (GPU-3010), not by CI; and a `dev` rebuild
invalidates `gpu`.

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

Anything wanting both ROS and CUDA now has a place, `gpu` (amendment 2026-10-25). ROS plus JAX is still
only the `racing_sim_gym_jax` CPU exception, unchanged in `gpu`.

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
- **A top-level pin does not pin a pip tool's tree either.** `pre-commit` pulls `virtualenv`, whose
  new floor `packaging>=26.3` collides with Debian's apt-owned `packaging 24.0`, and the dev image
  stopped building. The dev tools are installed `--no-deps` from a full pinned closure
  (`docker/dev-tools-requirements.txt`), followed by `pip3 check`.
- **Upstream's `[tool.uv] override-dependencies` does not apply to consumers.** `jax_pf` declares
  `jax[cuda12]>=0.6.1,<0.7`, unsatisfiable against the gym's `jax>=0.7.2,<0.8`; upstream resolves it
  with a pyproject override that only takes effect when uv operates *on that project*. Installing the
  gym as a dependency requires reproducing that override explicitly.
- **2026-10-28 (autodrive-sim part 7): the dev image carries the Socket.IO closure.** Our AutoDRIVE
  server and its replay client import python-socketio 4.2.0, python-engineio 3.13.0, gevent 26.9.0 and
  gevent-websocket 0.10.1 (the sim speaks Engine.IO v3; repo-gotchas #30); with their dependencies the
  closure is 8 pins (adding greenlet, zope-event, zope-interface, websocket-client), in
  `docker/dev-tools-requirements.txt` beside the dev tools and guarded by SIMAD-1150. The rebuild of
  the shared image follows the part's merge; until then its gate runs with the closure in a worktree
  `PYTHONPATH`.
