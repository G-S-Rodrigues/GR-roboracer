# GR-roboracer

An autonomous racing stack for F1TENTH / RoboRacer on ROS 2 Jazzy, plus a headless JAX simulator.

@docs/agents/repo-gotchas.md

## The rule

**`./scripts/check.sh --full` must pass before any change is described as done.** Not "the tests I
touched" — the gate. It runs in the `dev` container:

```bash
docker exec gr-roboracer-dev bash -lc 'cd /ws && ./scripts/check.sh --full'
```

`--fast` (tiers 0–1, the pre-commit hook) and `--ci` (adds tier 2 and one seeded lap) exist for
faster loops, but neither of them is "done". `--full` is the gate; it is not the same as *every test
that exists*, and ADR 0005 says why — the expensive cross-product belongs to a nightly sweep, and a
gate nobody runs is worse than no gate.

## Algorithms are added, never replaced

This is a research stack. When a better method lands, **the one it beat stays** — deleting it
destroys the baseline the next comparison needs. Improvement work on an existing algorithm is
welcome; substitution is not. Selecting between implementations is a launch argument with a default,
never a code deletion.

The rule that keeps this affordable: **every implementation is verified once, against ground truth,
inside the reference stack current when it lands** — never against every peer, and never against its
predecessors. Combinations are not tested. Test count grows linearly with implementations, not
quadratically. ADR 0005 has the reasoning and what it gives up; the reference stack is whatever
`racing_bringup/launch/sim_pure_pursuit.launch.py` composes until `config/reference_stack.yaml`
lands with the SLAM phase.

## Where you are

Everything runs in containers. `docker compose -f docker/docker-compose.yaml up -d dev` starts the
long-lived `dev` container; `test -f /.dockerenv` tells you whether you are inside one. ROS nodes,
`colcon`, lint and every test tier run in `dev`. The headless GPU simulator runs in `fast-sim`, which
has no ROS in it at all — that split is deliberate and load-bearing (ADR 0002, ADR 0003).

## Read before you do

| Before... | Read |
|---|---|
| writing or moving any test | `docs/agents/testing.md` |
| touching `docker/`, or adding a dependency to an image | `docs/adr/0002-development-environment.md` |
| changing anything under `sim/`, or "unifying" it with `ros_ws/` | `docs/adr/0003-sim-outside-ros-workspace.md` |
| adding a `.robot` suite | `docs/adr/0004-robot-framework-at-tiers-3-4.md` |
| touching the gym action mapping | `docs/adr/0001-simulator-action-contract.md` |
| working in any `ros_ws/src/<package>/` | that package's own `CLAUDE.md` |
| adding a second implementation of anything, or wondering whether to delete the old one | `docs/adr/0005-algorithms-are-added-not-replaced.md` |
| touching message stamps, timers, `use_sim_time`, or anything that reads the clock | `docs/adr/0006-simulated-time-is-the-time-base.md` |
| regenerating `tests/golden/baseline.json` | `docs/agents/repo-gotchas.md` #14 — it is a reviewed diff, never a way to green a run |

## Git

Work goes on a branch off `main` and lands by PR. The one standing exception, authorized by the
owner: **the `1-stack-bootstrap` work may push directly to `main`**, because it is what created the
default branch. That authorization does not extend past it.

Host setup, the WSL2 quirks that bite from a Windows shell, and how to drive a scenario by hand are
in `docs/running.md`.

## Conventions

- **C++:** Google style, `IndentWidth: 4`, `ColumnLimit: 80`, clang-tidy
  `readability-identifier-naming`. Carried over from the Evo repos so both codebases match.
- **Python:** `ruff`, 80 columns, double quotes.
- **Per-package `CLAUDE.md` is capped at ~20 lines.** Purpose, public contract, language and why, the
  test command, the one trap. Boilerplate there is worse than nothing: it gets loaded into every
  session in that directory and then ignored, which teaches the reader to ignore all of them.

## Repository layout

```
ros_ws/src/     ROS 2 packages. Each is a thin shell over a ROS-free core.
sim/            Headless GPU rollout + golden generation. uv project, no ROS. (ADR 0003)
config/         Tracks, scenarios, and one versioned vehicle parameter file.
tests/          Tier 3 (pytest), tier 4 (Robot Framework), shared keywords, golden baseline.
scripts/        check.sh (the gate), run_scenario.py, compare_metrics.py.
docker/         Multi-stage image: base → underlay → dev / fast-sim / runtime.
docs/           ADRs, agent-facing notes, research.
.scratch/       Grills, specs and plans. Gitignored; `git clean -e .scratch`.
```
