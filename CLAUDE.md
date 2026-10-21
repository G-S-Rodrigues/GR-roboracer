# GR-roboracer

An autonomous racing stack for F1TENTH / RoboRacer on ROS 2 Jazzy, plus a headless JAX simulator.

@docs/agents/repo-gotchas.md

## The rule

**`./scripts/check.sh --ready` must pass once at the end of an implementation (and again after a
failure), and CI must be green on the PR head, before any change is described as done.** Not "the tests
I touched" — the gate. It runs in the `dev` container:

```bash
docker exec gr-roboracer-dev bash -lc 'cd /ws && ./scripts/check.sh --ready'
```

`--ready` is `--fast`'s scoped tiers 0–2, the seeded lap, and the tier-3/4 tests that
`config/reference_stack.yaml` selects for the implementations the branch touched; whatever the map
cannot place runs the whole tree (ADR 0007). `--full` is no longer in the development loop: the nightly
runs it, and ADR 0007 says why. `--fast` (tiers 0–1, the pre-commit hook) and `--ci` (the PR job: adds
tier 2 and one seeded lap) are faster loops. A gate nobody runs is worse than no gate (ADR 0005, ADR
0007).

**Nor is a change done while a user-facing doc it made untrue is unfixed.** The docs change in the
same PR as the code; the `gr-roboracer-docs` skill says which pages and when a video is due.

**A red nightly does not block development.** The nightly is watched, not waited on: a PR merges on
its ready gate and green PR CI, never on a nightly run, and a red nightly halts no branch in flight. A
fix can always land afterwards. The fix for a nightly failure is carried into the next plan written,
as one of its steps or as a fix part at the head of the queue.

## Algorithms are added, never replaced

This is a research stack. When a better method lands, **the one it beat stays** — deleting it
destroys the baseline the next comparison needs. Improvement work on an existing algorithm is
welcome; substitution is not. Selecting between implementations is a launch argument with a default,
never a code deletion.

The rule that keeps this affordable: **every implementation is verified once, against ground truth,
inside the reference stack current when it lands** — never against every peer, and never against its
predecessors. Combinations are not tested. Test count grows linearly with implementations, not
quadratically. ADR 0005 has the reasoning and what it gives up; the reference stack is named in
`config/reference_stack.yaml` and composed by `racing_bringup/launch/sim_pure_pursuit.launch.py`
(BRINGUP-1030 fails if they disagree). Changing what it names is a reviewed decision.

## Built for the real car

Today's track is a known simulator map; next is the competition's unseen map, then our own car.
Design every implementation for that last case: a map built by SLAM with no ground-truth centerline,
walls that may be open in places, a track that may pass over itself, and walls that move — a car hits
a barrier and whoever resets it does not put it back exactly where it was, so the map is a snapshot
the track drifts from. Each spec states how its feature will be validated without ground truth; that
validation is built once a mapped-map workflow exists (AutoDRIVE), not before.

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
| changing `check.sh --ready`'s selection, or a `paths` / `verified_by` in `config/reference_stack.yaml` | `docs/adr/0007-ready-is-a-scoped-local-run.md` |
| changing anything a user can see or run (a launch argument, an implementation, a command, a capability status, a result) | the `gr-roboracer-docs` skill (`.claude/skills/gr-roboracer-docs/`) |
| regenerating `tests/golden/baseline.json` | `docs/agents/repo-gotchas.md` #14 — it is a reviewed diff, never a way to green a run |

## Git

Work goes on a branch off `main` and lands by PR. PRs land as merge commits
(`gh pr merge --merge`). The one standing exception, authorized by the owner: **the
`1-stack-bootstrap` work may push directly to `main`**, because it is what created the default
branch. That authorization does not extend past it.

Host setup and the first run are in `docs/setup.md` and `docs/quick-start.md`, driving a scenario by
hand is in `docs/running.md`, and the WSL2 quirks (WSLg display, Windows shell) are in
`docs/agents/running-wsl.md`.

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
raceline/       Offline raceline generation (TUM optimizer). uv project, no ROS, runs in dev.
scripts/        check.sh (the gate), run_scenario.py, compare_metrics.py.
docker/         Multi-stage image: base → underlay → dev / fast-sim / runtime.
docs/           ADRs, agent-facing notes, research.
.scratch/       Grills, specs and plans. Gitignored; `git clean -e .scratch`.
```
