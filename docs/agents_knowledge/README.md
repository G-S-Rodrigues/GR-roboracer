# Agents knowledge

What this repo has learned, one file per concern: what is built, what was measured, what was tried
and dropped, ideas not yet specified, and open questions. Read the files a piece of work touches
before grilling, specifying or planning it, so the work starts from what is known rather than
re-measuring it or retrying a dead end.

| Concern | Read before work that touches |
|---|---|
| [architecture](architecture.md) | the stack's shape and direction, the simulator adapter contract, `config/reference_stack.yaml`, the data repo, gates and benchmarks, test tiers, the ready gate, the nightly, the heavy lane, the dev environment |
| [simulation](simulation.md) | f1tenth gym_jax, the AutoDRIVE simulator and its bridges, the league car's vehicle file and limits, the vehicle model, sim-to-sim |
| [localization-slam](localization-slam.md) | `pose_source`, mapping, a map in the data repo, `/scan` `/odom` `/imu` as estimator inputs, the FAR-LIO fork |
| [raceline](raceline.md) | track extraction, the TUM optimizers and the fork, vehicle limits and GGV, committed racelines, the verifier |
| [control](control.md) | Pure Pursuit, the safety supervisor, speed loops, a new controller, the start at t=0 |
| [reinforcement-learning](reinforcement-learning.md) | a learned component, a training loop, batched gym rollouts |
| [competition](competition.md) | the AutoDRIVE league and RoboRacer events, their rules, the submission, our target |

Traps any task can hit are not here: they are in [`docs/agents/repo-gotchas.md`](../agents/repo-gotchas.md).
Decisions with alternatives are ADRs in [`docs/adr/`](../adr/). User-facing docs stay in `docs/`; these
files link to them rather than restate them.

## Each file's shape

1. **Current state** — what is on `main`, each item linked to its code, ADR or PR.
2. **What we measured** — dated results with their numbers and source (a PR, commit, issue or file on
   `main`).
3. **Dead ends** — what was tried, why it failed.
4. **Ideas not yet specified** — wanted, not specced, with the owner's dated words where given. An idea
   that gets an issue becomes a link to it.
5. **Open questions** — unknowns that shape future work.

## Keeping it current

A feature's close-out harvest writes here: before the PR that closes a feature's issue, each open topic
and finding in its scratch folder becomes a gotcha, a line in one of these files, an issue, or is
dropped as resolved (`where-documents-go.md`, "Closing out a feature", in GR-ai-agents). Cite PRs,
commits and files on `main`, never a scratch path: scratch folders are deleted once harvested.
