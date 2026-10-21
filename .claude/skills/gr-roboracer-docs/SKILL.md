---
name: gr-roboracer-docs
description: Keep GR-roboracer's user-facing docs true to the code on main. Use when a change adds or alters something a user can see or run — a launch argument, an implementation, a command, a status in the README capability table, a measured result — when a planned or exploratory idea is chosen for the roadmap, or when a demonstration video needs recording or retiring.
---

# gr-roboracer-docs

The docs follow the code: every sentence a user reads is **true on `main`** the moment the PR
that changed the code merges. A claim a plan only promises is not true yet.

## Two audiences, never mixed

| Audience | Files | Written for |
|---|---|---|
| **User** | `README.md`, `docs/*.md`, `docs/workflows/`, `docs/assets/` | someone on Linux with Docker who has never read the code |
| **Agent** | `CLAUDE.md`, `ros_ws/src/*/CLAUDE.md`, `docs/agents/`, `docs/adr/` | an agent working in the repo |

User docs assume Linux with Docker and link only to user docs and code. Host quirks (WSL2,
WSLg, the Windows shell), gotchas, test conventions and process rulings belong to agent docs
(`docs/agents/running-wsl.md` holds the WSL material). A fact both audiences need is written
once, in the user page, and the agent doc points at it.

## The owner page of each fact

Each fact lives on one **owner page**; every other page links there instead of restating it.

| Fact | Owner page |
|---|---|
| What exists and its status (Available / In development / Planned / Exploratory) | `README.md` capability table |
| The headline video | `README.md` `## Demonstration` |
| Installing and first run | `docs/setup.md`, `docs/quick-start.md` |
| Launch arguments, their values, the target YAML fields | `docs/configuration.md` |
| The command for each workflow, what the car drives on, what it saves | `docs/workflows/mapping.md`, `docs/workflows/racing.md` |
| Measured numbers, the mapping video | `docs/results.md` |
| Every earlier video | `docs/demonstrations.md` |
| What each test layer proves, the `check.sh` modes | `docs/verification.md` |
| Running a scenario by hand, metrics, goldens | `docs/running.md` |

A command that appears on two pages is byte-identical on both.

## The capability table

A row's status is a claim about `main`, backed as follows:

- **Available** — merged code a user can run or select today, documented on its workflow page
  with the exact command. An offline tool counts when it has a command.
- **In development** — work in flight on an open issue or branch.
- **Planned** — a written spec exists; nothing merged.
- **Exploratory** — the owner chose to investigate it; no spec. A brainstormed idea enters the
  table only once the owner chooses it; until then it stays in `.scratch/`.

Rows are added, never deleted (ADR 0005): a method that is beaten stays Available, with its
scope saying what beat it. A status moves only when `main` backs the new one.

## Steps

### 1. List what the change makes untrue

Read the branch diff (`git diff main...HEAD`) and, for each user-visible change, every page the
owner-page table routes it to. Write the list into the plan or PR: page, line, old claim, new
claim, the source line that backs it.

**Done when** every changed launch argument, implementation, command, default, status and
measured number in the diff has a row, or the list says "no user-visible change" and why.

### 2. Update the pages in the same PR as the code

Edit each listed page, owner page first, then the pages that link to it. Re-read each new
sentence against its backing source line; a number is copied from the file that produced it.

**Done when** every row from step 1 is applied and `docs_check.py` passes:

```bash
python3 -I .claude/skills/gr-roboracer-docs/scripts/docs_check.py .
```

It checks that links and anchors resolve, that no user page links an agent doc, and that no
user page mentions WSL or Windows. It does not check prose; step 3 does.

### 3. Decide whether a video is due

Record a new video only when **what a user would see changes**:

- the composition the README demonstration shows changes (a new default in
  `config/reference_stack.yaml`, or the demo's pose source, planner or controller), so the lap
  would look different; or
- an implementation becomes Available whose behaviour looks unlike anything already shown — a
  new localizer, planner path, mapper or map type.

A change that only makes the same picture faster or more accurate updates `docs/results.md`'s
numbers, not the video. Refactors, tests, tuning and docs never need one.

**Done when** the PR body says "video due: <which, why>" or "no video: <why>".

### 4. Record it as a follow-up PR

A due video is its own small PR after the code merges, so no code PR waits on recording. Follow
[VIDEO.md](VIDEO.md) — it covers recording, the speed label, the Drive upload and moving the
previous video into `docs/demonstrations.md`.

**Done when** the follow-up PR is merged, or the run's ledger names it as the next part.

## Two branches touching the same pages

Whichever PR merges second updates the docs: it merges `main`, re-checks every claim it touches
against the merged code, and fixes the pages in its own PR. Tell the other session which pages
yours changes before either merges.

## Merging a docs-only PR

A PR is **docs-only** when every file it changes is a `.md` file or under `docs/assets/`. It
needs neither `check.sh --ready` nor green CI (owner ruling, 2026-10-22): run `docs_check.py`,
then merge with `gh pr merge --merge` without waiting for CI. A PR that also touches code,
config, a test or a script — the scripts in this skill included — takes the full gate in
`CLAUDE.md`.
