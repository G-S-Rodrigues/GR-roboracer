# ADR 0009 — Track data lives in GR-roboracer-data

**Status:** accepted
**Date:** 2026-11-02
**Amends:** nothing. ADR 0007's whole-tree rule is applied to one more path.

## Context

The committed Spielberg and Porto maps and the Spielberg racelines were 28 files, 2.6 MB of binaries
and generated records, in a code repository (owner ruling OQ-R9, 2026-10-24). The TUM fork already
keeps its fixtures in `G-S-Rodrigues/GR-roboracer-data`, and the owner ruled that tracks, SLAM maps and
racelines live there too, pinned by SHA as a `third_party/` submodule.

## Decision

- **Layout of record:** `tracks/<Map>/maps/<Map>/…` and `tracks/<Map>/racelines/<vehicle-version>/…`,
  where `<Map>` is the scenario's `environment.map` with its case kept (`Spielberg`, `Porto`). `<Map>`
  repeats under `maps/` because the gym finds a map as `<F1TENTH_GYM_JAX_MAP_DIR>/<map>/<map>.yaml` and
  bringup builds the same path from `map_directory`. A scenario's `map_directory` is therefore
  `…/tracks/<Map>/maps`.
- **Raceline lookup** derives the folder from the scenario's map name, not from the track stem. A
  record's `track:` field stays the lower-case track-file stem; only the folder is keyed by the map.
- **`raceline --out-dir` is the tracks root.** The default is `third_party/GR-roboracer-data/tracks`; a
  record goes to `<out-dir>/<Map>/racelines/<vv>/`, `<Map>` being the `--map` file's stem, case kept.
  `--track` writes `tracks/<stem>/` (the lower-case track-file stem) while the lookup reads the map name,
  so generate a line for the launch to find with `--map` (open topic 2026-11-02-d1-track-flag-lookup).
- **Pin by SHA, no tags (E2).** Data lands by PR into the data repo's `main` before GR-roboracer pins it
  (E4), so CI can always fetch the pinned commit.
- **analytic_circle and `config/tracks/` stay in-tree (E3).** Tier 1, metrics and the supervisor read
  the ground-truth tracks, and a clone without the submodule still runs the seeded lap (contract_test).
  Every test that reads real-track data fails closed, naming the missing path and the init command
  (SIMJAX-1600, BRINGUP-1970).
- **A bump is whole tree in both scopes (E6).** A gitlink change shows no file diff, so `--fast` and
  `--ready` treat `third_party/GR-roboracer-data` and `.gitmodules` as whole-tree triggers (CHECK-2350,
  2360, 2320). Per-file placement is deferred; it fails closed (ADR 0007).
- **The gym's bundled-map precedence is not decided here.** The gym searches its bundled maps before
  `F1TENTH_GYM_JAX_MAP_DIR`, so its Spielberg is upstream's. Part G1 fixes that.

## Consequences

- Every checkout and worktree needs `git submodule update --init third_party/GR-roboracer-data`, run on
  the host (repo-gotchas #44). CI initialises it (CHECK-2340).
- A new or re-surveyed map, or a regenerated raceline, is two PRs: one into the data repo, then a
  gitlink bump here. A record's `code:` blobs pin the raceline code, so a change to `CODE_FILES`
  regenerates the records in the data repo first.
- A data-only bump costs a whole-tree `--fast` and `--ready`.
