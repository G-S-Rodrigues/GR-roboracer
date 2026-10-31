# Porto - provenance

The official AutoDRIVE map, copied byte-for-byte (sha256 below, pinned by RACELINE-1180) from
`AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Racetracks` at
`cf3cf1e5d37489f4768c8041706e53977e4d6bc7` (2026-09-14), directory
`Legacy Tracks/Porto Track/`, licensed **BSD-2-Clause**.

| File | sha256 |
|---|---|
| `Porto.pgm` | `404a72551715fbf48ebcbd5b3b4093fcdf62934fb815898b144a017fe2c02efe` |
| `Porto.yaml` | `b78d687bae1758fdd62070b6da30c77a817bb102926630d973503035584d4d0d` |
| upstream `LICENSE` (below) | `928db06fa55c94014ac8cc807b7d4ac884ca96ca2977f0f8dc49d7e551f158bb` |

`config/tracks/porto.yaml` is produced from this map, unedited, by:

```bash
env -u PYTHONPATH uv run --project raceline --locked python -m raceline.track_from_map \
  --map config/scenarios/maps/Porto/Porto.yaml --start-pose 0 0 3.14159 --direction cw \
  --walls observed --name Porto --out config/tracks/porto.yaml \
  --source-note "$(sed -n 2p config/tracks/porto.yaml | cut -c3-)"
```

(`--source-note` is the header's second line; the header records the whole command.)

**Direction:** clockwise, from the sim's own data (#44 part 10, 2026-10-31): the 21 checkpoints,
the spawn heading (+x at the mapped spawn, where a counter-clockwise track runs against the car) and the
recorded drive. So `porto.yaml` is extracted `--direction cw` from `--start-pose 0 0 3.14159`
(`s = 0` near the map origin, which need not be the sim's lap line: racing_metrics counts from its first
sample). A first live drive of the stack along it ran 21.5 laps of path in 300 s with no stale-input
latch; the sim's own lap counter confirms it in part 10 Step 7.

The gym cannot run this directory as it is: its track loader raises "At least centerline file is
expected" because there is no `Porto_centerline.csv` (f1tenth_gym_jax `track.py`, the
`<stem>_centerline.csv` branch), and the gym importer's `Porto.yaml` would collide with the official one.

## Licence

<!-- LICENSE begin -->
BSD 2-Clause License

Copyright (c) 2025, AutoDRIVE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
<!-- LICENSE end -->
