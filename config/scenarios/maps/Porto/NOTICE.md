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
  --map config/scenarios/maps/Porto/Porto.yaml --start-pose 0 0 0 --direction ccw \
  --walls observed --name Porto --out config/tracks/porto.yaml
```

**Assumption (not measured):** AutoDRIVE spawns at the map origin with heading +x, so the racing
direction is `ccw` and `s = 0` is the loop point nearest (0, 0). The map has no direction marker.
#44 part 10 checks this against AutoDRIVE's lap counter; if it is wrong, regenerate with
`--start-pose 0 0 3.14159 --direction cw`.

The gym cannot run this directory: there is no `Porto.png` and there are no CSVs, and the gym
importer's `Porto.yaml` would collide with the official one.

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
