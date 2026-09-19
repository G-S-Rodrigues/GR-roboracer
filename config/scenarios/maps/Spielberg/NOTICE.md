# Spielberg — provenance

Derived from `f1tenth/f1tenth_racetracks` at
`866fe657f0bbe245f90a5042bc6eec0b0489d4c3` (`Spielberg/Spielberg_centerline.csv`,
`Spielberg/Spielberg_raceline.csv`), licensed **GPL-3.0**.

`config/tracks/spielberg.yaml` is produced by `tools/import_f1tenth_racetrack.py`; every file in
this directory is then generated from that track file by `sim/track_importer.py`, the same code
`sim/rollout.py` runs before a headless golden. The upstream `Spielberg_map.png` is deliberately
**not** used: the sim, the golden generator and the supervisor's track limits must all see one
world, and that world is the one the canonical centerline and widths describe.

Regenerate with:

```bash
python3 -m sim.track_importer config/tracks/spielberg.yaml config/scenarios/maps
```
