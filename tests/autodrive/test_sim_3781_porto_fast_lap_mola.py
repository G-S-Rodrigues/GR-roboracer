"""SIM-3781: SIM-3750's fast min-time lap on Porto driven on MOLA's estimate
over the MOLA map (`tracks/Porto/maps/Porto/mola/`, read through porto.yaml's
`map_directory`).

LIVE, like SIM-3750 (needs the AutoDRIVE sim; fails, never skips). Run it with
`scripts/autodrive_qualify.sh --domain <60-69> --case fast_lap_mola --sim
windows`. Without the MOLA map in the data pin the launch fails in
`mola_map_files` naming `scripts/mola_map_track.sh`. The gates are SIM-3750's;
lap times and the live localization error are reported, never bounded.
"""

from pathlib import Path

from _porto_estimator import fast_lap_on_estimator

HOST_COMMAND = (
    "scripts/autodrive_qualify.sh --domain <60-69> --case fast_lap_mola"
    " --sim windows"
)


def test_sim_3781_porto_fast_lap_mola(tmp_path: Path) -> None:
    fast_lap_on_estimator("mola", "fast_lap_mola", HOST_COMMAND, tmp_path)
