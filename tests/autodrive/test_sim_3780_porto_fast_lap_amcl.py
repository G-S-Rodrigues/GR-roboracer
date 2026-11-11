"""SIM-3780: SIM-3750's fast min-time lap on Porto driven on amcl's estimate
over the survey map (`tracks/Porto/maps/slam_toolbox/res_0050/`, read
through porto.yaml's `map_directory`).

LIVE, like SIM-3750 (needs the AutoDRIVE sim; fails, never skips). Run it with
`scripts/autodrive_qualify.sh --domain <60-69> --case fast_lap_amcl --sim
windows`. Without the survey map in the data pin amcl never publishes and the
estimator wait in `drive_porto` times out naming it. The gates are SIM-3750's;
lap times and the live localization error are reported, never bounded.
"""

from pathlib import Path

from _porto_estimator import fast_lap_on_estimator

HOST_COMMAND = (
    "scripts/autodrive_qualify.sh --domain <60-69> --case fast_lap_amcl"
    " --sim windows"
)


def test_sim_3780_porto_fast_lap_amcl(tmp_path: Path) -> None:
    fast_lap_on_estimator("amcl", "fast_lap_amcl", HOST_COMMAND, tmp_path)
