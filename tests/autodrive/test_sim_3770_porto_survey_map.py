"""SIM-3770: slam_toolbox maps Porto on the real AutoDRIVE sim, one lap on
ground truth, and the saved grid is scored against the official map.

LIVE (needs the AutoDRIVE sim; fails, never skips, without one). Run it with
`scripts/autodrive_qualify.sh --domain <60-69> --case survey --sim windows`.
The launch is slam_toolbox_mapping.launch.py on `simulator:=autodrive`: the
reference stack drives the centerline on ground truth while the mapper lays
/scan at those poses. When the lap ends `map_saver_cli` saves /map to
`$AUTODRIVE_OUT/Porto.{png,yaml}` (never under third_party/; the grid is
committed to the data repo by hand, a reviewed diff). Its arguments are
scripts/map_track.sh's, except `use_sim_time`: AutoDRIVE has no /clock, so it
comes from `sim_time_for("autodrive")` (false). The map's wall error against
porto.yaml's official map (`map_to_wall`) is REPORTED in survey.json, never
bounded. The gates: no latch, the spawn, one lap, no collision, a saved grid.
"""

import dataclasses
import json
import math
import os
import subprocess
from pathlib import Path

from _porto_drive import SPAWN_MAP, SPAWN_TOL_M, SPAWN_TOL_RAD, drive_porto
from racing_bringup.simulators import sim_time_for
from racing_evaluation.map_accuracy import (
    load_grid,
    map_to_wall,
    scenario_truth_grid,
)
from racing_test_keywords.autodrive_live import judge_stale_input

REPO = Path(__file__).resolve().parents[2]
LAPS = 1
BUDGET_S = 600.0
SAVE_TIMEOUT_S = 120.0
# Q5 (b0-d): the existing survey config unless the odometry drift says
# otherwise.
SLAM_TOOLBOX_CONFIG = "config/localization/slam_toolbox_survey.yaml"
LAUNCH = [
    "ros2",
    "launch",
    "racing_bringup",
    "slam_toolbox_mapping.launch.py",
    "simulator:=autodrive",
    "vehicle:=config/vehicles/autodrive_roboracer.yaml",
    "scenario:=config/scenarios/porto.yaml",
    "use_rviz:=false",
    f"laps:={LAPS}",
    "autodrive_port:=4567",
    f"slam_toolbox_config:={SLAM_TOOLBOX_CONFIG}",
]
HOST_COMMAND = (
    "scripts/autodrive_qualify.sh --domain <60-69> --case survey --sim windows"
)
END_LATCH_WAIT_S = 1.0


def save_map(stem: Path) -> None:
    """scripts/map_track.sh's map_saver_cli call, `use_sim_time` aside."""
    use_sim_time = str(sim_time_for("autodrive")).lower()
    subprocess.run(
        [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            str(stem),
            "--fmt",
            "png",
            "--occ",
            "0.65",
            "--free",
            "0.196",
            "--ros-args",
            "-p",
            f"use_sim_time:={use_sim_time}",
            "-p",
            "save_map_timeout:=60.0",
            "-p",
            "map_subscribe_transient_local:=true",
        ],
        cwd=REPO,
        check=True,
        timeout=SAVE_TIMEOUT_S,
    )


def test_sim_3770_porto_survey_map(tmp_path: Path) -> None:
    out = Path(os.environ.get("AUTODRIVE_OUT", tmp_path))
    out.mkdir(parents=True, exist_ok=True)
    stem = out / "Porto"
    run = drive_porto(
        LAUNCH,
        LAPS,
        out,
        budget_s=BUDGET_S,
        host_command=HOST_COMMAND,
        node_name="sim_3770_driver",
        after_laps=lambda: save_map(stem),
    )
    m = run.metrics
    accuracy = None
    if stem.with_suffix(".yaml").exists():
        accuracy = dataclasses.asdict(
            map_to_wall(
                load_grid(stem.with_suffix(".yaml")),
                scenario_truth_grid(REPO / "config/scenarios/porto.yaml"),
            )
        )
    result = {
        "slam_toolbox_config": SLAM_TOOLBOX_CONFIG,
        "laps_stack_s": list(m.lap_times) if m else [],
        "laps_sim_s": [i[1] for i in run.increments],
        "collisions_stack": int(m.collision_count) if m else None,
        "collisions_sim": run.sim_collisions,
        "stale_input": run.stale,
        "safety_reasons": dict(run.reasons),
        "host_load": run.load,
        "wall_s": run.wall_s,
        "map_to_wall": accuracy,
        "grid": str(stem),
    }
    (out / "survey.json").write_text(json.dumps(result, indent=2))
    print("SURVEY", json.dumps(result))

    verdict = judge_stale_input(
        run.latch, run.stamps, 0.1, run.stale, run.trace, END_LATCH_WAIT_S
    )
    assert run.stale == 0, f"STALE_INPUT x{run.stale}: {verdict}"
    assert run.reasons.get("REASON_ESTOP_LATCHED", 0) == 0, (
        f"emergency stop latched: {dict(run.reasons)}"
    )
    assert run.spawn_error is not None, "no odom frame after the reset"
    assert (
        run.spawn_error[0] <= SPAWN_TOL_M
        and run.spawn_error[1] <= SPAWN_TOL_RAD
    ), f"first pose {run.first_pose} vs {SPAWN_MAP}: {run.spawn_error}"
    assert m is not None, "metrics never published"
    assert run.last_count >= LAPS, f"sim lap_count {run.last_count} < {LAPS}"
    assert m.laps_completed == LAPS, m.laps_completed
    assert all(math.isfinite(t) for t in m.lap_times)
    assert m.collision_count == 0, f"stack collisions {m.collision_count}"
    assert run.sim_collisions == 0, f"sim collisions {run.sim_collisions}"
    assert accuracy is not None, "map_saver wrote no Porto.yaml"
    assert accuracy["wall_cell_count"] > 0, accuracy
