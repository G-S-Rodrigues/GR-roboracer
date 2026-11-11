"""SIM-3771: MOLA-LO maps Porto on the real AutoDRIVE sim, one lap on ground
truth, and `mola_map_track.sh --from-simplemap` builds the map from it.

LIVE (needs the AutoDRIVE sim; fails, never skips, without one). Run it with
`scripts/autodrive_qualify.sh --domain <60-69> --case mola_map --sim
windows`. The reference stack drives one centerline lap on ground truth
(recording_path -> `$AUTODRIVE_OUT/recording.jsonl`) while
mola_mapping.launch.py `simulator:=autodrive` maps beside it. MOLA starts
before the t = 0 reset, while the car is parked at the spawn; its alignment T
is the ground-truth pose at the first recorded frame (AutoDRIVE has no held
start, unlike the gym's stamp 0). At the lap's end SIGINT reaches the MOLA
session, which saves the simplemap; the script then writes
`$AUTODRIVE_OUT/Porto.{mm,yaml}` and `Porto_grid.{png,yaml}` (never under
third_party/). The map's wall p95 against the official map is REPORTED in
mola_map.json, never bounded. The gates: no latch, the spawn, one lap, no
collision, a simplemap and a map written.
"""

import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

from _porto_drive import SPAWN_MAP, SPAWN_TOL_M, SPAWN_TOL_RAD, drive_porto
from racing_evaluation.map_accuracy import (
    load_grid,
    map_to_wall,
    scenario_truth_grid,
)
from racing_test_keywords.autodrive_live import judge_stale_input

REPO = Path(__file__).resolve().parents[2]
LAPS = 1
BUDGET_S = 600.0
MOLA_READY_S = 180.0
SAVE_TIMEOUT_S = 900.0
SEED = 42
LAUNCH = [
    "ros2",
    "launch",
    "racing_bringup",
    "sim_pure_pursuit.launch.py",
    "simulator:=autodrive",
    "vehicle:=config/vehicles/autodrive_roboracer.yaml",
    "scenario:=config/scenarios/porto.yaml",
    "use_rviz:=false",
    f"laps:={LAPS}",
    "autodrive_port:=4567",
]
HOST_COMMAND = (
    "scripts/autodrive_qualify.sh --domain <60-69> --case mola_map --sim"
    " windows"
)
END_LATCH_WAIT_S = 1.0


def test_sim_3771_porto_mola_map(tmp_path: Path) -> None:
    out = Path(os.environ.get("AUTODRIVE_OUT", tmp_path))
    out.mkdir(parents=True, exist_ok=True)
    recording = out / "recording.jsonl"
    simplemap = out / "mola_mapping.simplemap"
    stem = out / "Porto"
    simplemap.unlink(missing_ok=True)
    mola_log = out / "mola_mapping.log"
    mola: list[subprocess.Popen] = []

    def start_mola():
        """Start MOLA on the parked car and return the readiness predicate
        `drive_porto` polls while it spins the node (A6: never a blocking
        wait here, the hold must be observed)."""
        with mola_log.open("w") as log:
            mola.append(
                subprocess.Popen(
                    [
                        "ros2",
                        "launch",
                        "racing_bringup",
                        "mola_mapping.launch.py",
                        f"simplemap_output:={simplemap}",
                        "simulator:=autodrive",
                    ],
                    cwd=REPO,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        end = time.monotonic() + MOLA_READY_S

        def ready() -> bool:
            if "Initial re-localization done" in mola_log.read_text():
                return True
            if time.monotonic() > end:
                raise AssertionError(
                    "MOLA never logged its initial re-localization"
                    f" ({mola_log})"
                )
            return False

        return ready

    def save_simplemap() -> None:
        # SIGINT to the whole mapping session reaches mola-cli itself, which
        # saves the simplemap on that signal (mola_map_track.sh says why).
        os.killpg(mola[0].pid, signal.SIGINT)
        try:
            mola[0].wait(timeout=SAVE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            os.killpg(mola[0].pid, signal.SIGKILL)

    try:
        run = drive_porto(
            LAUNCH,
            LAPS,
            out,
            budget_s=BUDGET_S,
            host_command=HOST_COMMAND,
            node_name="sim_3771_driver",
            recording_path=recording,
            held_start=start_mola,
            after_laps=save_simplemap,
        )
    finally:
        if mola and mola[0].poll() is None:
            os.killpg(mola[0].pid, signal.SIGKILL)
    m = run.metrics
    built = None
    accuracy = None
    if simplemap.exists() and simplemap.stat().st_size > 0:
        built = subprocess.run(
            [
                "scripts/mola_map_track.sh",
                "--from-simplemap",
                str(simplemap),
                "config/scenarios/porto.yaml",
                str(stem),
                str(SEED),
            ],
            cwd=REPO,
            env={
                **os.environ,
                "MOLA_RECORDING": str(recording),
            },
            capture_output=True,
            text=True,
            timeout=SAVE_TIMEOUT_S,
        )
        grid_yaml = Path(f"{stem}_grid.yaml")
        if built.returncode == 0 and grid_yaml.exists():
            accuracy = map_to_wall(
                load_grid(grid_yaml),
                scenario_truth_grid(REPO / "config/scenarios/porto.yaml"),
            ).__dict__
    result = {
        "laps_stack_s": list(m.lap_times) if m else [],
        "laps_sim_s": [i[1] for i in run.increments],
        "collisions_stack": int(m.collision_count) if m else None,
        "collisions_sim": run.sim_collisions,
        "stale_input": run.stale,
        "safety_reasons": dict(run.reasons),
        "host_load": run.load,
        "wall_s": run.wall_s,
        "simplemap_bytes": simplemap.stat().st_size
        if simplemap.exists()
        else 0,
        "map_script_exit": built.returncode if built else None,
        "map_script_tail": built.stderr[-600:] if built else None,
        "grid_map_to_wall": accuracy,
        "stem": str(stem),
    }
    (out / "mola_map.json").write_text(json.dumps(result, indent=2))
    print("MOLA_MAP", json.dumps(result))

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
    assert result["simplemap_bytes"] > 0, "MOLA wrote no simplemap"
    assert built is not None and built.returncode == 0, result[
        "map_script_tail"
    ]
    assert accuracy is not None, "no grid written"
