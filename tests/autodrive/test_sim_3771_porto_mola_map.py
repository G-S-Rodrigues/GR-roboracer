"""SIM-3771: MOLA-LO maps Porto on the real AutoDRIVE sim, one lap on ground
truth, and `mola_map_track.sh --from-simplemap` builds the map from it.

LIVE (needs the AutoDRIVE sim; fails, never skips, without one). Run it with
`scripts/autodrive_qualify.sh --domain <60-69> --case mola_map --sim
windows`. The reference stack drives one centerline lap on ground truth
(recording_path -> `$AUTODRIVE_OUT/recording.jsonl`) while
mola_mapping.launch.py `simulator:=autodrive` maps beside it. The stack
drives from launch (AutoDRIVE has no `start_held`), so `drive_porto` holds the
car: the supervisor's `~/emergency_stop`, the reset to the spawn, MOLA started
on the parked car, a wait for its initial re-localization, then
`~/clear_emergency_stop`. The alignment T is the ground-truth pose at MOLA's
first keyframe stamp, which `mola_map_track.sh` reads from the simplemap. The
sim's lap time (`laps_sim_s`) includes the hold (`hold_s`, reported). At the
lap's end SIGINT reaches the MOLA session, which saves the simplemap; the
script then writes `$AUTODRIVE_OUT/Porto.{mm,yaml}` and `Porto_grid.{png,yaml}`
(never under third_party/). The map's wall p95 against the official map is
REPORTED in mola_map.json, never bounded. The gates: a clean hold, no latch,
the spawn, one lap, no collision, a simplemap and a map written, T at the
spawn and joined to the first keyframe, and no teleport in the keyframes.
"""

import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import yaml
from _porto_drive import (
    SPAWN_MAP,
    SPAWN_TOL_M,
    SPAWN_TOL_RAD,
    _wrap,
    drive_porto,
)
from racing_evaluation.map_accuracy import (
    load_grid,
    map_to_wall,
    scenario_truth_grid,
)
from racing_test_keywords.autodrive_live import (
    judge_stale_input,
    stale_input_timeout_s,
)

from scripts.compare_localization import poses, read_recording

REPO = Path(__file__).resolve().parents[2]
LAPS = 1
BUDGET_S = 600.0
MOLA_READY_S = 180.0
SAVE_TIMEOUT_S = 900.0
SEED = 42
VEHICLE = "config/vehicles/autodrive_roboracer.yaml"
LAUNCH = [
    "ros2",
    "launch",
    "racing_bringup",
    "sim_pure_pursuit.launch.py",
    "simulator:=autodrive",
    f"vehicle:={VEHICLE}",
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
# A teleport is about 8 m; keyframes are 0.2-0.3 m apart.
MAX_KEYFRAME_STEP_M = 1.0


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
        "hold_s": run.hold_s,
        "hold_error": run.hold_error,
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
        run.latch, run.stamps, stale_input_timeout_s(VEHICLE), run.stale, run.trace, END_LATCH_WAIT_S
    )
    assert run.hold_error is None, run.hold_error
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

    # A6: T is the parked spawn, taken at MOLA's first keyframe, and the map
    # holds no teleport. Under the defect T was 6.75 m off.
    provenance = yaml.safe_load(Path(f"{stem}.yaml").read_text())
    t = provenance["T_map_mola"]
    assert (
        math.hypot(t["x"] - SPAWN_MAP[0], t["y"] - SPAWN_MAP[1]) <= SPAWN_TOL_M
        and abs(_wrap(t["yaw"] - SPAWN_MAP[2])) <= SPAWN_TOL_RAD
    ), f"T_map_mola {t} vs the spawn {SPAWN_MAP}"
    keyframes = out / "mola_keyframes.tum"
    subprocess.run(
        [
            "sm-cli",
            "export-keyframes",
            str(simplemap),
            "--output",
            str(keyframes),
        ],
        check=True,
        capture_output=True,
    )
    rows = [
        [float(v) for v in line.split()]
        for line in keyframes.read_text().splitlines()
    ]
    assert provenance["T_stamp"] == rows[0][0], (provenance, rows[0])
    assert provenance["T_stamp_source"] == "simplemap_first_keyframe"
    truth = min(
        poses(read_recording(recording), "/ground_truth/odom"),
        key=lambda p: abs(p.stamp - rows[0][0]),
    )
    assert math.hypot(truth.x - t["x"], truth.y - t["y"]) <= 1e-3, (truth, t)
    assert abs(_wrap(truth.yaw - t["yaw"])) <= 1e-4, (truth, t)
    step = max(
        math.hypot(b[1] - a[1], b[2] - a[2])
        for a, b in zip(rows, rows[1:], strict=False)
    )
    assert step < MAX_KEYFRAME_STEP_M, f"keyframe step {step:.2f} m: a teleport"
