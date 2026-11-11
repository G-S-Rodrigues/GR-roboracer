"""SIM-3760: one rung of the closed-loop corner sweep (part 1L) on the real
AutoDRIVE sim, Porto, two laps, with the lateral channels recorded (part 1L).

LIVE: needs the AutoDRIVE sim container connected to this launch's server, so
it lives in tests/autodrive/ and never under tests/system/ (no gate has a sim,
ADR 0007). Run it with `scripts/autodrive_qualify.sh --domain <60-69> --case
corner_sweep --sweep-dir <dir in dev>`. With no sim it FAILS within the frame
wait, naming that command; it never skips.

`AUTODRIVE_SWEEP_DIR` is a rung's directory from the throwaway sweep-line
generator: `porto.yaml` (the scenario copy, whose map copy holds the rung's
raceline), `autodrive_roboracer_fast.yaml` (the vehicle copy) and
`sweep.json`. The launch is the track centerline planner with the speed from
the line (`speed_source:=trajectory`). The route depends on the launch's
stem-only checks (a vehicle and a scenario outside config/ keep their stems);
a launch refusal shows in `launch.log` of the run directory.

It asserts only run validity: the spawn, no latch (a STALE_INPUT or an
ESTOP_LATCHED status), and a non-empty IMU channel with `linear_acceleration`
present. A collision is RECORDED (corner_sweep.json), not a failure: it is the
ladder's stop point (the plan's Step 5d). Laps, sim/wall, the load and the
server's stamp status are reported and never asserted: they are host
properties (ADR 0008 section 1). The run json and channels.jsonl.gz are written
before any assert, so a failing rung keeps its data.
"""

import gzip
import hashlib
import json
import os
import subprocess
from pathlib import Path

from _porto_drive import SPAWN_MAP, SPAWN_TOL_M, SPAWN_TOL_RAD, drive_porto
from racing_test_keywords.autodrive_live import judge_stale_input

LAPS = 2
BUDGET_S = 300.0
HOST_COMMAND = (
    "scripts/autodrive_qualify.sh --domain <60-69> --case corner_sweep "
    "--sweep-dir <dir>"
)
END_LATCH_WAIT_S = 1.0


def _launch(sweep: Path) -> list[str]:
    return [
        "ros2",
        "launch",
        "racing_bringup",
        "sim_pure_pursuit.launch.py",
        "simulator:=autodrive",
        f"vehicle:={sweep / 'autodrive_roboracer_fast.yaml'}",
        f"scenario:={sweep / 'porto.yaml'}",
        "use_rviz:=false",
        "planner:=track_centerline",
        "speed_source:=trajectory",
        f"laps:={LAPS}",
        "autodrive_port:=4567",
    ]


def _sim_wall(rows: list[dict]) -> float | None:
    """Summed positive `lap_time` deltas of the counters stream over the wall
    span of the same frames (there is no /clock under AutoDRIVE)."""
    counters = [r for r in rows if r["stream"] == "counters"]
    if len(counters) < 2:
        return None
    sim = sum(
        max(b["lap_time"] - a["lap_time"], 0.0)
        for a, b in zip(counters, counters[1:], strict=False)
    )
    wall = counters[-1]["t"] - counters[0]["t"]
    return sim / wall if wall > 0 else None


def test_sim_3760_porto_corner_sweep(tmp_path):
    sweep = Path(os.environ["AUTODRIVE_SWEEP_DIR"])
    out = Path(os.environ.get("AUTODRIVE_OUT", tmp_path))
    sweep_json = json.loads((sweep / "sweep.json").read_text())
    uptime = subprocess.run(
        ["uptime"], capture_output=True, text=True, check=False
    ).stdout.strip()
    run = drive_porto(
        _launch(sweep),
        LAPS,
        out,
        budget_s=BUDGET_S,
        host_command=HOST_COMMAND,
        node_name="sim_3760_driver",
        channels=True,
        meta={
            "sweep": sweep_json,
            "line_sha256": sweep_json.get("line_sha256"),
            "vehicle_copy": (
                sweep / "autodrive_roboracer_fast.yaml"
            ).read_text(),
            "uptime": uptime,
        },
    )
    m = run.metrics
    with gzip.open(out / "channels.jsonl.gz", "rt", encoding="utf-8") as f:
        rows = [json.loads(line) for line in f][1:]
    imu = [r for r in rows if r["stream"] == "imu"]
    status = run.stamp_status
    result = {
        "sweep": sweep.name,
        "line_sha256": sweep_json.get("line_sha256"),
        "channels_sha256": hashlib.sha256(
            (out / "channels.jsonl.gz").read_bytes()
        ).hexdigest(),
        "laps_stack_s": list(m.lap_times) if m else [],
        "laps_sim_s": [i[1] for i in run.increments],
        "collisions_stack": int(m.collision_count) if m else None,
        "collisions_sim": run.sim_collisions,
        "safety_reasons": dict(run.reasons),
        "stale_input": run.stale,
        "host_load": run.load,
        "uptime": uptime,
        "wall_s": run.wall_s,
        "sim_over_wall": _sim_wall(rows),
        "imu_frames": len(imu),
        "stamp_clamped_frames": (
            status.get("clamped_frames") if status else None
        ),
        "stamp_step_backs": status.get("step_backs") if status else None,
        "stamp_largest_behind_ms": (
            status.get("largest_behind_ms") if status else None
        ),
    }
    (out / "corner_sweep.json").write_text(json.dumps(result, indent=2))
    print("CORNER_SWEEP", json.dumps(result))

    verdict = judge_stale_input(
        run.latch,
        run.stamps,
        0.1,
        run.stale,
        run.trace,
        END_LATCH_WAIT_S,
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
    assert imu, "no IMU messages recorded"
    assert any(r["ax"] != 0.0 or r["ay"] != 0.0 for r in imu), (
        "imu_ax/imu_ay all zero: a server from before SIMAD-2300"
    )
