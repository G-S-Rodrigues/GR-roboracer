"""SIM-3530: MOLA's mapping workflow produces a map the stack can read.

It measures **MOLA's own map** from one mapping lap at 1x on
`spielberg_1080.yaml`, seed 42 (Spielberg, never analytic_circle:
repo-gotchas #18). The car drives on ground truth; MOLA reads only `/scan`
and `/odom`. `scripts/mola_map_track.sh` writes the world-frame occupancy
grid `<stem>_grid.{png,yaml}` beside the native `.mm`, with no loop closure
(Step 0b of the plan measured 0 accepted loop closures on 2D data: the
closed map was unchanged to 1e-4 m, so it was not adopted). The grid is
placed in `map` by the truth-derived T, which is used only for scoring.

The metric is the raw cell-centre distance to the true wall faces, so a
perfect 5 cm map scores 2.5 cm (owner ruling O2). The q3 targets (wall
p95 <= 5 cm, |mean offset| <= 3 cm) are printed, not gated.

The second case loads the committed grid through nav2's map_server, the
loader amcl uses, which is what "amcl consumes it unchanged" (acceptance 2)
means without claiming it localizes.
"""

import contextlib
import dataclasses
import os
import signal
import subprocess
import time
from collections.abc import Iterator
from pathlib import Path

import pytest
import yaml
from racing_evaluation.map_accuracy import (
    load_grid,
    map_to_wall,
    scenario_truth_grid,
)
from racing_test_keywords.report import catastrophic_violations, write_report

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SCENARIO = REPOSITORY_ROOT / "config/scenarios/spielberg_1080.yaml"
COMMITTED_GRID = (
    REPOSITORY_ROOT
    / "third_party"
    / "GR-roboracer-data"
    / "tracks"
    / "Spielberg"
    / "maps"
    / "Spielberg"
    / "mola"
    / "Spielberg_grid.yaml"
)
SEED = 42
TIMEOUT_SECONDS = 900.0

# The old bounds below are kept as the baseline a report is read against
# (ADR 0008); no assert reads them. The asserts are the catastrophic bounds in
# `racing_test_keywords.report`. How each was set: each bound is the worst lap
# x 1.25 rounded up to 1 cm; the cell floor is the minimum x 0.8. The bounds
# were first set on five laps of the baseline mapping pipeline (p95 0.518-
# 0.988 m: 1.24 / 0.54 / 0.10 / 34 500). Part 2 then changed the mapping
# pipeline (icp_prior_weight 50, which makes seed 7 close and halves its
# wall p95), so they are re-measured on ten laps of the final pipeline,
# 1x, no loop closure, dev container slam-p2, 2026-10-29/30, host load 4-8
# at each start (seeds 7 and 42; wall p95 / median / signed_mean / cells):
#   s42 (this test)  1.565 0.410 +0.1604 43716  <- worst p95, worst signed
#   s42 (this test)  0.790 0.196 +0.0849 43651
#   s42 (tuning)     0.921 0.210 +0.0871 43761
#   s42 (tuning)     0.892 0.409 +0.0902 43708
#   s7  (tuning)     1.149 0.390 +0.0926 43812
#   s7  (tuning)     0.877 0.260 +0.0770 43835
#   s7  (script)     0.993 0.310 +0.0783 43660
#   s7  (script)     0.890 0.240 +0.0800 43722
#   s7  (script)     1.010 0.250 +0.0798 43644
#   s7  (committed)  0.821 0.214 +0.0771 43631  <- fewest cells
# worst p95 1.565 -> 1.96; worst median 0.410 -> 0.52; worst |signed_mean|
# 0.1604 -> 0.21 (0.2005 rounded up); fewest cells 43 631 x 0.8 -> 34 905.
# The spread (p95 0.79-1.57 m over identical-seed laps) is MOLA's lap-to-lap
# drift with no loop closure, not noise in the scorer, and it is wider than
# the baseline pipeline's: the bound guards against a different map, not
# against a different drift. Q3 targets (spec acceptance 7: wall p95 <= 5 cm,
# |mean offset| <= 3 cm) are NOT met and are reported, not gated: they apply
# to the known-pose survey map only; the measured p95 is 16-31x the target.
MEASUREMENT = (
    "ten laps of the final pipeline (seeds 7 and 42), wall p95 0.790-1.565 "
    "m, median 0.196-0.410 m, signed_mean +0.0770..+0.1604 m, 43 631-43 835 "
    "wall cells"
)
MIN_CELLS_BASELINE = 34905
P95_BASELINE = 1.96
MEDIAN_BASELINE = 0.52
SIGNED_BASELINE = 0.21
Q3_P95_TARGET = 0.05
Q3_MEAN_TARGET = 0.03
BASELINE = {
    "wall_cell_count": MIN_CELLS_BASELINE,
    "p95": P95_BASELINE,
    "median": MEDIAN_BASELINE,
    "signed_mean": SIGNED_BASELINE,
    "q3_p95_target": Q3_P95_TARGET,
    "q3_mean_target": Q3_MEAN_TARGET,
    "measurement": MEASUREMENT,
}


def _processes() -> Iterator[tuple[int, int, int, str]]:
    """Live (pid, ppid, pgid, cmdline) rows from /proc, zombies skipped.

    The dev container has no init, so a reparented child that exits stays a
    zombie for ever and `killpg(pgid, 0)` still succeeds on its group.
    """
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
            command = (entry / "cmdline").read_bytes()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        fields = stat.rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            continue
        text = command.replace(b"\0", b" ").decode(errors="replace").strip()
        yield int(entry.name), int(fields[1]), int(fields[2]), text


def _live_group_members(pgid: int) -> list[tuple[int, str]]:
    return [(p, c) for p, _, g, c in _processes() if g == pgid]


def _child_groups(pid: int) -> set[int]:
    return {g for _, pp, g, _ in _processes() if pp == pid}


def _named_survivors(domain: str) -> list[tuple[int, str]]:
    found = []
    for pid, _, _, command in _processes():
        if "__node:=sim_3530_map_server" not in command:
            continue
        try:
            raw = Path(f"/proc/{pid}/environ").read_bytes()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        variables = dict(
            item.decode(errors="replace").partition("=")[::2]
            for item in raw.split(b"\0")
            if item
        )
        if variables.get("ROS_DOMAIN_ID", "0") == domain:
            found.append((pid, command))
    return found


def _kill_group(pgid: int) -> None:
    with contextlib.suppress(ProcessLookupError):
        os.killpg(pgid, signal.SIGKILL)


def _stop(process: subprocess.Popen, stop_signal: int = signal.SIGINT) -> None:
    """Stop `process` and leave no live process in its group or its children's.

    Repo-gotchas #10 in a `ros2 run` form: `Popen.terminate()` signals the
    Python `ros2` wrapper only and the node it ran is reparented to pid 1,
    still active. SIM-3610 and SIM-3620 stop their launch the same way
    (killpg SIGINT, then SIGKILL after 30 s); this copy adds the poll and the
    children's groups. The poll does not close a race on SIGINT (the wrapper
    outlives its child); it covers a wrapper already dead and a child that
    ignores the signal, and makes "nothing left" a checked result. A script's
    `setsid` children are in groups of their own and, once the script is dead,
    are reparented away, so their groups are read before any signal.
    """
    groups = {process.pid} | _child_groups(process.pid)
    deadline = time.monotonic() + 30

    def live() -> list[tuple[int, str]]:
        return [m for g in groups for m in _live_group_members(g)]

    for group in groups:
        with contextlib.suppress(ProcessLookupError):
            os.killpg(group, stop_signal)
    with contextlib.suppress(subprocess.TimeoutExpired):
        process.wait(timeout=30)
    while live() and time.monotonic() < deadline:
        time.sleep(0.1)
    if live():
        for group in groups:
            _kill_group(group)
        with contextlib.suppress(subprocess.TimeoutExpired):
            process.wait(timeout=10)
        end = time.monotonic() + 10
        while live() and time.monotonic() < end:
            time.sleep(0.1)
    left = live()
    assert not left, "still alive after SIGKILL: " + ", ".join(
        f"{p} {c}" for p, c in left
    )


def _run_script(command: list[str], log: Path, timeout: float) -> int:
    # Output goes to a file, never a pipe. SIGTERM, not SIGINT: a background
    # job of a non-interactive shell starts with SIGINT ignored
    # (mola_map_track.sh:126-127).
    with log.open("w") as handle:
        process = subprocess.Popen(
            command,
            cwd=REPOSITORY_ROOT,
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    try:
        return process.wait(timeout=timeout)
    finally:
        if process.poll() is None:
            _stop(process, signal.SIGTERM)


@contextlib.contextmanager
def _map_server() -> Iterator[subprocess.Popen]:
    server = subprocess.Popen(
        [
            "ros2",
            "run",
            "nav2_map_server",
            "map_server",
            "--ros-args",
            "-p",
            f"yaml_filename:={COMMITTED_GRID}",
            "-r",
            "__node:=sim_3530_map_server",
        ],
        env=os.environ,
        start_new_session=True,
    )
    try:
        yield server
    finally:
        _stop(server)


def test_sim_3530_mapping_lap_grid_fits_the_walls(tmp_path: Path) -> None:
    stem = tmp_path / "Spielberg"
    # Output goes to a file, never a pipe: a log file is simpler to
    # read back after a failure. (The script's watchdogs no longer hold a
    # pipe open, BRINGUP-1770; this once timed out at 905 s.)
    log = tmp_path / "mola_map_track.log"
    returncode = _run_script(
        [
            "./scripts/mola_map_track.sh",
            "config/scenarios/spielberg_1080.yaml",
            str(stem),
            str(SEED),
        ],
        log,
        TIMEOUT_SECONDS,
    )
    assert returncode == 0, log.read_text()[-2000:]
    provenance = yaml.safe_load(stem.with_suffix(".yaml").read_text("utf-8"))
    assert provenance["scenario"] == "config/scenarios/spielberg_1080.yaml"
    assert provenance["seed"] == SEED
    accuracy = map_to_wall(
        load_grid(tmp_path / "Spielberg_grid.yaml"),
        scenario_truth_grid(SCENARIO),
    )
    print(
        f"SIM-3530 {accuracy}; measured: {MEASUREMENT}; q3 targets (reported,"
        f" not gated): p95 <= {Q3_P95_TARGET}, |mean| <= {Q3_MEAN_TARGET}"
    )
    numbers = dataclasses.asdict(accuracy)
    # Reported before anything is asserted, so a red run keeps its record
    # (ADR 0008).
    write_report(
        "SIM-3530",
        numbers,
        seed=SEED,
        scenario="config/scenarios/spielberg_1080.yaml",
        implementation="mola",
        baseline=BASELINE,
    )
    violations = catastrophic_violations(
        numbers, {"wall_cell_count": "wall_cells", "median": "map_median"}
    )
    assert violations == [], (violations, accuracy)


def test_sim_3530_map_server_loads_the_grid() -> None:
    """nav2 map_server serves the committed grid, and `/map` matches the
    yaml's size, resolution and origin."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    document = yaml.safe_load(COMMITTED_GRID.read_text("utf-8"))
    grid = load_grid(COMMITTED_GRID)
    height, width = grid.occupied.shape
    try:
        with _map_server():
            rclpy.init()
            node = rclpy.create_node("sim_3530_probe")
            received = []
            node.create_subscription(
                OccupancyGrid,
                "/map",
                received.append,
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )
            for transition in ("configure", "activate"):
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    done = subprocess.run(
                        [
                            "ros2",
                            "lifecycle",
                            "set",
                            "/sim_3530_map_server",
                            transition,
                        ],
                        capture_output=True,
                        text=True,
                        check=False,
                    )
                    if "successful" in done.stdout:
                        break
                    time.sleep(1)
            deadline = time.monotonic() + 30
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
            assert received, "map_server published no /map"
            message = received[0]
            assert message.info.width == width and message.info.height == height
            assert abs(message.info.resolution - document["resolution"]) < 1e-9
            assert message.info.origin.position.x == document["origin"][0]
            assert message.info.origin.position.y == document["origin"][1]
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_sim_3530_map_server_leaves_no_process_behind() -> None:
    """SIM-3530: no process started for the map_server outlives the test.

    `ros2 run` is a Python wrapper; terminating it orphaned the node, active
    on the gate's domain and serving MOLA's grid on /map (research
    2026-11-02-sim3080-in-gate; ACC-4030's 0.70 fits it, not verified).
    """
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    pgid = 0
    try:
        with _map_server() as server:
            pgid = server.pid  # start_new_session: pgid == pid
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                if any(
                    p != server.pid and "map_server" in c
                    for p, c in _live_group_members(pgid)
                ):
                    break
                time.sleep(0.1)
            else:
                pytest.fail("map_server child never appeared in the group")
        left = _live_group_members(pgid)
        assert left == [], "\n".join(f"{p} {c}" for p, c in left)
        named = _named_survivors(domain)
        assert named == [], "\n".join(f"{p} {c}" for p, c in named)
    finally:
        if pgid:
            _kill_group(pgid)


def test_sim_3530_mapping_timeout_leaves_no_process_behind(
    tmp_path: Path,
) -> None:
    """SIM-3530: a timed-out script leaves neither `setsid` group alive.

    The stand-in has the shape of mola_map_track.sh:68-75 without its EXIT
    trap, which a SIGKILL of the script skips.
    """
    pids = tmp_path / "pids"
    script = tmp_path / "standin.sh"
    script.write_text(
        "#!/bin/bash\n"
        f"setsid sleep 300 & echo $! >> {pids}\n"
        f"setsid sleep 300 & echo $! >> {pids}\n"
        "wait\n"
    )
    script.chmod(0o755)
    groups: list[int] = []
    try:
        with pytest.raises(subprocess.TimeoutExpired):
            _run_script([str(script)], tmp_path / "standin.log", 3.0)
        groups = [int(x) for x in pids.read_text().split()]
        assert len(groups) == 2, groups
        left = [m for g in groups for m in _live_group_members(g)]
        assert left == [], "\n".join(f"{p} {c}" for p, c in left)
    finally:
        for group in groups:
            _kill_group(group)
