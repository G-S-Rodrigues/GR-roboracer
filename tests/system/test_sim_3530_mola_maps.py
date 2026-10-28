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

import os
import subprocess
import time
from pathlib import Path

import yaml
from racing_evaluation.map_accuracy import (
    load_grid,
    map_to_wall,
    scenario_truth_grid,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SCENARIO = REPOSITORY_ROOT / "config/scenarios/spielberg_1080.yaml"
COMMITTED_GRID = (
    REPOSITORY_ROOT / "config/scenarios/maps/Spielberg/mola/Spielberg_grid.yaml"
)
SEED = 42
TIMEOUT_SECONDS = 900.0

# Rule (gotcha #14, the spec's testing section): each bound is the worst of
# five seed-42 laps x 1.25 rounded up to 1 cm; the cell floor is the minimum
# x 0.8. Measured 2026-10-28, dev container slam-p2, 1x, no loop closure,
# host load 0.2-4.4 at each start, 140-145 s per mapping script run:
#   run  wall_cells  p95 (m)   median (m)  signed_mean (m)
#   1    43372       0.9885     0.3514      +0.0748
#   2    43180       0.5180     0.1604      +0.0784
#   3    43526       0.8604     0.3107      +0.0752
#   4    43445       0.8104     0.4261      +0.0743
#   5    43521       0.8104     0.2873      +0.0735
# worst p95 0.9885 -> 1.24; worst median 0.4261 -> 0.54; worst |signed_mean|
# 0.0784 -> 0.10; fewest cells 43180 -> 34 544 -> 34 500. The spread of the
# p95 over five identical-seed laps (0.52-0.99 m) is MOLA's lap-to-lap drift
# with no loop closure, not noise in the scorer; the bound guards against a
# different map, not against a different drift. Q3 targets (spec acceptance
# 7: wall p95 <= 5 cm, |mean offset| <= 3 cm) are NOT met and are reported,
# not gated: the measured p95 is 10-20x the target.
MEASUREMENT = (
    "five seed-42 laps, wall p95 0.518-0.988 m, median 0.160-0.426 m, "
    "signed_mean +0.0735..+0.0784 m, 43 180-43 526 wall cells"
)
MIN_CELLS = 34500
P95_BOUND = 1.24
MEDIAN_BOUND = 0.54
SIGNED_BOUND = 0.10
Q3_P95_TARGET = 0.05
Q3_MEAN_TARGET = 0.03


def test_sim_3530_mapping_lap_grid_fits_the_walls(tmp_path: Path) -> None:
    stem = tmp_path / "Spielberg"
    # Output goes to a file, never a pipe: the script's background
    # watchdogs inherit a pipe and hold it open for up to 900 s after the
    # script exits (measured: a capture_output run timed out at 905 s).
    log = tmp_path / "mola_map_track.log"
    with log.open("w") as handle:
        completed = subprocess.run(
            [
                "./scripts/mola_map_track.sh",
                "config/scenarios/spielberg_1080.yaml",
                str(stem),
                str(SEED),
            ],
            cwd=REPOSITORY_ROOT,
            stdout=handle,
            stderr=subprocess.STDOUT,
            timeout=TIMEOUT_SECONDS,
            check=False,
        )
    assert completed.returncode == 0, log.read_text()[-2000:]
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
    assert accuracy.wall_cell_count >= MIN_CELLS, accuracy
    assert accuracy.p95 <= P95_BOUND, accuracy
    assert accuracy.median <= MEDIAN_BOUND, accuracy
    assert abs(accuracy.signed_mean) <= SIGNED_BOUND, accuracy


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
        server.terminate()
        server.wait(timeout=30)
        if rclpy.ok():
            rclpy.shutdown()
