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

# Rule (gotcha #14, the spec's testing section): each bound is the worst lap
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
# 0.1604 -> 0.21 (0.2005 rounded up); fewest cells 43 631 x 0.8 -> 34 900.
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
MIN_CELLS = 34900
P95_BOUND = 1.96
MEDIAN_BOUND = 0.52
SIGNED_BOUND = 0.21
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
