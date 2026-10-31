"""SIM-3730: the stack qualifies on the real AutoDRIVE sim, Porto, 12 laps,
and the stack's metrics agree with the sim's own counters (#44 part 10).

LIVE: needs the AutoDRIVE sim container connected to this launch's server, so
it lives in tests/autodrive/ and never under tests/system/ (no gate has a sim,
ADR 0007). Run it with `scripts/autodrive_qualify.sh --domain <60-69>`, which
recreates the single-use sim container, runs this in `dev` and stops the sim.
With no sim it FAILS within the frame wait, naming that command; it never
skips.

t = 0 is the first frame after `/racing_sim/reset` (gotcha 19). The run ends
when metrics publishes (laps:=12) and the sim's own `lap_count` reaches 12.
"""

import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import rclpy
from conftest import stop
from nav_msgs.msg import Odometry
from racing_interfaces.msg import ScenarioMetrics, SimCounters
from racing_interfaces.srv import Reset
from racing_test_keywords.autodrive_live import judge_stale_input
from racing_test_keywords.ros_helpers import (
    ensure_rclpy_initialized,
    reset_rclpy,
    wait_for_endpoints,
    wait_for_node,
)
from racing_test_keywords.scenario_runner import (
    METRICS_QOS,
    PARTICIPANT_NODES,
    RELIABLE_QOS,
    _HangObserver,
    host_load_delta,
    read_host_load,
)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "ros_ws/src/racing_evaluation"))
from racing_evaluation.qualification import score  # noqa: E402

LAPS = 12
BUDGET_S = 600.0
STARTUP_S = 120.0
FRAME_WAIT_S = 60.0
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
HOST_COMMAND = "scripts/autodrive_qualify.sh --domain <60-69>"
# The fixture's spawn CoG through porto_frame.yaml (its header): the first
# pose after the reset must be here, within 0.05 m / 0.05 rad. Closes the
# spawn-y topic (2026-10-30-autodrive-spawn-y-offset).
SPAWN_MAP = (-5.90, 2.00, 0.0)
SPAWN_TOL_M = 0.05
SPAWN_TOL_RAD = 0.05
# Per-lap |stack lap_time - sim last_lap_time| allowed, seconds. Set from the
# measured disagreement in step 7; capped at 0.3 s (gotcha 14).
LAP_TOL_S = 0.3
END_LATCH_WAIT_S = 1.0


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _yaw(q) -> float:
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
    )


def _wrap(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi


def test_sim_3730_porto_qualification(tmp_path):
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain is None or domain == "42":
        pytest.fail("ROS_DOMAIN_ID must be set and not 42 (run the wrapper)")
    out = Path(os.environ.get("AUTODRIVE_OUT", tmp_path))
    out.mkdir(parents=True, exist_ok=True)
    launch_log = out / "launch.log"
    with launch_log.open("w") as log:
        launched = subprocess.Popen(
            LAUNCH,
            cwd=REPO,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    ensure_rclpy_initialized()
    node = rclpy.create_node("sim_3730_driver")
    observer = None
    try:
        deadline = time.monotonic() + STARTUP_S
        for participant in PARTICIPANT_NODES:
            wait_for_node(
                node, participant, (), max(deadline - time.monotonic(), 1.0)
            )
        wait_for_endpoints(
            node,
            "racing_sim",
            {"/ground_truth/odom", "/ground_truth/scan"},
            "publisher",
            max(deadline - time.monotonic(), 1.0),
        )
        observer = _HangObserver(node)
        odoms: list[tuple[float, float, float, float]] = []
        counters: list[SimCounters] = []
        metrics: list[ScenarioMetrics] = []
        scans: list[tuple[LaserScan, tuple]] = []
        scan_count = [0]

        def on_odom(m):
            p = m.pose.pose
            odoms.append(
                (
                    _seconds(m.header.stamp),
                    p.position.x,
                    p.position.y,
                    _yaw(p.orientation),
                )
            )

        def on_scan(m):
            scan_count[0] += 1
            if scan_count[0] % 5 == 0 and odoms:
                scans.append((m, odoms[-1]))

        node.create_subscription(
            Odometry, "/ground_truth/odom", on_odom, qos_profile_sensor_data
        )
        node.create_subscription(
            LaserScan,
            "/ground_truth/scan",
            on_scan,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            SimCounters,
            "/racing_sim/autodrive_counters",
            counters.append,
            RELIABLE_QOS,
        )
        node.create_subscription(
            ScenarioMetrics, "/scenario/metrics", metrics.append, METRICS_QOS
        )

        # The sim's first frame: with no sim container, fail here, telling
        # the human what to run.
        end = time.monotonic() + FRAME_WAIT_S
        while not odoms and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not odoms:
            pytest.fail(
                f"no AutoDRIVE frame within {FRAME_WAIT_S:.0f} s: the sim is"
                f" not connected. Run from the host: {HOST_COMMAND}, which"
                " recreates the single-use sim container (docker compose"
                " --profile autodrive up -d --no-deps --force-recreate"
                " autodrive-sim, AUTODRIVE_BRIDGE_HOST=gr-roboracer-dev)"
            )

        client = node.create_client(Reset, "/racing_sim/reset")
        assert client.wait_for_service(timeout_sec=30.0)
        future = client.call_async(Reset.Request(seed=42, scenario_id=""))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        assert future.done() and future.result().success, "reset failed"
        counters.clear()
        metrics.clear()
        scans.clear()
        first_after = len(odoms)
        load_before = read_host_load()

        increments: list[tuple[float, float, int]] = []  # wall, last, count
        last_count = 0
        began = time.monotonic()
        while time.monotonic() - began < BUDGET_S:
            rclpy.spin_once(node, timeout_sec=0.02)
            if counters and counters[-1].lap_count > last_count:
                c = counters[-1]
                increments.append(
                    (time.monotonic() - began, c.last_lap_time, c.collisions)
                )
                last_count = c.lap_count
            if metrics and last_count >= LAPS:
                break
        load = host_load_delta(load_before, read_host_load())
        stale = observer.stale_count
        latch = observer.first_latch
        trace = observer._startup_trace()
    finally:
        if observer is not None:
            observer.close()
        node.destroy_node()
        reset_rclpy()
        stop(launched, 60.0)
    stamps = [o[0] for o in odoms]
    periods = np.diff(stamps)
    rows = np.array([s[1] for s in scans])
    np.savez(
        out / "porto_run.npz",
        poses=rows[:, 1:] if len(rows) else np.zeros((0, 3)),
        scans=np.array([s[0].ranges for s in scans], dtype=float),
        angle_min=scans[0][0].angle_min if scans else 0.0,
        angle_increment=scans[0][0].angle_increment if scans else 0.0,
        range_max=scans[0][0].range_max if scans else 0.0,
    )
    m = metrics[-1] if metrics else None
    stack_laps = list(m.lap_times) if m else []
    sim_laps = [i[1] for i in increments]
    sim_collisions = counters[-1].collisions if counters else -1
    result = {
        "laps_sim": sim_laps,
        "laps_stack": stack_laps,
        "collisions_sim": sim_collisions,
        "collisions_stack": int(m.collision_count) if m else None,
        "period_s": {
            "median": float(np.median(periods)),
            "p99": float(np.percentile(periods, 99)),
            "max": float(periods.max()),
        },
        "stale_input": stale,
        "host_load": load,
        "wall_s": time.monotonic() - began,
    }
    if len(stack_laps) >= LAPS:
        result["score_stack"] = score(stack_laps, sim_collisions).total
    if len(sim_laps) >= LAPS:
        result["score_sim"] = score(sim_laps, sim_collisions).total
    (out / "qualification.json").write_text(json.dumps(result, indent=2))
    print("QUALIFICATION", json.dumps(result))

    first = odoms[first_after]
    spawn_error = (
        math.hypot(first[1] - SPAWN_MAP[0], first[2] - SPAWN_MAP[1]),
        abs(_wrap(first[3] - SPAWN_MAP[2])),
    )
    verdict = judge_stale_input(
        latch,
        stamps,
        0.1,
        stale,
        trace,
        END_LATCH_WAIT_S,
    )
    assert stale == 0, f"STALE_INPUT x{stale}: {verdict}"
    assert spawn_error[0] <= SPAWN_TOL_M and spawn_error[1] <= SPAWN_TOL_RAD, (
        f"first pose after reset {first[1:]} vs {SPAWN_MAP}: {spawn_error}"
    )
    assert m is not None, "metrics never published"
    if last_count == 0 and m.laps_completed > 0:
        pytest.fail(
            "the sim's lap_count stayed 0 while metrics counted"
            f" {m.laps_completed} laps: direction or lap line"
        )
    assert last_count >= LAPS, f"sim lap_count {last_count} < {LAPS}"
    assert m.laps_completed == LAPS, m.laps_completed
    assert sim_collisions <= 10, sim_collisions
    for k in range(1, LAPS):  # laps 2..12; lap 1 starts off the line
        assert abs(stack_laps[k] - sim_laps[k]) <= LAP_TOL_S, (
            f"lap {k + 1}: stack {stack_laps[k]:.3f} s vs sim"
            f" {sim_laps[k]:.3f} s (tol {LAP_TOL_S})"
        )
