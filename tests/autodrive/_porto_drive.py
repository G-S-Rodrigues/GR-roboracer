"""The live Porto driver shared by SIM-3730 and SIM-3750 (#44).

Not a test. Launches the stack against the AutoDRIVE sim, takes the t = 0
reset (gotcha 19), drives `laps` laps and returns what was observed; each test
judges it and writes its own JSON. Extracted from SIM-3730 unchanged in
behaviour (its asserts and `qualification.json` keys stay the same).
"""

import math
import os
import subprocess
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pytest
import rclpy
from conftest import stop
from nav_msgs.msg import Odometry
from racing_interfaces.msg import SafetyStatus, ScenarioMetrics, SimCounters
from racing_interfaces.srv import Reset
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
STARTUP_S = 120.0
FRAME_WAIT_S = 60.0
# The fixture's spawn CoG through porto_frame.yaml (its header): the first
# pose after the reset must be here, within 0.05 m / 0.05 rad. Closes the
# spawn-y topic (2026-10-30-autodrive-spawn-y-offset).
SPAWN_MAP = (-5.8454, 1.9713, 0.0028)
SPAWN_WINDOW = 100  # frames, 0.6 s
SPAWN_TOL_M = 0.05
SPAWN_TOL_RAD = 0.05


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _yaw(q) -> float:
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
    )


def _wrap(angle: float) -> float:
    return (angle + math.pi) % math.tau - math.pi


def _reason_name(value: int) -> str:
    for name in dir(SafetyStatus):
        if name.startswith("REASON_") and getattr(SafetyStatus, name) == value:
            return name
    return f"REASON_{value}"


@dataclass
class PortoRun:
    stamps: list[float]
    speeds: list[float]
    metrics: ScenarioMetrics | None
    increments: list[tuple[float, float, int]]  # wall, last lap, collisions
    last_count: int
    sim_collisions: int
    stale: int
    latch: object
    trace: object
    load: object
    wall_s: float
    first_after: int
    spawn_error: tuple[float, float] | None  # None: no frame after reset
    first_pose: tuple | None
    reasons: Counter = field(default_factory=Counter)

    @property
    def periods(self) -> np.ndarray:
        return np.diff(self.stamps)


def drive_porto(
    launch: list[str],
    laps: int,
    out: Path,
    *,
    budget_s: float,
    host_command: str,
    node_name: str,
) -> PortoRun:
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain is None or domain == "42":
        pytest.fail("ROS_DOMAIN_ID must be set and not 42 (run the wrapper)")
    out.mkdir(parents=True, exist_ok=True)
    launch_log = out / "launch.log"
    with launch_log.open("w") as log:
        launched = subprocess.Popen(
            launch,
            cwd=REPO,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    ensure_rclpy_initialized()
    node = rclpy.create_node(node_name)
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
        speeds: list[float] = []  # twist.linear, parallel to odoms
        reasons: Counter = Counter()
        counters: list[SimCounters] = []
        metrics: list[ScenarioMetrics] = []
        scans: list[tuple[LaserScan, tuple]] = []
        scan_count = [0]

        def on_odom(m):
            p = m.pose.pose
            tw = m.twist.twist.linear
            speeds.append(math.hypot(tw.x, tw.y))
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
            SafetyStatus,
            "/safety/status",
            lambda s: reasons.update([_reason_name(s.reason)]),
            RELIABLE_QOS,
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
                f" not connected. Run from the host: {host_command}, which"
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
        while time.monotonic() - began < budget_s:
            rclpy.spin_once(node, timeout_sec=0.02)
            if counters and counters[-1].lap_count > last_count:
                c = counters[-1]
                increments.append(
                    (time.monotonic() - began, c.last_lap_time, c.collisions)
                )
                last_count = c.lap_count
            if metrics and last_count >= laps:
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
    wall_s = time.monotonic() - began
    stamps = [o[0] for o in odoms]
    rows = np.array([s[1] for s in scans])
    np.savez(
        out / "porto_run.npz",
        poses=rows[:, 1:] if len(rows) else np.zeros((0, 3)),
        scans=np.array([s[0].ranges for s in scans], dtype=float),
        angle_min=scans[0][0].angle_min if scans else 0.0,
        angle_increment=scans[0][0].angle_increment if scans else 0.0,
        range_max=scans[0][0].range_max if scans else 0.0,
    )
    # Frames already in flight when the reset answers still show the old
    # pose (live run 3, 2026-10-31: the first one was 4.5 m away), so the
    # spawn is the closest of the first SPAWN_WINDOW frames after the call;
    # the car starts at rest, so the true first frame is the nearest.
    window = odoms[first_after : first_after + SPAWN_WINDOW]
    if window:
        first = min(
            window,
            key=lambda o: math.hypot(o[1] - SPAWN_MAP[0], o[2] - SPAWN_MAP[1]),
        )
        spawn_error = (
            math.hypot(first[1] - SPAWN_MAP[0], first[2] - SPAWN_MAP[1]),
            abs(_wrap(first[3] - SPAWN_MAP[2])),
        )
        first_pose = first[1:]
    else:  # the caller writes its JSON, then fails on None
        spawn_error = None
        first_pose = None
    return PortoRun(
        stamps=stamps,
        speeds=speeds,
        metrics=metrics[-1] if metrics else None,
        increments=increments,
        last_count=last_count,
        sim_collisions=counters[-1].collisions if counters else -1,
        stale=stale,
        latch=latch,
        trace=trace,
        load=load,
        wall_s=wall_s,
        first_after=first_after,
        spawn_error=spawn_error,
        first_pose=first_pose,
        reasons=reasons,
    )
