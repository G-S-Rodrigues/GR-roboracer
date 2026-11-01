"""SIM-3740: follow_the_gap plus the survey mapper lap AutoDRIVE's Porto and
save a map, on the sim's dead-reckoned odometry (#44 part 11a).

LIVE: needs the AutoDRIVE sim container connected to this launch's server, so
it lives in tests/autodrive/ and never under tests/system/ (no gate has a sim,
ADR 0007). Run it with `scripts/autodrive_qualify.sh --case mapping --domain
<60-69>`, which recreates the single-use sim container, runs this in `dev`
and stops the sim. With no sim it FAILS within the frame wait, naming that
command; it never skips and never retries.

What is and is not ground truth. The driver reads /scan only. The mapper
reads /scan and /odom, whose yaw rate is the sim's exact one: this is a map on
a *perfect-gyro odometry* (known problem 25), so the loop closure below
measures only the encoder-speed part of dead reckoning. The official
`Porto.pgm` is development ground truth: it is read for the shape score and
the supervisor's TRACK_LIMIT, never by the driver or the mapper.

t = 0 is the first frame after `/racing_sim/reset` (gotcha 19). The controller
is launched held (`controller_hold:=true`: it publishes zero commands), and
this test releases it from its own node once /scan has been steady for
SETTLE_S after the reset (K29, K30): a launch from standing in a wall-clock
sim was measured to spin the wheels, and a degraded sim makes the first
seconds unrepresentative. A latch before the release is reported as such.

Bounds (decision D6, numbers measured in part 11a Step 9, written beside each).
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
import yaml
from conftest import stop
from nav_msgs.msg import Odometry
from racing_interfaces.msg import SafetyStatus, ScenarioMetrics, SimCounters
from racing_interfaces.srv import Reset
from racing_test_keywords import autodrive_mapping as mapping
from racing_test_keywords.ros_helpers import (
    ensure_rclpy_initialized,
    reset_rclpy,
    wait_for_endpoints,
    wait_for_node,
)
from racing_test_keywords.scenario_runner import (
    METRICS_QOS,
    RELIABLE_QOS,
    _HangObserver,
    host_load_delta,
    participant_nodes,
    read_host_load,
)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "ros_ws/src/racing_evaluation"))
from racing_evaluation.map_accuracy import load_grid, map_to_wall  # noqa: E402

BUDGET_S = 180.0
STARTUP_S = 120.0
FRAME_WAIT_S = 60.0
SETTLE_S = 1.0  # steady /scan before the release (L2)
SETTLE_GAP_S = 0.05
SETTLE_CAP_S = 40.0
EXTRA_TRAVEL_M = 3.0  # past the lap, for start/end overlap (D5)
VEHICLE = "config/vehicles/autodrive_roboracer.yaml"
SERVER = "config/vehicles/autodrive_roboracer/server.yaml"
TRUTH = "config/scenarios/maps/Porto/Porto.yaml"
CONTROLLER = "racing_controller_follow_the_gap"
LAUNCH = [
    "ros2",
    "launch",
    "racing_bringup",
    "slam_toolbox_mapping.launch.py",
    "simulator:=autodrive",
    f"vehicle:={VEHICLE}",
    "scenario:=config/scenarios/porto.yaml",
    "controller:=follow_the_gap",
    "controller_hold:=true",
    "use_rviz:=false",
    "laps:=1",
    "autodrive_port:=4567",
]
HOST_COMMAND = "scripts/autodrive_qualify.sh --case mapping --domain <60-69>"
# The fixture's spawn CoG through porto_frame.yaml (SIM-3730's constant).
SPAWN_MAP = (-5.8454, 1.9713, 0.0028)
SPAWN_WINDOW = 100  # frames
RESOLUTION = 0.05
# Bounds, D6. FILLED at Step 9 from the measured runs.
LOOP_CLOSURE_M = float("inf")
LOOP_CLOSURE_RAD = float("inf")
WALL_CELLS_MIN = 0
SHAPE_MEDIAN_M = float("inf")
SHAPE_P95_M = float("inf")
FIT_RESIDUAL_M = float("inf")
AGREEMENT_MEDIAN_M = float("inf")
AGREEMENT_P95_M = float("inf")


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _yaw(q) -> float:
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
    )


def _find(document, key):
    if isinstance(document, dict):
        if key in document:
            return document[key]
        for value in document.values():
            found = _find(value, key)
            if found is not None:
                return found
    return None


def _laser_x() -> float:
    """The scan origin from the CoG: the vehicle file's rear-axle `laser_x`
    minus the server's lr (gotcha 26), never a literal."""
    vehicle = yaml.safe_load((REPO / VEHICLE).read_text(encoding="utf-8"))
    server = yaml.safe_load((REPO / SERVER).read_text(encoding="utf-8"))
    return float(_find(vehicle, "laser_x")) - float(_find(server, "lr"))


def _spin_until(node, seconds: float, done=lambda: False) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end and not done():
        rclpy.spin_once(node, timeout_sec=0.01)


def _release_hold(node, timeout_s: float = 20.0) -> None:
    """follow_the_gap's `hold` to false, through a parameter client on this
    node (L2: no `ros2` command-line process is started mid-run)."""
    client = node.create_client(SetParameters, f"/{CONTROLLER}/set_parameters")
    assert client.wait_for_service(timeout_sec=timeout_s), "no set_parameters"
    request = SetParameters.Request(
        parameters=[
            Parameter(
                name="hold",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=False
                ),
            )
        ]
    )
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    assert future.done(), "the hold release got no answer"
    assert all(r.successful for r in future.result().results), future.result()


def _provenance(result: dict) -> str:
    """The header written ahead of the saved Porto.yaml (the style of
    config/scenarios/maps/Spielberg/slam/Spielberg.yaml)."""
    simulator = yaml.safe_load(
        (REPO / "config/autodrive/simulator.yaml").read_text(encoding="utf-8")
    )
    lines = [
        "Porto's occupancy grid, mapped by this stack on AutoDRIVE's Porto",
        "(part 11a): follow_the_gap (scan-only driver) lapped once while",
        "slam_toolbox's survey mapper (config/localization/slam_toolbox_survey",
        ".yaml, scan matching off) laid /scan at the sim's dead-reckoned",
        "/odom.",
        "That odometry is a PERFECT-GYRO odometry (its yaw rate is the sim's",
        "exact one, known problem 25): never read this as a map built with",
        "no ground truth under a real gyro's drift.",
        "",
        f"Regenerate: {HOST_COMMAND} (sim: config/autodrive/simulator.yaml,"
        f" edition {simulator.get('edition', '?')})",
        "start_pose (first /odom pose after the reset): "
        f"{result['start_pose']}",
        f"git: {result['git_sha']}; host load: {result['host_load']}",
        f"lap time (sim): {result['lap_time_sim']}",
        "controller defaults: " + json.dumps(result["controller_defaults"]),
    ]
    for key in ("loop_closure", "shape", "agreement"):
        if result.get(key) is not None:
            lines.append(f"{key}: {json.dumps(result[key])}")
    return "".join(f"# {line}\n" for line in lines)


def test_sim_3740_porto_mapping(tmp_path):
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain is None or domain == "42":
        pytest.fail("ROS_DOMAIN_ID must be set and not 42 (run the wrapper)")
    out = Path(os.environ.get("AUTODRIVE_OUT", tmp_path))
    out.mkdir(parents=True, exist_ok=True)
    reference = os.environ.get("AUTODRIVE_REFERENCE_MAP")
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
    node = rclpy.create_node("sim_3740_driver")
    observer = None
    try:
        deadline = time.monotonic() + STARTUP_S
        for participant in participant_nodes("follow_the_gap"):
            wait_for_node(
                node, participant, (), max(deadline - time.monotonic(), 1.0)
            )
        wait_for_endpoints(
            node,
            "slam_toolbox",
            {"/scan"},
            "subscription",
            max(deadline - time.monotonic(), 1.0),
        )
        observer = _HangObserver(node)
        odoms: list[tuple[float, float, float, float]] = []
        gts: list[tuple[float, float, float]] = []
        counters: list[SimCounters] = []
        metrics: list[ScenarioMetrics] = []
        statuses: list[tuple[float, int]] = []
        scans: list[tuple[LaserScan, tuple]] = []
        scan_times: list[float] = []
        scan_count = [0]
        anchored = [None]  # index into odoms of the first post-reset pose

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

        def on_gt(m):
            p = m.pose.pose
            gts.append((_seconds(m.header.stamp), p.position.x, p.position.y))

        def on_scan(m):
            scan_times.append(time.monotonic())
            scan_count[0] += 1
            if scan_count[0] % 5 == 0 and anchored[0] is not None and odoms:
                scans.append((m, odoms[-1]))

        node.create_subscription(
            Odometry, "/odom", on_odom, qos_profile_sensor_data
        )
        node.create_subscription(
            Odometry, "/ground_truth/odom", on_gt, qos_profile_sensor_data
        )
        node.create_subscription(
            LaserScan, "/scan", on_scan, qos_profile_sensor_data
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
        node.create_subscription(
            SafetyStatus,
            "/safety/status",
            lambda m: statuses.append((time.monotonic(), m.reason)),
            RELIABLE_QOS,
        )

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
        statuses.clear()
        scan_times.clear()
        first_after = len(odoms)
        load_before = read_host_load()

        # Frames in flight when the reset answers still show the old pose
        # (SIM-3730), so the anchor is the frame nearest the spawn among the
        # next SPAWN_WINDOW; the mapper's inputs begin there (D10).
        _spin_until(
            node, 30.0, lambda: len(odoms) >= first_after + SPAWN_WINDOW
        )
        window = odoms[first_after : first_after + SPAWN_WINDOW]
        assert window, "no /odom after the reset"
        nearest = min(
            range(len(window)),
            key=lambda i: math.hypot(
                window[i][1] - SPAWN_MAP[0], window[i][2] - SPAWN_MAP[1]
            ),
        )
        anchor = first_after + nearest
        start_pose = odoms[anchor][1:]
        anchored[0] = anchor

        # L2: release only once /scan has been steady; a latch before the
        # release is K29's, reported as that and never retried here.
        settle_start = time.monotonic()
        while True:
            _spin_until(node, 0.05)
            now = time.monotonic()
            if observer.first_latch is not None or any(r for _, r in statuses):
                pytest.fail(
                    "latch BEFORE the release (reason "
                    f"{[r for _, r in statuses if r][:1]}), commands zero:"
                    " the stale-input stall of known problem 29 (or 30 if"
                    f" /scan is slow): {observer.verdict()}"
                    f"{observer._startup_trace()}"
                )
            if mapping.quiet_for(scan_times, now, SETTLE_S, SETTLE_GAP_S):
                break
            if now - settle_start > SETTLE_CAP_S:
                pytest.fail(
                    f"/scan was not steady (a gap under {SETTLE_GAP_S} s for"
                    f" {SETTLE_S} s) within {SETTLE_CAP_S:.0f} s of the"
                    " reset: the sim is degraded (known problem 30)"
                )
        scans.clear()
        anchored[0] = len(odoms)
        scan_count[0] = 0
        anchor = len(odoms) - 1
        start_pose = odoms[anchor][1:]
        statuses.clear()
        _release_hold(node)
        released = time.monotonic()
        release_index = len(scan_times)

        began = released
        lap_done_at = None
        lap_wall = None
        while time.monotonic() - began < BUDGET_S:
            rclpy.spin_once(node, timeout_sec=0.02)
            if lap_done_at is None and counters and counters[-1].lap_count >= 1:
                lap_done_at = len(odoms)
                lap_wall = time.monotonic() - began
            if lap_done_at is not None and metrics:
                after = np.array(odoms[lap_done_at:])
                if (
                    len(after) > 1
                    and np.hypot(*np.diff(after[:, 1:3], axis=0).T).sum()
                    >= EXTRA_TRAVEL_M
                ):
                    break
        load = host_load_delta(load_before, read_host_load())
        stale = observer.stale_count
        saved = None
        if lap_done_at is not None:
            saved = subprocess.run(
                [
                    "ros2",
                    "run",
                    "nav2_map_server",
                    "map_saver_cli",
                    "-f",
                    str(out / "Porto"),
                    "--fmt",
                    "png",
                    "--occ",
                    "0.65",
                    "--free",
                    "0.196",
                    "--ros-args",
                    "-p",
                    "use_sim_time:=false",
                    "-p",
                    "save_map_timeout:=60.0",
                    "-p",
                    "map_subscribe_transient_local:=true",
                ],
                capture_output=True,
                text=True,
                timeout=120,
            )
    finally:
        if observer is not None:
            observer.close()
        node.destroy_node()
        reset_rclpy()
        stop(launched, 60.0)

    run_times = np.array(scan_times[release_index:])
    scan_gaps_ms = np.diff(run_times) * 1000.0 if len(run_times) > 2 else [0.0]
    rows = np.array([s[1] for s in scans])
    poses = rows[:, 1:] if len(rows) else np.zeros((0, 3))
    ranges = np.array([s[0].ranges for s in scans], dtype=float)
    angle_min = scans[0][0].angle_min if scans else 0.0
    angle_increment = scans[0][0].angle_increment if scans else 0.0
    range_max = scans[0][0].range_max if scans else 0.0
    np.savez(
        out / "porto_map_run.npz",
        poses=poses,
        scans=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_max=range_max,
        odom=np.array(odoms[anchor:]),
        gt=np.array(gts),
    )
    m = metrics[-1] if metrics else None
    sim_collisions = counters[-1].collisions if counters else -1
    reasons = sorted({r for _, r in statuses})
    result = {
        "start_pose": [round(float(v), 4) for v in start_pose],
        "lap_time_sim": counters[-1].last_lap_time if counters else None,
        "lap_wall_s": lap_wall,
        "sim_lap_count": counters[-1].lap_count if counters else 0,
        "collisions_sim": sim_collisions,
        "stale_input": stale,
        "status_reasons_after_release": reasons,
        "host_load": load,
        "scan_gap_ms": {
            "median": float(np.median(scan_gaps_ms)),
            "max": float(np.max(scan_gaps_ms)),
        },
        "git_sha": subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=REPO,
            capture_output=True,
            text=True,
        ).stdout.strip(),
        "controller_defaults": json.loads(
            subprocess.run(
                [
                    sys.executable,
                    "-c",
                    "import dataclasses, json;"
                    "from racing_controller_follow_the_gap.gap import "
                    "Parameters;"
                    "print(json.dumps(dataclasses.asdict(Parameters())))",
                ],
                capture_output=True,
                text=True,
            ).stdout
            or "{}"
        ),
        "loop_closure": None,
        "shape": None,
        "agreement": None,
    }
    laser_x = _laser_x()
    if saved is not None and saved.returncode == 0 and len(scans) > 50:
        produced = load_grid(out / "Porto.yaml")
        truth = load_grid(REPO / TRUTH)
        closure = mapping.loop_closure(
            poses, ranges, angle_min, angle_increment, laser_x, range_max
        )
        result["loop_closure"] = {
            "dx": closure.dx,
            "dy": closure.dy,
            "dyaw": closure.dyaw,
        }
        shape = mapping.shape_after_fit(
            produced,
            truth,
            poses,
            ranges,
            angle_min,
            angle_increment,
            laser_x,
            range_max,
        )
        result["shape"] = {
            "median": shape.median,
            "p95": shape.p95,
            "residual": shape.residual,
            "fit": [
                shape.correction.dx,
                shape.correction.dy,
                shape.correction.dyaw,
            ],
            "raw_median": shape.raw.median,
            "raw_p95": shape.raw.p95,
            "raw_signed_median": shape.raw.signed_median,
            "wall_cells": shape.raw.wall_cell_count,
        }
        if reference:
            other = map_to_wall(produced, load_grid(Path(reference)))
            result["agreement"] = {
                "median": other.median,
                "p95": other.p95,
                "reference": reference,
            }
        yaml_path = out / "Porto.yaml"
        body = yaml_path.read_text(encoding="utf-8")
        yaml_path.write_text(_provenance(result) + body, encoding="utf-8")
    (out / "mapping.json").write_text(json.dumps(result, indent=2))
    print("MAPPING", json.dumps(result))

    assert stale == 0, f"STALE_INPUT x{stale} after the release"
    assert SafetyStatus.REASON_TRACK_LIMIT not in reasons, (
        "TRACK_LIMIT after the reset: the car left the official corridor"
        f" (reasons {reasons})"
    )
    assert reasons == [] or reasons == [0], reasons
    assert m is not None, "metrics never published"
    assert lap_done_at is not None, (
        f"sim lap_count {counters[-1].lap_count if counters else 0} < 1"
    )
    assert sim_collisions == 0, sim_collisions
    assert saved is not None and saved.returncode == 0, (
        saved.stderr[-400:] if saved else "no map saved"
    )
    assert result["loop_closure"] is not None, "too few scans recorded"
    drift = math.hypot(closure.dx, closure.dy)
    assert drift <= LOOP_CLOSURE_M, f"loop closure {drift:.3f} m"
    assert abs(closure.dyaw) <= LOOP_CLOSURE_RAD, closure.dyaw
    assert shape.raw.wall_cell_count >= WALL_CELLS_MIN, shape.raw
    assert shape.median <= SHAPE_MEDIAN_M, result["shape"]
    assert shape.p95 <= SHAPE_P95_M, result["shape"]
    assert shape.residual <= FIT_RESIDUAL_M, result["shape"]
    if reference:
        assert result["agreement"]["median"] <= AGREEMENT_MEDIAN_M, result
        assert result["agreement"]["p95"] <= AGREEMENT_P95_M, result
