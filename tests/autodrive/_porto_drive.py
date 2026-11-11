"""The live Porto driver shared by SIM-3730 and SIM-3750 (#44).

Not a test. Launches the stack against the AutoDRIVE sim, takes the t = 0
reset (gotcha 19), drives `laps` laps and returns what was observed; each test
judges it and writes its own JSON. Extracted from SIM-3730 unchanged in
behaviour (its asserts and `qualification.json` keys stay the same).
"""

import gzip
import json
import math
import os
import subprocess
import time
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from conftest import stop
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from racing_interfaces.msg import (
    SafetyStatus,
    ScenarioMetrics,
    SimCounters,
    TrackRelativeState,
)
from racing_interfaces.srv import Reset
from racing_test_keywords.ros_helpers import (
    ensure_rclpy_initialized,
    reset_rclpy,
    wait_for_endpoints,
    wait_for_message,
    wait_for_node,
)
from racing_test_keywords.scenario_runner import (
    ESTIMATE_TOPIC,
    METRICS_QOS,
    PARTICIPANT_NODES,
    POSE_SOURCE_NODES,
    RELIABLE_QOS,
    _HangObserver,
    host_load_delta,
    read_host_load,
)
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_srvs.srv import Trigger

REPO = Path(__file__).resolve().parents[2]
STARTUP_S = 120.0
FRAME_WAIT_S = 60.0
ESTIMATOR_WAIT_S = 120.0
# The fixture's spawn CoG through porto_frame.yaml (its header): the first
# pose after the reset must be here, within 0.05 m / 0.05 rad. Closes the
# spawn-y topic (2026-10-30-autodrive-spawn-y-offset).
SPAWN_MAP = (-5.8454, 1.9713, 0.0028)
SPAWN_WINDOW = 100  # frames, 0.6 s
SPAWN_TOL_M = 0.05
SPAWN_TOL_RAD = 0.05
# The held start (A6): the supervisor's own services, and what the hold waits
# for. Each wait has a deadline and fails naming the pose, speed and reason.
ESTOP_SERVICE = "/racing_safety_supervisor/emergency_stop"
CLEAR_SERVICE = "/racing_safety_supervisor/clear_emergency_stop"
REST_SPEED = 0.05  # m/s
HOLD_WAIT_S = 30.0
RELEASE_WAIT_S = 1.0


# The channels arrive at the sim's frame rate (about 200 Hz on an idle host),
# several topics per frame, and `spin_once` handles one message per call, so a
# keep-last queue of 10 would drop rows silently. Deep queues, the endpoints'
# own reliability (a reliable subscriber never hears a best-effort publisher,
# gotcha 6).
_DEPTH = 5000
DEEP_SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=_DEPTH,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
DEEP_STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=_DEPTH,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


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


def hold_error(reasons: Counter, stale_count: int) -> str | None:
    """What the hold got wrong, or None; checked before the release.

    The hold must have seen the supervisor's emergency stop latch
    (`REASON_ESTOP_LATCHED`) and no STALE_INPUT. `clear_emergency_stop`
    clears a STALE latch too (supervisor.cpp), so a release must never do
    that silently.
    """
    if stale_count:
        return f"STALE_INPUT x{stale_count} during the hold: {dict(reasons)}"
    if reasons.get("REASON_STALE_INPUT", 0):
        return f"REASON_STALE_INPUT seen during the hold: {dict(reasons)}"
    if not reasons.get("REASON_ESTOP_LATCHED", 0):
        return (
            f"REASON_ESTOP_LATCHED never seen during the hold: {dict(reasons)}"
        )
    return None


def released_reasons(names: list[str]) -> Counter:
    """The reasons received after the clear response, minus the leading run
    of `REASON_ESTOP_LATCHED`: statuses published before the clear and still
    queued, which are not a latch of the drive. A STALE right at the release
    followed by its latch keeps that latch."""
    start = 0
    while start < len(names) and names[start] == "REASON_ESTOP_LATCHED":
        start += 1
    return Counter(names[start:])


def _subscribe_channels(node, rows: list[dict]) -> None:
    """The lateral channels, one `rows` entry per message. The topics are the
    server's (contract_node.py); `/drive` is the stack's command."""

    def on_imu(m):
        a = m.linear_acceleration
        rows.append(
            {
                "stream": "imu",
                "t": _seconds(m.header.stamp),
                "ax": a.x,
                "ay": a.y,
                "az": a.z,
                "wz": m.angular_velocity.z,
            }
        )

    def on_drive(m):
        rows.append(
            {
                "stream": "drive",
                "t": _seconds(m.header.stamp),
                "steer": m.drive.steering_angle,
                "speed": m.drive.speed,
            }
        )

    def on_encoders(m):
        rows.append(
            {
                "stream": "enc",
                "t": _seconds(m.header.stamp),
                "left": m.position[0],
                "right": m.position[1],
            }
        )

    def on_track(m):
        rows.append(
            {
                "stream": "track",
                "t": _seconds(m.header.stamp),
                "s": m.s,
                "d": m.d,
                "heading_error": m.heading_error,
            }
        )

    node.create_subscription(Imu, "/imu", on_imu, DEEP_SENSOR_QOS)
    node.create_subscription(
        AckermannDriveStamped, "/drive", on_drive, DEEP_STATE_QOS
    )
    node.create_subscription(
        JointState,
        "/racing_sim/wheel_encoders",
        on_encoders,
        DEEP_SENSOR_QOS,
    )
    node.create_subscription(
        TrackRelativeState,
        "/ground_truth/track_relative_state",
        on_track,
        DEEP_STATE_QOS,
    )


def _write_channels(
    path: Path, meta: dict, rows: list[dict], counters: list
) -> None:
    """The header, the streams of `_subscribe_channels`, and the sim's own
    counters (every message `drive_porto` kept since the reset)."""
    lines = [{"meta": meta}, *rows]
    for c in counters:
        lines.append(
            {
                "stream": "counters",
                "t": _seconds(c.header.stamp),
                "lap_count": c.lap_count,
                "lap_time": c.lap_time,
                "collisions": c.collisions,
            }
        )
    with gzip.open(path, "wt", encoding="utf-8") as handle:
        for line in lines:
            handle.write(json.dumps(line) + "\n")


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
    # The server's last `racing_sim: stamps` status (Step 5.0), only with
    # `channels`; None with channels off, or from a server before 5.0.
    stamp_status: dict | None = None
    # Only with `held_start`: what the hold got wrong (`hold_error`; None is
    # a clean hold) and the wall seconds from the emergency stop to the
    # release.
    hold_error: str | None = None
    hold_s: float | None = None

    @property
    def periods(self) -> np.ndarray:
        return np.diff(self.stamps)


def estimator_waits(pose_source: str) -> tuple[tuple[str, str, str], ...]:
    """What to wait for before the t = 0 reset, in order: ("node", name,
    topic), ("subscription", name, topic) and ("message", "", topic).

    The table is the gym runner's `POSE_SOURCE_NODES`
    (`scenario_runner._reset_at_deterministic_t0`, repo-gotcha #19): an
    estimator that publishes after the drive stream latches the supervisor.
    Ground truth waits for nothing. Only amcl also waits for its first
    estimate: the support node withholds MOLA's until MOLA has applied
    /initialpose, and slam_toolbox emits only after motion; both need the sim
    clock to advance, which on AutoDRIVE (free-running) it does anyway, so
    the wait would only ever time out for them.
    """
    if pose_source == "ground_truth":
        return ()
    if pose_source not in POSE_SOURCE_NODES:
        raise ValueError(
            f"unknown pose_source {pose_source!r}; expected ground_truth or"
            f" one of {sorted(POSE_SOURCE_NODES)}"
        )
    estimator, scan_topic = POSE_SOURCE_NODES[pose_source]
    waits = (
        ("node", estimator, scan_topic),
        ("subscription", estimator, scan_topic),
    )
    if pose_source == "amcl":
        waits += (("message", "", ESTIMATE_TOPIC),)
    return waits


def wait_for_estimator(node, pose_source: str, timeout: float) -> None:
    """Run `estimator_waits` on one shared deadline (not a fresh budget per
    wait), the `scenario_runner` pattern."""
    deadline = time.monotonic() + timeout

    def remaining() -> float:
        return max(deadline - time.monotonic(), 0.0)

    for kind, name, topic in estimator_waits(pose_source):
        if kind == "node":
            wait_for_node(node, name, (topic,), remaining())
        elif kind == "subscription":
            wait_for_endpoints(node, name, {topic}, "subscription", remaining())
        else:
            wait_for_message(node, Odometry, topic, RELIABLE_QOS, remaining())


def drive_porto(
    launch: list[str],
    laps: int,
    out: Path,
    *,
    budget_s: float,
    host_command: str,
    node_name: str,
    channels: bool = False,
    meta: dict | None = None,
    pose_source: str = "ground_truth",
    recording_path: Path | None = None,
    held_start: Callable[[], Callable[[], bool]] | None = None,
    after_laps: Callable[[], None] | None = None,
) -> PortoRun:
    """Drive `launch` for `laps` laps and return what was seen.

    `pose_source` names the estimator `launch` runs on (`ground_truth`, the
    default, is SIM-3730/3750's unchanged behaviour). Any other waits, after
    the first frame and before the reset, for the estimator node, its /scan
    subscription and (amcl) its first estimate on one shared deadline
    (`estimator_waits`). `recording_path` (None: the launch's default) is
    appended to `launch` as `recording_path:=`, so the run's recording, which
    scores the live localization error offline, lands where the caller reads
    it.

    With `channels` (off by default: SIM-3730/3750 never pass it) the lateral
    channels of part 1L's corner sweep are also written to
    `out/channels.jsonl.gz`, beside `porto_run.npz`, which is the same in
    both modes: a header line `{"meta": ...}` (`meta` as given), then one
    line per message, `{"stream": ..., "t": ..., ...}`. `t` is the message's
    own `header.stamp` (the server's receive time, one value per sim frame,
    which joins a frame's streams; never the subscriber's receive time).

    `held_start` (None: nothing, SIM-3730/3750/3770/3780/3781's behaviour,
    byte for byte) is a held start through the supervisor's existing
    services, the real car's procedure (park, latch the stop, start the
    mapper, wait for it, release). After the first frame and the estimator
    waits: `~/emergency_stop`; wait until the truth speed is under
    `REST_SPEED`; the t = 0 reset; wait until a post-reset frame is at
    `SPAWN_MAP` and at rest; call `held_start()`, which starts the mapper and
    RETURNS a readiness predicate (it raises its own AssertionError at its
    own deadline); spin until that is true, polling it once a second; check
    `hold_error` (a STALE latch must never be cleared silently);
    `~/clear_emergency_stop`; drive. The node is spun for the whole hold, on
    deep queues, so a STALE during the mapper's start-up is seen, not
    overwritten by the latch's own statuses. `PortoRun.reasons` then counts
    from the first status after the clear that is not the stop's latch
    (`released_reasons`); `stale` stays the whole run's count. `budget_s`
    counts from the release, but the sim's own lap time (`laps_sim_s`)
    includes the hold (`hold_s`).

    `after_laps` runs when the drive ends, with the launch still up (a mapper
    is saved here). It defaults to nothing.
    """
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain is None or domain == "42":
        pytest.fail("ROS_DOMAIN_ID must be set and not 42 (run the wrapper)")
    out.mkdir(parents=True, exist_ok=True)
    if recording_path is not None:
        launch = [*launch, f"recording_path:={recording_path}"]
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
    release_observer = None  # held start: latch/trace from the release
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
        names: list[str] = []  # held start: every reason, in arrival order
        counters: list[SimCounters] = []
        metrics: list[ScenarioMetrics] = []
        scans: list[tuple[LaserScan, tuple]] = []
        scan_count = [0]
        rows: list[dict] = []  # the channels, in arrival order

        def on_odom(m):
            p = m.pose.pose
            tw = m.twist.twist.linear
            speeds.append(math.hypot(tw.x, tw.y))
            if channels:
                rows.append(
                    {
                        "stream": "truth",
                        "t": _seconds(m.header.stamp),
                        "vx": tw.x,
                        "vy": tw.y,
                    }
                )
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
            Odometry,
            "/ground_truth/odom",
            on_odom,
            DEEP_SENSOR_QOS if channels else qos_profile_sensor_data,
        )
        node.create_subscription(
            LaserScan,
            "/ground_truth/scan",
            on_scan,
            qos_profile_sensor_data,
        )

        def on_status(s):
            name = _reason_name(s.reason)
            reasons.update([name])
            if held_start is not None:
                names.append(name)

        node.create_subscription(
            SafetyStatus,
            "/safety/status",
            on_status,
            # /safety/status is KEEP_LAST 10 at 100 Hz on both ends: a hold
            # that is not spun on a deep queue loses a STALE to the latch.
            DEEP_STATE_QOS if held_start is not None else RELIABLE_QOS,
        )
        node.create_subscription(
            SimCounters,
            "/racing_sim/autodrive_counters",
            counters.append,
            DEEP_STATE_QOS if channels else RELIABLE_QOS,
        )
        node.create_subscription(
            ScenarioMetrics, "/scenario/metrics", metrics.append, METRICS_QOS
        )
        stamp_status: list[dict] = []
        if channels:
            _subscribe_channels(node, rows)

            def on_stamp_status(m):
                if m.name == "racing_sim: stamps":
                    stamp_status[:] = [{v.key: v.value for v in m.values}]

            node.create_subscription(
                DiagnosticStatus,
                "/racing_sim/stamp_status",
                on_stamp_status,
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
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

        wait_for_estimator(node, pose_source, ESTIMATOR_WAIT_S)

        def where() -> str:
            pose = odoms[-1][1:] if odoms else None
            speed = speeds[-1] if speeds else None
            last = names[-1] if names else None
            return f"pose {pose}, speed {speed}, last reason {last}"

        def spin_until(done: Callable[[], bool], what: str) -> None:
            end = time.monotonic() + HOLD_WAIT_S
            while not done():
                if time.monotonic() > end:
                    pytest.fail(f"held start: {what} ({where()})")
                rclpy.spin_once(node, timeout_sec=0.02)

        def call_trigger(name: str):
            trigger = node.create_client(Trigger, name)
            assert trigger.wait_for_service(timeout_sec=30.0), name
            answer = trigger.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, answer, timeout_sec=30.0)
            assert answer.done() and answer.result().success, name
            return answer

        if held_start is not None:
            hold_began = time.monotonic()
            call_trigger(ESTOP_SERVICE)
            seen = len(odoms)
            spin_until(
                lambda: len(odoms) > seen and speeds[-1] < REST_SPEED,
                f"the car never came to rest after the emergency stop"
                f" (under {REST_SPEED} m/s)",
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
        released_at = 0
        hold_s = None
        hold_problem = None
        if held_start is not None:

            def parked_at_spawn() -> bool:
                if len(odoms) <= first_after or speeds[-1] >= REST_SPEED:
                    return False
                _, x, y, yaw = odoms[-1]
                return (
                    math.hypot(x - SPAWN_MAP[0], y - SPAWN_MAP[1])
                    <= SPAWN_TOL_M
                    and abs(_wrap(yaw - SPAWN_MAP[2])) <= SPAWN_TOL_RAD
                )

            spin_until(
                parked_at_spawn,
                f"no post-reset frame at rest at the spawn {SPAWN_MAP}",
            )
            ready = held_start()
            polled = 0.0
            mapper_ready = False
            while not mapper_ready:
                rclpy.spin_once(node, timeout_sec=0.02)
                if time.monotonic() - polled >= 1.0:
                    polled = time.monotonic()
                    mapper_ready = ready()
            hold_problem = hold_error(Counter(names), observer.stale_count)
            release_observer = _HangObserver(node)
            call_trigger(CLEAR_SERVICE)
            hold_s = time.monotonic() - hold_began
            released_at = len(names)
            end = time.monotonic() + RELEASE_WAIT_S
            while all(n == "REASON_ESTOP_LATCHED" for n in names[released_at:]):
                if time.monotonic() > end:
                    pytest.fail(
                        "held start: no status after the release other than"
                        f" the stop's latch ({where()})"
                    )
                rclpy.spin_once(node, timeout_sec=0.02)
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
        if after_laps is not None:
            after_laps()
        stale = observer.stale_count
        judged = release_observer or observer
        latch = judged.first_latch
        trace = judged._startup_trace()
        if held_start is not None:
            reasons = released_reasons(names[released_at:])
    finally:
        if observer is not None:
            observer.close()
        if release_observer is not None:
            release_observer.close()
        node.destroy_node()
        reset_rclpy()
        stop(launched, 60.0)
    wall_s = time.monotonic() - began
    if channels:
        _write_channels(
            out / "channels.jsonl.gz",
            {
                **(meta or {}),
                "stamp_status": stamp_status[0] if stamp_status else None,
            },
            rows,
            counters,
        )
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
        stamp_status=(stamp_status[0] if stamp_status else None)
        if channels
        else None,
        hold_error=hold_problem,
        hold_s=hold_s,
    )
