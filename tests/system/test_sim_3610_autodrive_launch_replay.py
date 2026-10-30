"""SIM-3610: `simulator:=autodrive` launches the whole stack against the
recorded AutoDRIVE stream, with every node on wall time and no stale-input
latch (#44 part 8).

The Unity sim is not in CI, so the replay client stands in for it (the
tier-2 server tests' stand-in): `racing_sim_autodrive.replay` connects to
the launch's server once, the way the sim does (gotcha 30), and sends the
recorded frames at their recorded pace. This verifies the launch wiring and
the clock rule (ADR 0006: AutoDRIVE is free-running, so no node reads
/clock); it cannot see the image or the real sim's rate, which parts 9 and 10
own (plan D1/D2).

The window is bounded on purpose: frames 132..448 of the fixture (about
5.7 s). The recording's own reset is at frame 449 and the replay can never
show one, so a window past it would only wait out `RESET_WAIT_FRAMES` with
an ERROR (SIMAD-1190); and there is no `--loop`, whose wrap would teleport
the car. Frame 132 starts the driving segment (the fixture respawns at
frames 83 and 131 and drives to about 337; it stands from about 360).

No `/racing_sim/reset` is called: t=0 here is the first replayed frame.
"""

import contextlib
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from racing_test_keywords.ros_helpers import (
    ensure_rclpy_initialized,
    reset_rclpy,
    wait_for_endpoints,
    wait_for_node,
)
from racing_test_keywords.scenario_runner import (
    PARTICIPANT_NODES,
    _HangObserver,
    read_host_load,
)
from rclpy.qos import qos_profile_sensor_data

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
# The slice the conformance replay loops (7.1): one definition of where the
# fixture starts driving.
sys.path.append(str(REPOSITORY_ROOT / "ros_ws/src/racing_sim_autodrive/test"))
from _replay_loop import PORTO_DRIVING_LOOP  # noqa: E402

FIXTURE = (
    REPOSITORY_ROOT
    / "ros_ws/src/racing_sim_autodrive/test/fixtures/porto_stream.jsonl.gz"
)
START_FRAME = PORTO_DRIVING_LOOP[0]  # the fixture drives 132..337 (7.1)
LAST_FRAME = 448  # the recorded reset is frame 449
FRAMES = LAST_FRAME + 1 - START_FRAME
STARTUP_SECONDS = 120.0
VEHICLE = "config/vehicles/autodrive_roboracer.yaml"
# How long the end-of-stream latch may take to arrive after the replay exits:
# the supervisor's timeout is 100 ms plus one timer tick.
END_LATCH_WAIT_SECONDS = 1.0
LAUNCH = [
    "ros2",
    "launch",
    "racing_bringup",
    "sim_pure_pursuit.launch.py",
    "simulator:=autodrive",
    f"vehicle:={VEHICLE}",
    "scenario:=config/scenarios/porto.yaml",
    "use_rviz:=false",
]


def _free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def _stale_input_timeout_seconds() -> float:
    """The supervisor's timeout, read from the vehicle file (gotcha 19: it is
    never widened to cure a latch)."""
    params = yaml.safe_load((REPOSITORY_ROOT / VEHICLE).read_text())
    supervisor = params["racing_safety_supervisor"]["ros__parameters"]
    return supervisor["stale_input_timeout_ms"] / 1000.0


def _judge_stale_input(latch, odom_stamps, timeout_s, stale_count, trace):
    """None when the only STALE_INPUT is the end-of-stream latch, else why not.

    The window is anchored to the data, not to the replay process: the last
    /ground_truth/odom stamp is the last input the stack received. A latch
    stamped at or before it happened mid-stream. One stamped after it is the
    supervisor noticing that the input stopped (the commands stop with it).
    """
    if latch is None or stale_count == 0:
        return (
            f"no end-of-stream STALE_INPUT within {END_LATCH_WAIT_SECONDS} s "
            f"of the replay's exit (first non-NONE status {latch}): the "
            "supervisor is not judging the stream on wall time"
        )
    odom = sorted(odom_stamps)
    last = odom[-1]
    stamp = latch.header.stamp.sec + latch.header.stamp.nanosec * 1e-9
    if stamp > last:
        return None
    later = [i for i, t in enumerate(odom) if t > stamp]
    if later and later[0] > 0:
        index = later[0]
        gap = odom[index] - odom[index - 1]
        if gap >= timeout_s:
            return (
                f"stand-in stall, not a stack latch: the replay/server "
                f"delivered no /ground_truth/odom for {gap * 1000:.1f} ms "
                f"at index {index} (timeout {timeout_s * 1000:.0f} ms, host "
                f"load {read_host_load()['load1']}), and "
                f"STALE_INPUT x{stale_count} "
                f"latched at {stamp:.3f} inside it"
            )
    return (
        f"STALE_INPUT x{stale_count} mid-stream, {last - stamp:.3f} s before "
        f"the last /ground_truth/odom; first latch {latch}{trace}"
    )


def _use_sim_time(node_name: str) -> str:
    """`ros2 param get`, retried: a node is in the graph a moment before its
    parameter services answer."""
    deadline = time.monotonic() + 30.0
    while True:
        result = subprocess.run(
            ["ros2", "param", "get", f"/{node_name}", "use_sim_time"],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
        if result.stdout.strip() or time.monotonic() > deadline:
            return result.stdout.strip() or result.stderr.strip()
        time.sleep(1.0)


def _stop(process: subprocess.Popen) -> None:
    """Stop the whole launch group (gotcha 10: killing the launcher alone
    leaves the nodes running)."""
    with contextlib.suppress(ProcessLookupError):
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        with contextlib.suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def test_sim_3610_autodrive_launch_replay_runs_on_wall_time_without_a_latch(
    tmp_path,
):
    port = _free_port()
    # launch.log stays in pytest's tmp_path, so a failure keeps its log.
    launch_log = tmp_path / "launch.log"
    replies_file = tmp_path / "replies.jsonl"
    with launch_log.open("w") as log:
        launched = subprocess.Popen(
            [*LAUNCH, f"autodrive_port:={port}"],
            cwd=REPOSITORY_ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    ensure_rclpy_initialized()
    node = rclpy.create_node("sim_3610_driver")
    observer = None
    try:
        deadline = time.monotonic() + STARTUP_SECONDS
        for participant in PARTICIPANT_NODES:
            wait_for_node(
                node,
                participant,
                (),
                max(deadline - time.monotonic(), 1.0),
            )
        wait_for_endpoints(
            node,
            "racing_sim",
            {"/odom", "/ground_truth/odom", "/scan"},
            "publisher",
            max(deadline - time.monotonic(), 1.0),
        )
        observer = _HangObserver(node)
        odom_stamps: list[float] = []
        node.create_subscription(
            Odometry,
            "/ground_truth/odom",
            lambda m: odom_stamps.append(
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            ),
            qos_profile_sensor_data,
        )
        clocks = {name: _use_sim_time(name) for name in PARTICIPANT_NODES}

        replay = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "racing_sim_autodrive.replay",
                "--fixture",
                str(FIXTURE),
                "--url",
                f"http://127.0.0.1:{port}",
                "--pace",
                "recorded",
                "--start-frame",
                str(START_FRAME),
                "--count",
                str(FRAMES),
                "--replies-out",
                str(replies_file),
            ]
        )
        while replay.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
        # Once the frames stop so do the controller's commands, and the
        # supervisor rightly latches STALE_INPUT one timeout later. That
        # latch is the end of the stream, not a fault in it, so keep
        # spinning until it arrives instead of timing it against the
        # replay process's exit (which races it under load).
        end = time.monotonic() + END_LATCH_WAIT_SECONDS
        while observer.stale_count == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        stale = observer.stale_count
        latch = observer.first_latch
        trace = observer._startup_trace()
    finally:
        if observer is not None:
            observer.close()
        node.destroy_node()
        reset_rclpy()
        _stop(launched)
    log_text = launch_log.read_text(errors="replace")
    replies = [
        json.loads(line)["reply"]
        for line in replies_file.read_text().splitlines()
    ]

    assert replay.returncode == 0, "replay stalled or failed"
    assert len(replies) == FRAMES
    for name, value in clocks.items():
        assert value.endswith("False"), f"{name}: use_sim_time {value!r}"
    throttles = [float(r["V1 Throttle"]) for r in replies]
    assert max(throttles) > 0.0, "the controller's speed never reached a reply"
    assert odom_stamps, "no /ground_truth/odom reached the driver"
    verdict = _judge_stale_input(
        latch,
        odom_stamps,
        _stale_input_timeout_seconds(),
        stale,
        trace,
    )
    assert verdict is None, verdict
    # The disconnect at the end of the replay is logged as an ERROR by design
    # (the sim never reconnects, gotcha 30); any other server ERROR is a fault.
    errors = [
        line
        for line in log_text.splitlines()
        if "racing_sim_autodrive_server" in line
        and "[ERROR]" in line
        and "sim disconnected" not in line
    ]
    assert errors == [], errors
