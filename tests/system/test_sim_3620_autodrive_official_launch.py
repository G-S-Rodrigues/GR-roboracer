"""SIM-3620: `autodrive_bridge:=official` launches the whole stack against the
recorded AutoDRIVE stream through the official-bridge path, with every node on
wall time and no stale-input latch (#44 part 9).

SIM-3610's twin for front end B. The devkit and the Unity sim are not in CI,
so `racing_sim_autodrive.devkit_standin` stands in for the devkit: it
publishes the recorded frames on the devkit's 13 topics and applies the
latched commands to its replies, on the devkit's own ROS domain (S+1, S-1 if
S+1 > 101). The launch is the real one; `autodrive_devkit_domain` names that
domain. The window is SIM-3610's: frames 132..324. The recorded car drives
into a wall at frame 340; with the PP controller aligned to the track (the CW
porto.yaml) the supervisor latches COMMAND_LIMIT about frame 331, which would
fail the mid-stream check for a reason that is the recording's, not the
launch wiring's, so the window ends before it. (The recording's reset at 449
is one the stand-in cannot show.)

The stand-in runs `--no-tf` and under the production topic names. On the
devkit domain those names are global, and so is the devkit's `/tf`, which
would reach a neighbouring worker whose stack runs on S+1; SIMAD-2250 and
SIM-3720 cover the `/tf` isolation. It also refuses to run if the derived
domain is `AUTODRIVE_ROS_DOMAIN_ID`, the live devkit's.

No `/racing_sim/reset` is called: t=0 here is the first replayed frame.
"""

import contextlib
import json
import os
import signal
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
LAST_FRAME = 324  # COMMAND_LIMIT latches ~frame 331 on the recorded crash
FRAMES = LAST_FRAME + 1 - START_FRAME
STARTUP_SECONDS = 120.0
VEHICLE = "config/vehicles/autodrive_roboracer.yaml"
# How long the end-of-stream latch may take to arrive after the replay exits:
# the supervisor's timeout is 100 ms plus one timer tick.
END_LATCH_WAIT_SECONDS = 1.0
STACK_DOMAIN = int(os.environ.get("ROS_DOMAIN_ID", "0"))
DEVKIT_DOMAIN = (
    STACK_DOMAIN + 1 if STACK_DOMAIN + 1 <= 101 else STACK_DOMAIN - 1
)
LAUNCH = [
    "ros2",
    "launch",
    "racing_bringup",
    "sim_pure_pursuit.launch.py",
    "simulator:=autodrive",
    "autodrive_bridge:=official",
    f"autodrive_devkit_domain:={DEVKIT_DOMAIN}",
    f"vehicle:={VEHICLE}",
    "scenario:=config/scenarios/porto.yaml",
    "use_rviz:=false",
]


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


def test_sim_3620_official_bridge_launch_runs_on_wall_time_without_a_latch(
    tmp_path,
):
    live = os.environ.get("AUTODRIVE_ROS_DOMAIN_ID")
    assert live is None or int(live) not in (STACK_DOMAIN, DEVKIT_DOMAIN), (
        f"stack domain {STACK_DOMAIN} / devkit domain {DEVKIT_DOMAIN} equals "
        f"AUTODRIVE_ROS_DOMAIN_ID={live}: never subscribe to the live devkit"
    )
    # launch.log stays in pytest's tmp_path, so a failure keeps its log.
    launch_log = tmp_path / "launch.log"
    replies_file = tmp_path / "replies.jsonl"
    with launch_log.open("w") as log:
        launched = subprocess.Popen(
            LAUNCH,
            cwd=REPOSITORY_ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    ensure_rclpy_initialized()
    node = rclpy.create_node("sim_3620_driver")
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
                "racing_sim_autodrive.devkit_standin",
                "--fixture",
                str(FIXTURE),
                "--no-tf",
                "--wait-subscribers",
                "30",
                "--pace",
                "recorded",
                "--start-frame",
                str(START_FRAME),
                "--count",
                str(FRAMES),
                "--replies-out",
                str(replies_file),
            ],
            env={**os.environ, "ROS_DOMAIN_ID": str(DEVKIT_DOMAIN)},
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

    assert replay.returncode == 0, "stand-in stalled or failed"
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
    # B logs no ERROR of its own in a clean stream (a resync or a bad frame
    # would); the end of the stream is a WARNING stall at most.
    errors = [
        line
        for line in log_text.splitlines()
        if "racing_sim_autodrive_devkit" in line and "[ERROR]" in line
    ]
    assert errors == [], errors
