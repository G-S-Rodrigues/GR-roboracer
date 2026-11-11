"""Serve the map a run uses on /map, for a recording whose run has none.

usage: python3 scripts/observer_map.py --scenario <scenario.yaml>
           --pose-source ground_truth|mola [--dry-run]

RViz's Map display wants /map (Transient Local, Reliable). The stack publishes
it only under amcl (map_server) and slam_toolbox; on ground truth, and on MOLA
if it publishes none, a video would show no map. This starts a nav2
map_server and its lifecycle manager beside RViz, outside the run's launch
(the run's graph is untouched), on the yaml the scenario's pose source uses,
and checks that /map matches it. It stays in the foreground until either
child exits. Exit codes: 2 a map file is missing, 3 /map already has a
publisher (or the pose source publishes its own), 4 /map does not match.

Needs the sourced ROS overlay (it imports racing_* packages and rclpy).
"""

from __future__ import annotations

import argparse
import contextlib
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

EXIT_MISSING = 2
EXIT_REFUSED = 3
EXIT_MISMATCH = 4
NODE = "observer_map_server"
MANAGER = "observer_map_lifecycle"
MAP_WAIT_S = 30.0


class Refused(Exception):
    """The run publishes /map itself; the observer must not."""


def map_yaml(scenario, pose_source):
    """The map_server yaml the run's `pose_source` is scored on."""
    if pose_source in ("amcl", "slam_toolbox"):
        raise Refused(
            f"pose_source {pose_source} publish /map itself; start nothing"
        )
    from racing_sim_gym_jax.scenario import load_scenario

    loaded = load_scenario(Path(scenario))
    base = loaded.map_directory / loaded.map_name
    if pose_source == "ground_truth":
        yaml_path = base / f"{loaded.map_name}.yaml"
    elif pose_source == "mola":
        from racing_bringup.estimators import mola_map_files

        slam = base / "slam" / f"{loaded.map_name}.yaml"
        mola_map_files(str(slam))  # raises FileNotFoundError, naming it
        yaml_path = base / "mola" / f"{loaded.map_name}_grid.yaml"
    else:
        raise ValueError(f"unsupported pose source: {pose_source}")
    if not yaml_path.is_file():
        raise FileNotFoundError(f"no map yaml {yaml_path}")
    return str(yaml_path.resolve())


def argvs(yaml_path):
    """The map_server and lifecycle_manager command lines."""
    server = [
        "ros2",
        "run",
        "nav2_map_server",
        "map_server",
        "--ros-args",
        "-r",
        f"__node:={NODE}",
        "-p",
        f"yaml_filename:={yaml_path}",
        "-p",
        "use_sim_time:=false",
    ]
    manager = [
        "ros2",
        "run",
        "nav2_lifecycle_manager",
        "lifecycle_manager",
        "--ros-args",
        "-r",
        f"__node:={MANAGER}",
        "-p",
        f"node_names:=[{NODE}]",
        "-p",
        "autostart:=true",
        "-p",
        "use_sim_time:=false",
    ]
    return [server, manager]


def refusal(publishers):
    """A message when /map already has publishers, else None."""
    if not publishers:
        return None
    return "/map already has a publisher: " + ", ".join(publishers)


def grid_matches(msg, yaml_path):
    """(True, "") when `msg` is the grid of `yaml_path`, else (False, why)."""
    from racing_evaluation.map_accuracy import load_grid

    grid = load_grid(Path(yaml_path))
    rows, cols = grid.occupied.shape
    info = msg.info
    problems = []
    if msg.header.frame_id != "map":
        problems.append(f"frame_id {msg.header.frame_id!r}, want 'map'")
    if (info.width, info.height) != (cols, rows):
        problems.append(f"size {info.width}x{info.height}, want {cols}x{rows}")
    if abs(info.resolution - grid.resolution) > 1e-6:
        problems.append(f"resolution {info.resolution}, want {grid.resolution}")
    origin = (info.origin.position.x, info.origin.position.y)
    if max(abs(a - b) for a, b in zip(origin, grid.origin, strict=True)) > 1e-3:
        problems.append(f"origin {origin}, want {grid.origin}")
    return (not problems, "; ".join(problems))


def _stop(procs):
    for proc in procs:
        if proc.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(proc.pid, signal.SIGTERM)
    deadline = time.monotonic() + 5.0
    for proc in procs:
        try:
            proc.wait(max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def supervise(commands, on_started=None):
    """Run the commands as children (own sessions); return when one exits.

    Either child exiting ends the helper non-zero and stops the other.
    `on_started(procs)` may return an exit code to stop early.
    """
    procs = [subprocess.Popen(c, start_new_session=True) for c in commands]
    signalled = []

    def handler(signum, _frame):
        signalled.append(signum)

    old = {
        sig: signal.signal(sig, handler)
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        if on_started is not None:
            early = on_started(procs)
            if early:
                return early
        while not signalled:
            for proc in procs:
                if proc.poll() is not None:
                    return proc.returncode or 1
            time.sleep(0.1)
        return 0
    finally:
        _stop(procs)
        for sig, previous in old.items():
            signal.signal(sig, previous)


def _publishers_of_map():
    import rclpy

    rclpy.init()
    node = rclpy.create_node("observer_map_probe")
    try:
        time.sleep(2.0)  # discovery
        return [
            f"{i.node_namespace.rstrip('/')}/{i.node_name}"
            for i in node.get_publishers_info_by_topic("/map")
        ]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _own_map(yaml_path):
    """Wait for /map (Transient Local, Reliable) and check it."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("observer_map_check")
    try:
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        got = []
        node.create_subscription(OccupancyGrid, "/map", got.append, qos)
        deadline = time.monotonic() + MAP_WAIT_S
        while not got and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not got:
            return None, f"no /map within {MAP_WAIT_S:.0f} s"
        ok, reason = grid_matches(got[0], yaml_path)
        return (got[0] if ok else None), reason
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument(
        "--pose-source",
        required=True,
        choices=("ground_truth", "mola", "amcl", "slam_toolbox"),
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        yaml_path = map_yaml(args.scenario, args.pose_source)
    except Refused as error:
        print(f"observer map: refused: {error}", file=sys.stderr)
        return EXIT_REFUSED
    except FileNotFoundError as error:
        print(f"observer map: {error}", file=sys.stderr)
        return EXIT_MISSING
    commands = argvs(yaml_path)
    if args.dry_run:
        print(f"yaml: {yaml_path}")
        for command in commands:
            print(" ".join(command))
        return 0
    message = refusal(_publishers_of_map())
    if message:
        print(f"observer map: refused: {message}", file=sys.stderr)
        return EXIT_REFUSED

    def check(_procs):
        msg, reason = _own_map(yaml_path)
        if msg is None:
            print(f"observer map: {yaml_path} mismatch: {reason}", flush=True)
            return EXIT_MISMATCH
        info = msg.info
        print(
            f"observer map: {yaml_path} {info.width}x{info.height} "
            f"{info.resolution} m origin ({info.origin.position.x}, "
            f"{info.origin.position.y}) ok",
            flush=True,
        )
        return 0

    return supervise(commands, on_started=check)


if __name__ == "__main__":
    raise SystemExit(main())
