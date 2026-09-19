"""Run the real ``racing_bringup`` launch headlessly and read back its metrics.

Shared by tier-3 system tests and tier-4 Robot Framework acceptance tests: both
need "launch the stack, run one seeded scenario to completion, read
``/scenario/metrics`` back" and neither should reimplement that wiring
separately from the other.

The graph is driven through the *real* launch file
(``racing_bringup/launch/sim_pure_pursuit.launch.py``), the same one
``ros2 launch`` and ``scripts/check.sh`` use, so a passing test proves the
composed system a human runs, not a parallel one built only for tests.

``launch.LaunchService.run()`` must be called from the thread that owns it
(see ``launch_testing/test_runner.py``'s own comment on this — running it on a
background thread is unsupported). So this module inverts the tier-2
``launch_testing`` arrangement: the *scenario driver* runs on a background
thread, spins its own rclpy node to wait for ``/scenario/metrics``, then calls
``LaunchService.shutdown()`` to make the foreground ``run()`` call return.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

import launch
import launch.actions
import launch.launch_description_sources
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racing_interfaces.msg import SafetyStatus, ScenarioMetrics
from racing_interfaces.srv import Reset
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from .ros_helpers import (
    ensure_rclpy_initialized,
    reset_rclpy,
    wait_for_message,
    wait_for_node,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
BRINGUP_LAUNCH_FILE = (
    REPOSITORY_ROOT
    / "ros_ws"
    / "src"
    / "racing_bringup"
    / "launch"
    / "sim_pure_pursuit.launch.py"
)
METRICS_TOPIC = "/scenario/metrics"
METRICS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ODOMETRY_TOPIC = "/odom"
DRIVE_TOPIC = "/drive"
SAFETY_STATUS_TOPIC = "/safety/status"
RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

RESET_SERVICE = "/racing_sim/reset"
# The seven nodes `sim_pure_pursuit.launch.py` always starts (excludes
# rviz2, gated off by use_rviz:=false here).
PARTICIPANT_NODES = (
    "racing_sim",
    "racing_bringup_support",
    "racing_controller_baseline",
    "racing_safety_supervisor",
    "racing_metrics",
    "racing_recording",
    "robot_state_publisher",
)
# `racing_metrics` only starts accumulating once these best-effort/reliable
# subscriptions have delivered something (scripts/compare_metrics.py's
# comment on minimum_wall_clearance) - the clearance nadir sits in the
# opening ticks, so these must be live *before* the deterministic reset
# below, or the nadir sample is missed regardless of t=0 being repeatable.
METRICS_ACCUMULATION_TOPICS = {
    "/scan",
    SAFETY_STATUS_TOPIC,
    "/ground_truth/track_relative_state",
}

# ADR 0006: `time_scale` decouples simulated time from wall time, so a
# budget reasoned in wall-clock lap durations measured on this host (the
# previous version of this constant) no longer holds once time_scale != 1.
# The budget is simulated lap length over time_scale, with a margin that
# absorbs process startup and JIT warm-up (~4-5s measured on this host)
# plus real host load. At the default time_scale=1.0, on the golden
# analytic_circle lap (22.85s, tests/golden/baseline.json):
# 22.85 / 1.0 * 4.0 ~= 91.4s - consistent with the 90s this constant carried
# before, which was already validated as sufficient under real host load
# (a 60s budget hit its ceiling once, running last in a `check.sh --full`
# pass immediately after a full colcon build and every other tier).
GOLDEN_LAP_TIME_SECONDS = 22.85
DEFAULT_TIME_SCALE = 1.0
TIMEOUT_MARGIN = 4.0
DEFAULT_TIMEOUT_SECONDS = (
    GOLDEN_LAP_TIME_SECONDS / DEFAULT_TIME_SCALE * TIMEOUT_MARGIN
)

_METRICS_FIELDS = (
    "source",
    "scenario_id",
    "seed",
    "timestep_ratio",
    "lap_completed",
    "lap_time",
    "collision_count",
    "minimum_wall_clearance",
    "maximum_tracking_error",
    "p95_tracking_error",
    "control_saturation_events",
    "code_revision",
    "container_image_digest",
    "track_version",
    "vehicle_parameter_version",
)


def metrics_to_dict(message: ScenarioMetrics) -> dict[str, Any]:
    return {field: getattr(message, field) for field in _METRICS_FIELDS}


def _reset_at_deterministic_t0(node, seed: int, timeout: float) -> None:
    """Wait for every participant, then release the held sim with a reset.

    t=0 is otherwise wherever the DDS discovery race happens to land, and
    `minimum_wall_clearance`'s nadir sits in the opening ticks
    (`scripts/compare_metrics.py`). The sim must be launched with
    `start_held:=true`: resetting a sim that is already running rewinds a
    /clock every consumer has followed, and racing_metrics aborts on the
    non-monotonic stamp.
    """
    for participant in PARTICIPANT_NODES:
        wait_for_node(node, participant, METRICS_ACCUMULATION_TOPICS, timeout)

    client = node.create_client(Reset, RESET_SERVICE)
    if not client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError(f"{RESET_SERVICE} unavailable within {timeout}s")
    future = client.call_async(Reset.Request(seed=seed, scenario_id=""))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise TimeoutError(f"{RESET_SERVICE} timed out")
    response = future.result()
    if not response.success:
        raise RuntimeError(f"{RESET_SERVICE} failed: {response.message}")
    node.destroy_client(client)


def run_scenario(
    seed: int,
    scenario: str | None = None,
    timeout: float = DEFAULT_TIMEOUT_SECONDS,
) -> dict[str, Any]:
    """Launch the full stack headlessly, run a seeded scenario, return metrics.

    Blocks until ``/scenario/metrics`` is published or ``timeout`` elapses,
    then tears the launch down either way. Raises ``TimeoutError`` on timeout.
    """
    ensure_rclpy_initialized()

    # Held until _reset_at_deterministic_t0 releases it, so nothing moves
    # or accumulates before every participant is discovered.
    launch_arguments = {
        "seed": str(seed),
        "use_rviz": "false",
        "time_scale": str(DEFAULT_TIME_SCALE),
        "start_held": "true",
    }
    if scenario is not None:
        launch_arguments["scenario"] = scenario

    description = launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    str(BRINGUP_LAUNCH_FILE)
                ),
                launch_arguments=launch_arguments.items(),
            )
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)

    outcome: dict[str, Any] = {}
    failure: list[BaseException] = []

    def _drive() -> None:
        node = rclpy.create_node("racing_test_keywords_scenario_runner")
        try:
            _reset_at_deterministic_t0(node, seed, timeout)
            message = wait_for_message(
                node, ScenarioMetrics, METRICS_TOPIC, METRICS_QOS, timeout
            )
            outcome.update(metrics_to_dict(message))
        except BaseException as error:  # noqa: BLE001 - re-raised on the caller's thread
            failure.append(error)
        finally:
            node.destroy_node()
            service.shutdown()

    driver = threading.Thread(target=_drive, daemon=True)
    driver.start()
    service.run()
    driver.join(timeout=10.0)
    reset_rclpy()

    if failure:
        raise failure[0]
    return outcome


def run_scenario_with_track_limit_violation(
    seed: int,
    scenario: str | None = None,
    settle_timeout: float = DEFAULT_TIMEOUT_SECONDS,
    response_timeout: float = 10.0,
    off_track_xy: tuple[float, float] = (500.0, 500.0),
) -> dict[str, Any]:
    """Launch the full stack, then force the vehicle off-track and return the
    resulting ``/drive`` command and ``/safety/status`` fields.

    Publishes a fake ``/odom`` far outside the track directly (bypassing the
    real sim's own odometry the same way SIM-3010 bypasses the real
    controller's ``/controller/drive``), proving the supervisor's TRACK_LIMIT
    clamp stops the vehicle inside the *complete* composed graph — not just in
    the isolated-node case SIM-3010 covers.
    """
    ensure_rclpy_initialized()

    launch_arguments = {"seed": str(seed), "use_rviz": "false"}
    if scenario is not None:
        launch_arguments["scenario"] = scenario

    description = launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    str(BRINGUP_LAUNCH_FILE)
                ),
                launch_arguments=launch_arguments.items(),
            )
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)

    outcome: dict[str, Any] = {}
    failure: list[BaseException] = []

    def _drive() -> None:
        node = rclpy.create_node("racing_test_keywords_track_limit_injector")
        drive_messages: list[AckermannDriveStamped] = []
        status_messages: list[SafetyStatus] = []
        try:
            wait_for_node(
                node,
                "racing_safety_supervisor",
                {ODOMETRY_TOPIC, DRIVE_TOPIC, SAFETY_STATUS_TOPIC},
                settle_timeout,
            )
            publisher = node.create_publisher(
                Odometry, ODOMETRY_TOPIC, RELIABLE_QOS
            )
            fake_odometry = Odometry()
            fake_odometry.header.frame_id = "map"
            fake_odometry.child_frame_id = "base_link"
            fake_odometry.pose.pose.position.x = off_track_xy[0]
            fake_odometry.pose.pose.position.y = off_track_xy[1]
            fake_odometry.pose.pose.orientation.w = 1.0

            drive_sub = node.create_subscription(
                AckermannDriveStamped,
                DRIVE_TOPIC,
                drive_messages.append,
                RELIABLE_QOS,
            )
            status_sub = node.create_subscription(
                SafetyStatus,
                SAFETY_STATUS_TOPIC,
                status_messages.append,
                RELIABLE_QOS,
            )
            try:
                deadline = time.monotonic() + response_timeout
                while time.monotonic() < deadline:
                    fake_odometry.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(fake_odometry)
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if (
                        status_messages
                        and status_messages[-1].active_clamps
                        & SafetyStatus.CLAMP_TRACK_LIMIT
                        and drive_messages
                        and drive_messages[-1].drive.speed == 0.0
                    ):
                        break
                if not (drive_messages and status_messages):
                    raise TimeoutError(
                        f"no {DRIVE_TOPIC} or {SAFETY_STATUS_TOPIC} observed "
                        f"within {response_timeout}s"
                    )
                outcome["active_clamps"] = status_messages[-1].active_clamps
                outcome["reason"] = status_messages[-1].reason
                outcome["speed"] = drive_messages[-1].drive.speed
                outcome["steering_angle"] = drive_messages[
                    -1
                ].drive.steering_angle
            finally:
                node.destroy_subscription(drive_sub)
                node.destroy_subscription(status_sub)
        except BaseException as error:  # noqa: BLE001
            failure.append(error)
        finally:
            node.destroy_node()
            service.shutdown()

    driver = threading.Thread(target=_drive, daemon=True)
    driver.start()
    service.run()
    driver.join(timeout=10.0)
    reset_rclpy()

    if failure:
        raise failure[0]
    return outcome
