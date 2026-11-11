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

import contextlib
import json
import sys
import threading
import time
from collections import deque
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import launch
import launch.actions
import launch.event_handlers
import launch.launch_description_sources
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from racing_interfaces.msg import (
    SafetyStatus,
    ScenarioMetrics,
    TrackRelativeState,
)
from racing_interfaces.srv import Reset
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock

from .probes import Probe
from .ros_helpers import (
    ensure_rclpy_initialized,
    reset_rclpy,
    wait_for_endpoints,
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
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
METRICS_QOS = LATCHED_QOS

# The supervisor's pose input: ground truth, not the dead reckoning on /odom
# (racing_safety_supervisor/src/node.cpp says why).
ODOMETRY_TOPIC = "/ground_truth/odom"
DRIVE_TOPIC = "/drive"
SAFETY_STATUS_TOPIC = "/safety/status"
RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# The observer's own drive subscription is deeper than the supervisor's (10),
# so a lagging observer drops nothing it is meant to count.
CONTROLLER_DRIVE_TOPIC = "/controller/drive"
OBSERVER_DRIVE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# racing_safety_supervisor's stale_input_timeout_ms (config/vehicles/
# f1tenth_default.yaml). Read here only to classify a latch; never changed.
STALE_INPUT_WINDOW_SECONDS = 0.100

# The sim publishes /clock best effort (racing_sim_gym_jax/node.py CLOCK_QOS).
CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# The supervisor's own report that it has received a /clock message (latched,
# only under use_sim_time; racing_safety_supervisor/src/node.cpp). Its
# /clock reader is rclcpp's internal one, which no other node can observe,
# and a held sim's /clock = 0 leaves now() at 0, so an endpoint wait cannot
# say it is in step. F1 Step 1 and Step A; repo-gotchas #19, #32.
SUPERVISOR_CLOCK_TOPIC = "/racing_safety_supervisor/clock_received"
# racing_metrics' latched report of its first accepted sample (a scan and a
# state both delivered; only under use_sim_time). F-T0, repo-gotchas #19.
METRICS_STARTED_TOPIC = "/racing_metrics/started"

RESET_SERVICE = "/racing_sim/reset"
# Where an active pose source puts the pose the controller may drive on:
# the support node's composition of map -> odom with /odom (BRINGUP-1040).
ESTIMATE_TOPIC = "/localization/odom"
# The eight nodes `sim_pure_pursuit.launch.py` always starts (excludes
# rviz2, gated off by use_rviz:=false here).
PARTICIPANT_NODES = (
    "racing_sim",
    "racing_bringup_support",
    "racing_controller_baseline",
    "racing_safety_supervisor",
    "racing_metrics",
    "racing_evaluation",
    "racing_recording",
    "robot_state_publisher",
)
# The node each non-default `pose_source` adds, and the topic it has to
# have subscribed to before t=0 is released. An estimator that starts
# publishing after the drive stream does is repo-gotcha #19's latch, and
# the fix is this deterministic t=0, never a longer stale-input timeout.
# nav2_amcl subscribes /scan only once the lifecycle manager has activated
# it, which is after map_server has served the grid: waiting on that
# subscription is waiting on the whole AMCL side being up.
POSE_SOURCE_NODES = {
    "slam_toolbox": ("slam_toolbox", "/scan"),
    "amcl": ("amcl", "/scan"),
    "mola": ("mola_bridge_ros2", "/scan"),
}

# `racing_metrics` only starts accumulating once these best-effort/reliable
# subscriptions have delivered something (scripts/compare_metrics.py's
# comment on minimum_wall_clearance) - the clearance nadir sits in the
# opening ticks, so these must be live *before* the deterministic reset
# below, or the nadir sample is missed regardless of t=0 being repeatable.
METRICS_ACCUMULATION_TOPICS = {
    "/ground_truth/scan",
    SAFETY_STATUS_TOPIC,
    "/ground_truth/track_relative_state",
}
# The subset of METRICS_ACCUMULATION_TOPICS that racing_sim itself
# publishes (/safety/status comes from racing_safety_supervisor instead).
# Defined as a difference so it can never drift from
# METRICS_ACCUMULATION_TOPICS the way a hand-copied literal did (gotcha
# #17: /scan is sensor noise, /ground_truth/scan is what racing_metrics
# actually accumulates).
SIM_PUBLISHED_ACCUMULATION_TOPICS = METRICS_ACCUMULATION_TOPICS - {
    SAFETY_STATUS_TOPIC
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


def timeout_for_time_scale(time_scale: float) -> float:
    """The wall-clock budget for a run at ``time_scale``: simulated lap
    length over the scale, plus ``TIMEOUT_MARGIN``'s margin for process
    startup, JIT warm-up and real host load (see the comment above)."""
    return GOLDEN_LAP_TIME_SECONDS / time_scale * TIMEOUT_MARGIN


DEFAULT_TIMEOUT_SECONDS = timeout_for_time_scale(DEFAULT_TIME_SCALE)

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


class RunOutcome(dict):
    """``run_scenario``'s metrics, plus what the run itself learned.

    The keys are exactly ``_METRICS_FIELDS``, so ``compare_metrics``, the
    goldens, the benchmark's run JSON and a report's ``numbers`` are unchanged;
    the diagnostics ride as attributes. A caller holding a plain ``dict`` (a
    stub) reads them with ``getattr(outcome, name, default)``.

    ``shutdown_escalations``: ``[{"name", "signal"}]`` for each process launch
    had to SIGTERM/SIGKILL at the end. ``frozen_at``: ``(t, s, d)`` where a
    frozen-pose stop saw the pose stop, else ``None``. ``stop_context``: on
    a frozen stop, ``{"text", "latched"}`` from the hang observer. ``start``:
    ``{"metrics_start", "first_motion", "tick", "t0_waits"}``, the record
    ``start_verdict`` judged (F-T0).
    """

    shutdown_escalations: list[dict[str, str]]
    frozen_at: tuple[float, float, float] | None
    stop_context: dict[str, Any] | None
    start: dict[str, Any] | None

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.shutdown_escalations = []
        self.frozen_at = None
        self.stop_context = None
        self.start = None


class StartMissed(RuntimeError):
    """The run's start was late: a link a seeded metric reads was not matched
    at t=0 (``start_verdict``). Raised after the lap, with the numbers.

    ``outcome``: the run's ``RunOutcome`` (metrics, ``start``, ``frozen_at``,
    ``stop_context``), so a caller can still write the record beside the
    verdict instead of losing the lap it paid for."""

    outcome: RunOutcome | None = None


def start_verdict(
    metrics_start: float | None,
    first_motion: float | None,
    tick: float | None,
    truth_driven: bool,
) -> str | None:
    """Name a late start, or return ``None``.

    ``metrics-start-missed``: racing_metrics' first accepted sample is later
    than the held stamp (normal: exactly 0). ``drive-start-missed``: a
    truth-driven car first moved later than 3 ticks (normal: 1 tick, call
    #23); it is the only guard on the supervisor -> sim /drive link, which is
    not provable while held. When both hold, the metrics start is named and
    both numbers are in the text. A condition whose input is ``None`` is not
    checked.
    """
    late_metrics = metrics_start is not None and metrics_start > 0.0
    late_drive = (
        truth_driven
        and first_motion is not None
        and tick is not None
        and first_motion > 3 * tick
    )
    numbers = f"metrics_start={metrics_start} s, first_motion={first_motion} s"
    if tick is not None:
        numbers += f", tick={tick} s"
    if late_metrics:
        also = " (the car's first motion was late too)" if late_drive else ""
        return (
            "racing_metrics' first accepted sample was after the held t=0 "
            f"(verdict=metrics-start-missed): {numbers}{also}"
        )
    if late_drive:
        return (
            "the car's first motion was later than 3 ticks "
            f"(verdict=drive-start-missed): {numbers}"
        )
    return None


_ESCALATION_SIGNALS = {-9: "SIGKILL", -15: "SIGTERM"}


class _ProcessLedger:
    """What launch did to each process: started, exited with which code, and
    which it had to escalate to SIGTERM or SIGKILL at shutdown.

    Launch dispatches a ``ProcessExited`` for every process during
    ``LaunchService.shutdown()``, before ``run()`` returns, with a return code
    of -9 for one it had to SIGKILL. A diagnostic never fails a run: each
    handler catches ``Exception`` (one that raised would make launch shut a
    green run down) and records it for ``describe()``.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.started: dict[str, float] = {}
        self.exited: dict[str, tuple[int, bool]] = {}
        self.errors: list[str] = []
        self.shutting_down = False

    def actions(self) -> list[Any]:
        return [
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessStart(on_start=self._on_start)
            ),
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(on_exit=self._on_exit)
            ),
        ]

    def _on_start(self, event: Any, context: Any) -> None:
        try:
            with self._lock:
                self.started[event.process_name] = time.monotonic()
        except Exception as error:  # noqa: BLE001 - never fail a run
            self._error(error)

    def _on_exit(self, event: Any, context: Any) -> None:
        try:
            with self._lock:
                self.exited[event.process_name] = (
                    int(event.returncode),
                    self.shutting_down,
                )
        except Exception as error:  # noqa: BLE001 - never fail a run
            self._error(error)

    def _error(self, error: Exception) -> None:
        with contextlib.suppress(Exception), self._lock:
            self.errors.append(repr(error))

    @property
    def shutdown_escalations(self) -> list[dict[str, str]]:
        with self._lock:
            return [
                {"name": name, "signal": _ESCALATION_SIGNALS[code]}
                for name, (code, during_shutdown) in self.exited.items()
                if during_shutdown and code in _ESCALATION_SIGNALS
            ]

    def describe(self) -> str:
        with self._lock:
            parts = [
                f"{name} exited rc={code}"
                for name, (code, _) in self.exited.items()
            ]
            parts += [
                f"{name} alive"
                for name in self.started
                if name not in self.exited
            ]
            parts += [f"ledger error: {error}" for error in self.errors]
        return "; ".join(parts) if parts else "no process seen"


def read_host_load() -> dict[str, float]:
    """A snapshot of host CPU pressure from ``/proc`` (Linux only).

    ``steal`` and ``iowait`` are cumulative jiffies from ``/proc/stat``'s
    aggregate ``cpu`` line, ``total`` the sum of all its fields, and
    ``load1`` the one-minute load average. Two snapshots subtracted by
    ``host_load_delta`` say whether the host was starved during the run
    between them. Empty on a host without ``/proc``.
    """
    try:
        with open("/proc/stat") as stat:
            fields = [float(value) for value in stat.readline().split()[1:]]
        with open("/proc/loadavg") as loadavg:
            load1 = float(loadavg.read().split()[0])
    except (OSError, ValueError, IndexError):
        return {}
    return {
        "total": sum(fields),
        "iowait": fields[4] if len(fields) > 4 else 0.0,
        "steal": fields[7] if len(fields) > 7 else 0.0,
        "load1": load1,
    }


def host_load_delta(
    before: Mapping[str, float], after: Mapping[str, float]
) -> dict[str, float]:
    """``after - before`` as shares of the elapsed CPU jiffies, plus load."""
    if not before or not after:
        return {}
    total = max(after["total"] - before["total"], 1.0)
    return {
        "steal_share": (after["steal"] - before["steal"]) / total,
        "iowait_share": (after["iowait"] - before["iowait"]) / total,
        "load1_start": before["load1"],
        "load1_end": after["load1"],
    }


class _HangObserver:
    """Records what the stack reported, so a hung lap can say why.

    The supervisor and the metrics node log nothing, so a latched stop is
    silent: the car stops, the lap never completes and /scenario/metrics
    never publishes. Observing /safety/status and /clock on the driver node
    names the latch and the sim time reached. It changes no deadline.

    It also keeps the stamps of the ``/controller/drive`` commands and of the
    ``/safety/status`` messages around the opening of a latch, to say which
    side of the controller -> supervisor chain went quiet (``verdict``).
    Drive stamps are the odom stamps the controller answered, so they do not
    move when this observer is late. Every sample is kept to sim t <= 2 s;
    after that a one-second ring buffer, and nothing once the first non-NONE
    status has arrived, so memory is bounded. The callbacks themselves run
    for the whole lap.
    """

    KEEP_ALL_UNTIL_SECONDS = 2.0
    RING_SAMPLES = 150  # > 1 s of 100 Hz

    def __init__(self, node) -> None:
        self.status = None
        self.first_latch = None
        self.stale_count = 0
        self.clock = None
        self._statuses_before_latch = 0
        self._node = node
        self._drive_stamps: deque[float] = deque()
        self._status_stamps: deque[float] = deque()
        self._max_drive_gap = 0.0
        self._max_status_gap = 0.0
        self._last_drive: float | None = None
        self._last_status: float | None = None
        self._load_at_start = read_host_load()
        # F-T0's start record. Subscribed before the sim exists, so it hears
        # the held sample (stamp 0) and racing_metrics' latched report.
        # ``tick`` is the smallest positive state stamp (the sim stamps
        # k * control_period, so there is no literal); ``first_motion`` the
        # first post-0 stamp with v_s != 0 (normal: 1 tick, measured in
        # Step 1, call #23); ``metrics_start`` the stamp of the first sample
        # racing_metrics accepted (normal: exactly 0, the held stamp).
        self.held_seen = False
        self.tick: float | None = None
        self.first_motion: float | None = None
        self.metrics_start: float | None = None
        self._subscriptions = [
            node.create_subscription(
                SafetyStatus, SAFETY_STATUS_TOPIC, self._on_status, RELIABLE_QOS
            ),
            node.create_subscription(
                Clock, "/clock", self._on_clock, CLOCK_QOS
            ),
            node.create_subscription(
                AckermannDriveStamped,
                CONTROLLER_DRIVE_TOPIC,
                self._on_drive,
                OBSERVER_DRIVE_QOS,
            ),
            node.create_subscription(
                TrackRelativeState,
                STATE_TOPIC,
                self._on_state,
                OBSERVER_DRIVE_QOS,
            ),
            node.create_subscription(
                TrackRelativeState,
                METRICS_STARTED_TOPIC,
                self._on_started,
                LATCHED_QOS,
            ),
        ]

    @staticmethod
    def _seconds(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _record(self, stamps: deque, seconds: float) -> None:
        stamps.append(seconds)
        if seconds > self.KEEP_ALL_UNTIL_SECONDS:
            while len(stamps) > self.RING_SAMPLES:
                stamps.popleft()

    def _on_drive(self, message) -> None:
        if self.first_latch is not None:
            return
        seconds = self._seconds(message.header.stamp)
        if self._last_drive is not None:
            self._max_drive_gap = max(
                self._max_drive_gap, seconds - self._last_drive
            )
        self._last_drive = seconds
        self._record(self._drive_stamps, seconds)

    def _on_state(self, message) -> None:
        if self.first_motion is not None:
            return
        seconds = self._seconds(message.header.stamp)
        if seconds == 0.0:
            self.held_seen = True
            return
        if seconds > 0.0 and (self.tick is None or seconds < self.tick):
            self.tick = seconds
        if message.v_s != 0.0:
            self.first_motion = seconds

    def _on_started(self, message) -> None:
        if self.metrics_start is None:
            self.metrics_start = self._seconds(message.header.stamp)

    def _on_status(self, message) -> None:
        self.status = message
        if message.reason == SafetyStatus.REASON_STALE_INPUT:
            self.stale_count += 1
        if self.first_latch is not None:
            return
        seconds = self._seconds(message.header.stamp)
        if message.reason != SafetyStatus.REASON_NONE:
            self.first_latch = message
        else:
            self._statuses_before_latch += 1
        if self._last_status is not None:
            self._max_status_gap = max(
                self._max_status_gap, seconds - self._last_status
            )
        self._last_status = seconds
        self._record(self._status_stamps, seconds)

    def _on_clock(self, message) -> None:
        self.clock = message.clock

    def close(self) -> None:
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)

    @staticmethod
    def _name(prefix: str, value: int) -> str:
        for name in dir(SafetyStatus):
            if name.startswith(prefix) and getattr(SafetyStatus, name) == value:
                return name
        return str(value)

    def _describe(self, status) -> str:
        clamps = [
            name
            for name in dir(SafetyStatus)
            if name.startswith("CLAMP_")
            and name != "CLAMP_NONE"
            and status.active_clamps & getattr(SafetyStatus, name)
        ]
        stamp = status.header.stamp
        return (
            f"{self._name('REASON_', status.reason)} "
            f"clamps={clamps or ['CLAMP_NONE']} "
            f"stamp={stamp.sec + stamp.nanosec * 1e-9:.3f}s"
        )

    def verdict(self) -> str:
        """Which side of the controller -> supervisor chain went quiet.

        ``supervisor-lag``: commands stamped inside the stale window exist,
        yet the supervisor's own status stamps skip at least that window - it
        judged a queue it had not read. ``controller-silent``: the status
        stamps are continuous and no command is stamped inside the window.
        ``supervisor-clock-late``: the first status the supervisor ever
        published is already STALE_INPUT, stamped at or past the window, with
        commands inside it - its clock stood at 0 while the drive stream ran
        and then jumped (F1). With the t=0 wait on ``~/clock_received`` in
        place this means the supervisor had heard /clock before release yet
        its clock did not advance through the window. This observer's own
        subscription can also be late, so it is a verdict about the first
        status seen, not proof. Anything else is ``unclassified``.
        """
        if self.first_latch is None:
            return "no-latch"
        latch = self._seconds(self.first_latch.header.stamp)
        in_window = [
            stamp
            for stamp in self._drive_stamps
            if latch - STALE_INPUT_WINDOW_SECONDS < stamp <= latch
        ]
        status_skipped = self._max_status_gap >= STALE_INPUT_WINDOW_SECONDS
        if (
            self.first_latch.reason == SafetyStatus.REASON_STALE_INPUT
            and self._statuses_before_latch == 0
            and latch >= STALE_INPUT_WINDOW_SECONDS - 1e-9
            and in_window
        ):
            return "supervisor-clock-late"
        if in_window and status_skipped:
            return "supervisor-lag"
        if not in_window and not status_skipped:
            return "controller-silent"
        return "unclassified"

    def _startup_trace(self) -> str:
        if self.first_latch is None:
            return ""
        latch = self._seconds(self.first_latch.header.stamp)
        recent = [
            f"{stamp:.3f}"
            for stamp in self._drive_stamps
            if latch - 0.2 < stamp <= latch
        ]
        return (
            f"; verdict={self.verdict()}; "
            f"drive stamps in ({latch - 0.2:.3f}, {latch:.3f}]={recent}; "
            f"max drive gap={self._max_drive_gap:.3f}s; "
            f"max status gap={self._max_status_gap:.3f}s"
        )

    def describe(self) -> str:
        clock = (
            "none"
            if self.clock is None
            else f"{self.clock.sec + self.clock.nanosec * 1e-9:.3f}s"
        )
        last = "none" if self.status is None else self._describe(self.status)
        first = (
            "none"
            if self.first_latch is None
            else self._describe(self.first_latch)
        )
        load = host_load_delta(self._load_at_start, read_host_load())
        load_text = (
            ""
            if not load
            else "; host load: "
            + ", ".join(f"{key}={value:.4f}" for key, value in load.items())
        )
        return (
            f"last /clock={clock}; last /safety/status={last}; "
            f"first non-NONE status={first}; "
            f"REASON_STALE_INPUT messages={self.stale_count}"
            f"{self._startup_trace()}{load_text}"
        )


def _reset_at_deterministic_t0(
    node,
    seed: int,
    timeout: float,
    pose_source: str = "ground_truth",
    drive_on_estimate: bool = True,
    observer=None,
) -> dict[str, float]:
    """Wait for every participant, then release the held sim with a reset.

    t=0 is otherwise wherever the DDS discovery race happens to land, and
    `minimum_wall_clearance`'s nadir sits in the opening ticks
    (`scripts/compare_metrics.py`). The sim must be launched with
    `start_held:=true`: resetting a sim that is already running rewinds a
    /clock every consumer has followed, and racing_metrics aborts on the
    non-monotonic stamp.

    `wait_for_node` and `wait_for_endpoints` prove discovery in this node's
    own graph view, which is not delivery: in 21 of 72 launches the sim's
    new endpoints took 0.15-1.07 s to match after they were created
    (findings 2026-11-11). Delivery is proven link by link, below, each wait
    on the shared deadline (never a fresh `timeout`, never a retry):

    - L4, sim /clock -> supervisor: the supervisor's latched
      ``~/clock_received``.
    - L2, sim scan and state -> racing_metrics: its latched ``~/started``,
      the first sample it accepted (both topics delivered).
    - L1, sim odometry -> controller: one ``/controller/drive``. A held sim
      republishes its stamp-0 snapshot, so one command proves the link (and
      /trajectory with it). Only where the controller's input exists while
      held: ground_truth, amcl, or ``drive_on_estimate=False``.

    L3, supervisor /drive -> sim, is not proven by a wait. Whether anything
    crosses it while the sim is held is not established: first motion is
    1 tick where the timing model predicted 2, so a command or a non-zero
    velocity reached the first step, mechanism not established (gotcha
    #19). ``drive-start-missed`` is its only guard.

    Held-sim only. A free-running backend (AutoDRIVE, a car) has no held t=0
    and must not call this function (gotcha #19, ADR 0006).

    With an ``observer`` (the hang observer, whose readers are among the
    links that may be late) the release also waits until it has seen the
    held, stamp-0 state, else raises ``verdict=start-unobserved``: without
    that the start record below would be blind in the launches whose links
    are late.

    Returns the seconds each of the delivery waits took, by name.
    """
    deadline = time.monotonic() + timeout
    waits: dict[str, float] = {}

    def _remaining() -> float:
        return max(deadline - time.monotonic(), 0.0)

    for participant in PARTICIPANT_NODES:
        wait_for_node(
            node, participant, METRICS_ACCUMULATION_TOPICS, _remaining()
        )
    wait_for_endpoints(
        node,
        "racing_metrics",
        METRICS_ACCUMULATION_TOPICS,
        "subscription",
        _remaining(),
    )
    wait_for_endpoints(
        node,
        "racing_sim",
        SIM_PUBLISHED_ACCUMULATION_TOPICS,
        "publisher",
        _remaining(),
    )
    if pose_source in POSE_SOURCE_NODES:
        estimator, scan_topic = POSE_SOURCE_NODES[pose_source]
        wait_for_node(node, estimator, (scan_topic,), _remaining())
        wait_for_endpoints(
            node, estimator, {scan_topic}, "subscription", _remaining()
        )
        # amcl only. The sim is held, but a held sim republishes its
        # snapshot, so amcl can already produce map -> odom and the support
        # node compose an estimate; releasing t=0 before one exists lets the
        # supervisor's stale-input timeout fire (gotcha #19). slam_toolbox is
        # deliberately left with the scan-subscription wait above: a held sim
        # republishes an identical snapshot at stamp 0, and slam_toolbox,
        # which (hypothesis, untested) needs motion before it emits, never
        # produced /localization/odom while held - measured as a 394 s
        # timeout on SIM-3070, while SIM-3080 passes with this wait.
        # mola is left with the scan wait too: the support node withholds
        # the estimate until MOLA has applied /initialpose (BRINGUP-2500),
        # and MOLA drops a held sim's stamp-0 scans, so that estimate does
        # not exist until time advances.
        if pose_source == "amcl":
            wait_for_message(
                node, Odometry, ESTIMATE_TOPIC, RELIABLE_QOS, _remaining()
            )

    # A first delivery, reported by the supervisor itself: the endpoint waits
    # above prove discovery, not that its /clock reader is matched, and a
    # supervisor released before that stamps its first evaluation past the
    # stale window (gotcha #19). The wait shares the deadline; it is not a
    # retry and not a longer timeout.
    began = time.monotonic()
    try:
        wait_for_message(
            node, Clock, SUPERVISOR_CLOCK_TOPIC, LATCHED_QOS, _remaining()
        )
    except TimeoutError as error:
        raise TimeoutError(
            "racing_safety_supervisor heard no /clock before t=0 "
            f"(verdict=supervisor-clock-unheard): {error}"
        ) from error
    waits["supervisor_clock"] = time.monotonic() - began

    # The same shape for the metrics link: racing_metrics emits nothing until
    # a lap ends, so its own latched report of the first accepted sample is
    # the only proof its scan and state readers are matched.
    began = time.monotonic()
    try:
        wait_for_message(
            node,
            TrackRelativeState,
            METRICS_STARTED_TOPIC,
            LATCHED_QOS,
            _remaining(),
        )
    except TimeoutError as error:
        raise TimeoutError(
            "racing_metrics accepted no sample before t=0 "
            f"(verdict=metrics-start-unheard): {error}"
        ) from error
    waits["metrics_started"] = time.monotonic() - began

    if pose_source in ("ground_truth", "amcl") or not drive_on_estimate:
        began = time.monotonic()
        try:
            wait_for_message(
                node,
                AckermannDriveStamped,
                CONTROLLER_DRIVE_TOPIC,
                RELIABLE_QOS,
                _remaining(),
            )
        except TimeoutError as error:
            raise TimeoutError(
                "racing_controller_baseline published no /controller/drive "
                f"before t=0 (verdict=controller-drive-unheard): {error}"
            ) from error
        waits["controller_drive"] = time.monotonic() - began

    if observer is not None and not observer.held_seen:
        began = time.monotonic()
        while not observer.held_seen:
            if _remaining() <= 0.0:
                raise TimeoutError(
                    "the hang observer saw no held (stamp 0) state before "
                    "t=0 (verdict=start-unobserved)"
                )
            rclpy.spin_once(node, timeout_sec=0.01)
        waits["observer_held"] = time.monotonic() - began

    client = node.create_client(Reset, RESET_SERVICE)
    if not client.wait_for_service(timeout_sec=_remaining()):
        raise TimeoutError(f"{RESET_SERVICE} unavailable within {timeout}s")
    future = client.call_async(Reset.Request(seed=seed, scenario_id=""))
    rclpy.spin_until_future_complete(node, future, timeout_sec=_remaining())
    if not future.done() or future.result() is None:
        raise TimeoutError(f"{RESET_SERVICE} timed out")
    response = future.result()
    if not response.success:
        raise RuntimeError(f"{RESET_SERVICE} failed: {response.message}")
    node.destroy_client(client)
    return waits


FINALIZE_SERVICE = "/racing_metrics/finalize"
STATE_TOPIC = "/ground_truth/track_relative_state"


def scenario_track_length(scenario: str | None) -> float:
    """Length of the track ``racing_metrics`` laps on for ``scenario``.

    Resolved as the launch does (``Path(scenario)`` against the process's
    working directory, ``track_parameters``), so a frozen-pose stop wraps s on
    the same track the lap is counted on (gotcha 15). Raises on ``None``."""
    import racing_common
    from racing_bringup.scenario_parameters import track_parameters

    track = track_parameters(Path(scenario))["racing_metrics"]["track_path"]
    return racing_common.Track.from_yaml(str(track)).length()


def _wait_for_metrics_or_frozen_pose(
    node: Any, timeout: float, window_s: float, track_length: float
) -> tuple[Any, tuple[float, float, float] | None]:
    """``wait_for_message`` for ``/scenario/metrics`` that also ends a run
    whose pose froze.

    Once the pose (s and d) has stood still for ``window_s`` of sim time, the
    metrics node is asked to publish once (``~/finalize``, ``lap_completed``
    false) and the wait goes on for that message until the original deadline.
    Returns ``(message, frozen_at)``: ``frozen_at`` is ``(t, s, d)`` where the
    pose stopped, ``None`` when the metrics came with no freeze. Raises the
    same ``TimeoutError`` ``wait_for_message`` does, so a caller that
    classifies by its text is unchanged. Imported lazily: tier 4 runs
    ``robot --pythonpath tests/lib``, where ``scripts`` is not importable
    until the repository root is put on the path.
    """
    if str(REPOSITORY_ROOT) not in sys.path:
        sys.path.insert(0, str(REPOSITORY_ROOT))
    from std_srvs.srv import Trigger

    from scripts.frozen_pose import FrozenPose

    pose = FrozenPose(window_s, track_length)
    messages: list[Any] = []
    pending: list[Any] = []

    def _on_state(message: Any) -> None:
        stamp = message.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        # Stamp 0 is the clock-0 hold; the run starts after ~/reset.
        if t > 0.0 and pose.update(t, message.s, message.d) and not pending:
            pending.append(client.call_async(Trigger.Request()))

    client = node.create_client(Trigger, FINALIZE_SERVICE)
    subscriptions = [
        node.create_subscription(
            ScenarioMetrics, METRICS_TOPIC, messages.append, METRICS_QOS
        ),
        node.create_subscription(
            TrackRelativeState, STATE_TOPIC, _on_state, RELIABLE_QOS
        ),
    ]
    deadline = time.monotonic() + timeout
    try:
        while not messages and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_client(client)
    if not messages:
        raise TimeoutError(f"no message on {METRICS_TOPIC} within {timeout}s")
    return messages[0], pose.frozen_at


def run_scenario(
    seed: int,
    scenario: str | None = None,
    timeout: float | None = None,
    time_scale: float = DEFAULT_TIME_SCALE,
    pose_source: str = "ground_truth",
    drive_on_estimate: bool = True,
    launch_arguments: Mapping[str, str] | None = None,
    *,
    stop_on_frozen_pose: float | None = None,
    track_length: float | None = None,
    probes: Sequence[Probe] = (),
) -> dict[str, Any]:
    """Launch the full stack headlessly, run a seeded scenario, return metrics.

    Blocks until ``/scenario/metrics`` is published or ``timeout`` elapses,
    then tears the launch down either way. Raises ``TimeoutError`` on timeout.

    ``time_scale`` sets how fast simulated time advances relative to wall
    clock (ADR 0006); it changes only the wall-clock rate of the run, never
    its physics or its simulated ``lap_time``. ``timeout`` defaults to
    ``timeout_for_time_scale(time_scale)`` when not given explicitly, since
    a scaled run's wall-clock budget must scale with it too - a fixed
    default sized for ``time_scale=1.0`` would starve a faster run of no
    extra margin, or give a slower one too little.

    ``pose_source`` selects which node owns ``map -> odom`` and so which
    pose the controller drives on; the default is the reference stack's
    ground truth, and the discovery wait above covers whatever it starts.

    ``drive_on_estimate`` closes the loop on it; with ``False`` the
    controller keeps ground truth and the estimator is only scored.

    ``launch_arguments`` passes further launch arguments through (e.g.
    ``recording_path``, ``estimate_topic``); it cannot override the ones
    this function owns.

    ``stop_on_frozen_pose`` (sim seconds) is opt-in: once the pose has stood
    still that long, ``/racing_metrics/finalize`` is called and the partial
    metrics (``lap_completed`` false) are returned instead of waiting out
    ``timeout``. It needs ``track_length`` (the caller's, from the track the
    scenario drives; the scenario names a map, not a track file) and raises
    ``ValueError`` without it. ``None`` leaves the wait untouched.

    ``probes`` (``racing_test_keywords.probes``) attach to the driver node
    beside the hang observer, are told when t=0 starts, and are closed at
    the end; the caller reads their results afterwards. The default ``()``
    changes nothing for any other caller.
    """
    if stop_on_frozen_pose is not None and track_length is None:
        raise ValueError("stop_on_frozen_pose needs track_length")
    if timeout is None:
        timeout = timeout_for_time_scale(time_scale)
    ensure_rclpy_initialized()

    # Held until _reset_at_deterministic_t0 releases it, so nothing moves
    # or accumulates before every participant is discovered.
    owned = {
        "seed": str(seed),
        "use_rviz": "false",
        "time_scale": str(time_scale),
        "start_held": "true",
        "pose_source": pose_source,
        "drive_on_estimate": "true" if drive_on_estimate else "false",
    }
    if scenario is not None:
        owned["scenario"] = scenario
    overlap = set(owned) & set(launch_arguments or {})
    if overlap:
        raise ValueError(f"launch_arguments may not set {sorted(overlap)}")
    launch_arguments = {**(launch_arguments or {}), **owned}

    ledger = _ProcessLedger()
    description = launch.LaunchDescription(
        [
            *ledger.actions(),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    str(BRINGUP_LAUNCH_FILE)
                ),
                launch_arguments=launch_arguments.items(),
            ),
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)

    outcome: dict[str, Any] = {}
    failure: list[BaseException] = []
    frozen: dict[str, Any] = {}
    start: dict[str, Any] = {}

    def _drive() -> None:
        node = rclpy.create_node("racing_test_keywords_scenario_runner")
        observer = _HangObserver(node)
        try:
            for probe in probes:
                probe.attach(node)
            try:
                t0_waits = _reset_at_deterministic_t0(
                    node,
                    seed,
                    timeout,
                    pose_source,
                    drive_on_estimate=drive_on_estimate,
                    observer=observer,
                )
            except TimeoutError as stall:
                # A stall names what launch saw: which process exited (and
                # with what code) and which nodes the driver could see.
                detail = ledger.describe()
                try:
                    names = ", ".join(
                        sorted(
                            f"{namespace.rstrip('/')}/{name}"
                            for name, namespace in (
                                node.get_node_names_and_namespaces()
                            )
                        )
                    )
                    names = f"[{names}]"
                except Exception:  # noqa: BLE001 - never mask the stall
                    names = "(unavailable)"
                raise TimeoutError(
                    f"{stall} [t0 stall: {detail}; nodes: {names}]"
                ) from stall
            for probe in probes:
                probe.started()
            if stop_on_frozen_pose is None:
                message = wait_for_message(
                    node, ScenarioMetrics, METRICS_TOPIC, METRICS_QOS, timeout
                )
            else:
                message, frozen_at = _wait_for_metrics_or_frozen_pose(
                    node, timeout, stop_on_frozen_pose, track_length
                )
                if frozen_at is not None:
                    # A supervisor latch freezes the pose too; the keyword
                    # must not call it a collision (raceline_report orders
                    # `latched` before `dnf` for this reason).
                    frozen["at"] = frozen_at
                    frozen["context"] = {
                        "text": observer.describe(),
                        "latched": observer.first_latch is not None,
                    }
            outcome.update(metrics_to_dict(message))
            # The start record, from observer stamps only: never lap_time,
            # which is 0 on a frozen-pose stop or a DNF and spans laps.
            start.update(
                metrics_start=observer.metrics_start,
                first_motion=observer.first_motion,
                tick=observer.tick,
                t0_waits=t0_waits or {},
            )
            print(f"t0 start: {json.dumps(start)}", file=sys.stderr, flush=True)
            verdict = start_verdict(
                observer.metrics_start,
                observer.first_motion,
                observer.tick,
                pose_source == "ground_truth" or not drive_on_estimate,
            )
            if verdict is not None:
                raise StartMissed(f"{verdict} [{json.dumps(start)}]")
        except TimeoutError as error:
            failure.append(TimeoutError(f"{error} [{observer.describe()}]"))
            failure[0].__cause__ = error
        except BaseException as error:  # noqa: BLE001 - re-raised on the caller's thread
            failure.append(error)
        finally:
            for probe in probes:
                with contextlib.suppress(BaseException):  # never mask the run
                    probe.close()
            observer.close()
            node.destroy_node()
            ledger.shutting_down = True
            service.shutdown()

    driver = threading.Thread(target=_drive, daemon=True)
    driver.start()
    service.run()
    driver.join(timeout=10.0)
    reset_rclpy()

    escalations = ledger.shutdown_escalations
    if failure:
        error = failure[0]
        if isinstance(error, StartMissed):
            missed = RunOutcome(outcome)
            missed.shutdown_escalations = escalations
            missed.frozen_at = frozen.get("at")
            missed.stop_context = frozen.get("context")
            missed.start = dict(start) or None
            error.outcome = missed
        if escalations:
            note = "shutdown escalations: " + ", ".join(
                f"{e['name']} {e['signal']}" for e in escalations
            )
            if isinstance(error, TimeoutError):
                raise TimeoutError(f"{error} [{note}]") from error
            error.add_note(note)
        raise error
    result = RunOutcome(outcome)
    result.shutdown_escalations = escalations
    result.frozen_at = frozen.get("at")
    result.stop_context = frozen.get("context")
    result.start = dict(start) or None
    return result


def score_localization(
    recording: str | Path, estimate_topic: str = ESTIMATE_TOPIC
) -> dict[str, Any]:
    """Score one run's recording against its ground truth, offline.

    The same comparator tier 3 calls directly
    (``scripts/compare_localization.py``); this wrapper exists so tier 4
    reaches it without `scripts` having to be importable from whatever
    directory ``robot`` was started in - tier 3 gets that for free from
    ``python3 -m pytest`` putting the repository root on the path, and
    ``robot --pythonpath tests/lib`` does not.
    """
    if str(REPOSITORY_ROOT) not in sys.path:
        sys.path.insert(0, str(REPOSITORY_ROOT))
    from scripts.compare_localization import (
        estimator_start,
        read_recording,
        score_recording,
    )

    events = read_recording(Path(recording))
    return {
        **score_recording(events, estimate_topic=estimate_topic),
        **estimator_start(events, estimate_topic),
    }


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

    The launch is held and injection starts only after
    ``_reset_at_deterministic_t0`` has released t=0 (repo-gotchas #19), so
    the supervisor's clock has moved before the first fake odometry. The
    outcome's ``safety_record`` is the first non-NONE ``/safety/status`` the
    stack reported, for a failure message that names a latch.
    """
    ensure_rclpy_initialized()

    # The sim is held until _reset_at_deterministic_t0 releases it (gotcha
    # #19; F1 fixed this only for run_scenario, F2 for this runner).
    launch_arguments = {
        "seed": str(seed),
        "use_rviz": "false",
        "start_held": "true",
    }
    if scenario is not None:
        launch_arguments["scenario"] = scenario

    ledger = _ProcessLedger()
    description = launch.LaunchDescription(
        [
            *ledger.actions(),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    str(BRINGUP_LAUNCH_FILE)
                ),
                launch_arguments=launch_arguments.items(),
            ),
        ]
    )
    service = launch.LaunchService()
    service.include_launch_description(description)

    outcome: dict[str, Any] = {}
    failure: list[BaseException] = []

    def _drive() -> None:
        node = rclpy.create_node("racing_test_keywords_track_limit_injector")
        observer = _HangObserver(node)
        drive_messages: list[AckermannDriveStamped] = []
        status_messages: list[SafetyStatus] = []
        try:
            wait_for_node(
                node,
                "racing_safety_supervisor",
                {ODOMETRY_TOPIC, DRIVE_TOPIC, SAFETY_STATUS_TOPIC},
                settle_timeout,
            )
            # Created before the reset so its DDS matching with the
            # supervisor's reader runs while the sim is held (gotcha #32);
            # nothing is published until t=0.
            publisher = node.create_publisher(
                Odometry, ODOMETRY_TOPIC, RELIABLE_QOS
            )
            _reset_at_deterministic_t0(node, seed, settle_timeout)
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
                outcome["safety_record"] = observer.describe()
            finally:
                node.destroy_subscription(drive_sub)
                node.destroy_subscription(status_sub)
        except TimeoutError as error:
            failure.append(TimeoutError(f"{error} [{observer.describe()}]"))
            failure[0].__cause__ = error
        except BaseException as error:  # noqa: BLE001
            failure.append(error)
        finally:
            observer.close()
            node.destroy_node()
            ledger.shutting_down = True
            service.shutdown()

    driver = threading.Thread(target=_drive, daemon=True)
    driver.start()
    service.run()
    driver.join(timeout=10.0)
    reset_rclpy()

    escalations = ledger.shutdown_escalations
    if failure:
        error = failure[0]
        if escalations:
            note = "shutdown escalations: " + ", ".join(
                f"{e['name']} {e['signal']}" for e in escalations
            )
            if isinstance(error, TimeoutError):
                raise TimeoutError(f"{error} [{note}]") from error
            error.add_note(note)
        raise error
    result = RunOutcome(outcome)
    result.shutdown_escalations = escalations
    return result
