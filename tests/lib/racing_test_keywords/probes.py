"""Optional probes that ``run_scenario(probes=...)`` attaches to its driver.

A probe is a thin rclpy subscriber around pure logic that lives in
``scripts.pose_source_report`` and ``racing_evaluation.map_accuracy`` (both
tier-1 tested). The wiring here has no tier-1 test; SIM-5510's structural
check fails a completed row whose probe heard nothing.

Lifecycle, driven by ``run_scenario``: ``attach(node)`` once the driver node
exists, ``started()`` the instant the deterministic t=0 reset returns,
``close()`` in the ``finally`` (always, before the node is destroyed). The
caller owns the probe objects and reads their results after the run returns.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Protocol

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

TF_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
# The sim publishes /scan and /clock best effort (gotcha #6).
SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
POSE_PARENT, POSE_CHILD = "map", "odom"


class Probe(Protocol):
    def attach(self, node) -> None: ...

    def started(self) -> None: ...

    def close(self) -> None: ...


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class _Probe:
    """Subscription bookkeeping shared by the probes."""

    def __init__(self) -> None:
        self._node = None
        self._subscriptions: list = []

    def _subscribe(self, *arguments) -> None:
        self._subscriptions.append(self._node.create_subscription(*arguments))

    def close(self) -> None:
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._subscriptions = []


class RateProbe(_Probe):
    """map -> odom (changed value) and TF rate, scan rate, sim duration.

    Samples count from ``started()`` to ``close()`` in sim time (/clock).
    """

    def __init__(self) -> None:
        super().__init__()
        self.samples: list[tuple[float, tuple[float, ...]]] = []
        self.scans = 0
        self._clock = 0.0
        self._t0: float | None = None
        self._t1 = 0.0

    def attach(self, node) -> None:
        self._node = node
        self._subscribe(Clock, "/clock", self._on_clock, SENSOR_QOS)
        self._subscribe(TFMessage, "/tf", self._on_tf, TF_QOS)
        self._subscribe(LaserScan, "/scan", self._on_scan, SENSOR_QOS)

    def started(self) -> None:
        self._t0 = self._clock

    def close(self) -> None:
        self._t1 = self._clock
        super().close()

    def _on_clock(self, message) -> None:
        self._clock = _seconds(message.clock)

    def _on_tf(self, message) -> None:
        if self._t0 is None:
            return
        for transform in message.transforms:
            if (
                transform.header.frame_id == POSE_PARENT
                and transform.child_frame_id == POSE_CHILD
            ):
                t = transform.transform.translation
                q = transform.transform.rotation
                self.samples.append(
                    (
                        _seconds(transform.header.stamp),
                        (t.x, t.y, q.z, q.w),
                    )
                )

    def _on_scan(self, _message) -> None:
        if self._t0 is not None:
            self.scans += 1

    @property
    def duration_s(self) -> float:
        return 0.0 if self._t0 is None else max(self._t1 - self._t0, 0.0)

    def results(self) -> dict:
        from scripts.pose_source_report import pose_rate

        rates = pose_rate(self.samples, self.duration_s)
        rates["scan_rate_hz"] = (
            self.scans / self.duration_s if self.duration_s > 0 else None
        )
        return rates


class ProcessCpuProbe(_Probe):
    """CPU ticks of the one process whose cmdline holds ``pattern``.

    Needs the scan count of the same window (``RateProbe.scans``) to give
    CPU ms per scan. A pattern that matches zero or several processes is
    recorded in ``error``, never raised: the run's data must survive it.
    """

    def __init__(self, pattern: str, proc: Path = Path("/proc")) -> None:
        super().__init__()
        self.pattern = pattern
        self.error: str | None = None
        self.ticks_before: int | None = None
        self.ticks_after: int | None = None
        self._proc = proc
        self._pid: int | None = None

    def attach(self, node) -> None:
        self._node = node

    def _ticks(self) -> int | None:
        from scripts.pose_source_report import parse_proc_stat

        try:
            return parse_proc_stat(
                (self._proc / str(self._pid) / "stat").read_text()
            )
        except OSError as error:
            self.error = f"{self.pattern}: {error}"
            return None

    def started(self) -> None:
        from scripts.pose_source_report import single_match

        cmdlines = {}
        for entry in self._proc.iterdir():
            if entry.name.isdigit():
                try:
                    cmdlines[int(entry.name)] = (
                        (entry / "cmdline")
                        .read_bytes()
                        .decode(errors="replace")
                    )
                except OSError:
                    continue
        try:
            self._pid = single_match(cmdlines, self.pattern)
        except ValueError as error:
            self.error = str(error)
            return
        self.ticks_before = self._ticks()

    def close(self) -> None:
        if self._pid is not None:
            self.ticks_after = self._ticks()

    def cpu_ms_per_scan(self, scans: int) -> float | None:
        from scripts.pose_source_report import cpu_ms_per_scan

        if self.ticks_before is None or self.ticks_after is None:
            return None
        return cpu_ms_per_scan(
            self.ticks_before,
            self.ticks_after,
            os.sysconf("SC_CLK_TCK"),
            scans,
        )


class LastMapProbe(_Probe):
    """The last ``/map`` (transient local, so a late join still hears it)."""

    def __init__(self) -> None:
        super().__init__()
        self.message: OccupancyGrid | None = None

    def attach(self, node) -> None:
        self._node = node
        self._subscribe(OccupancyGrid, "/map", self._on_map, MAP_QOS)

    def started(self) -> None:
        pass

    def _on_map(self, message) -> None:
        self.message = message

    def grid(self):
        """The last map as a ``map_accuracy.Grid``; None if none arrived."""
        from racing_evaluation.map_accuracy import grid_from_occupancy

        message = self.message
        if message is None:
            return None
        info = message.info
        return grid_from_occupancy(
            list(message.data),
            info.width,
            info.height,
            info.resolution,
            (info.origin.position.x, info.origin.position.y),
        )
