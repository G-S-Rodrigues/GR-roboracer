"""ROS-free core of front end B, the official devkit bridge (part 9).

Three pieces the ROS shell (`devkit_node.py`) wraps:

- `DevkitAssembler` gathers the devkit's 13 per-frame topics into frames.
- `DevkitCommands` turns a session `Step` into the three command values.
- `make_session` builds a `BridgeSession` that is receipt-timed only.

Frames are paired by DDS `publication_sequence_number`, never by a time
window. Step 1 measured the devkit on the real container (3 runs x 616
frames, recorded and lockstep pace): no sequence gap on any of the 13
topics, `source_timestamp` and the sequence populated on all, the sequence
identical across topics for a frame, `source_timestamp` spread within a frame
p100 7.2 ms against consecutive-frame gaps down to 4.2 ms. A window wide
enough for the spread (2 x 7.2 ms) is wider than the gap, so it cannot tell
frames apart; the sequence number can. The spread and the header stamps stay
as sanity checks with a loose bound: a frame that fails only them is dropped
and counted (`stamp_dropped`), with no resync.

Offsets between a topic's sequence and the lidar's are learned, not assumed:
at (re)sync, from bursts whose 13 heads are all present, and the assembler is
synced once two consecutive bursts give identical offsets. A per-topic
sequence gap is a loss: that frame is dropped from every topic and counted,
and nothing is ever paired across frames.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass

from racing_common import CartesianPose as Pose2D

from racing_sim_autodrive import bridge, core

__all__ = [
    "ALLOWED",
    "REQUIRED",
    "RESTRICTED",
    "SKEW_S",
    "AssembledFrame",
    "DevkitAssembler",
    "DevkitCommands",
    "make_session",
    "shutdown_commands",
]

_LOG = logging.getLogger(__name__)

# The 13 per-frame topic suffixes under /autodrive/roboracer_1/, in the order
# the devkit publishes them.
REQUIRED = (
    "throttle",
    "steering",
    "left_encoder",
    "right_encoder",
    "ips",
    "imu",
    "odom",
    "lidar",
    "lap_count",
    "lap_time",
    "last_lap_time",
    "best_lap_time",
    "collision_count",
)
# The 2026 guide restricts these 7 of the 13 (plus `reset_command`, which B
# publishes, and `/tf`, which B never subscribes).
RESTRICTED = (
    "ips",
    "odom",
    "lap_count",
    "lap_time",
    "last_lap_time",
    "best_lap_time",
    "collision_count",
)
ALLOWED = tuple(t for t in REQUIRED if t not in RESTRICTED)
HEADERED = ("lidar", "imu", "left_encoder", "right_encoder", "odom")

# Sanity bound on a frame's `source_timestamp` spread and header-stamp spread;
# the sequence number, not the stamps, separates frames. Measured on the
# stand-in's bursts (2026-10-31, 3299 frames unloaded, 3298 under `taskset -c
# 0,1` plus 4 busy loops): source spread max 13.6 ms (p99 2.9) unloaded, max
# 36.7 ms (p99 18.8) loaded bar one 276 ms outlier; header spread max 11.9 /
# 26.2 ms. 75 ms is 2x the loaded max (5.5x the unloaded) and below a burst
# gap, so only a publisher stalled for a whole frame is dropped.
SKEW_S = 0.075

# An incomplete backlog this deep means a topic is gone: resync.
_MAX_FRAMES = 64
_LIDAR = "lidar"
# What `_build` returns for a frame it rejects: stamps only, or the data.
_STAMPS = "stamps"
_DATA = "data"


@dataclass(frozen=True)
class AssembledFrame:
    sample: core.DevkitSample
    stamp_ns: int  # the lidar header stamp: the receipt stamp (E2)


class DevkitAssembler:
    def __init__(self) -> None:
        self.resyncs = 0
        self.emitted = 0
        self.synced_at: int | None = None  # lidar sequence of the sync
        self.dropped_frames = 0
        self.learn_dropped = 0  # bursts given up while learning
        self.learn_stamp_dropped = 0  # of those, by the stamp checks
        self.stamp_dropped = 0  # of those, by the stamp checks alone
        self.lost = dict.fromkeys(REQUIRED, 0)
        self.first_seen: dict[str, int] = {}  # a topic's first sequence
        self._in_error = False
        self._reset_state()

    def _reset_state(self) -> None:
        self._synced = False
        self._offsets: dict[str, int] | None = None  # relative to lidar
        self._candidate: dict[str, int] | None = None
        self._pending = {t: deque() for t in REQUIRED}  # learning phase
        self._frames: dict[int, dict[str, tuple]] = {}
        self._dead: set[int] = set()
        self._last: dict[str, int] = {}

    def _resync(self, why: str) -> None:
        self.resyncs += 1
        if not self._in_error:
            self._in_error = True
            _LOG.error("devkit frame assembler resyncing: %s", why)
        self._reset_state()

    def offer(self, *args, **kwargs) -> list[AssembledFrame]:
        out = self._offer(*args, **kwargs)
        self.emitted += len(out)
        return out

    def _offer(
        self,
        topic: str,
        value,
        source_stamp_ns: int,
        sequence: int,
        header_stamp_ns: int | None = None,
    ) -> list[AssembledFrame]:
        item = (value, source_stamp_ns, sequence, header_stamp_ns)
        self.first_seen.setdefault(topic, sequence)
        last = self._last.get(topic)
        if last is not None and sequence <= last:
            self._resync(f"{topic} sequence {sequence} after {last}")
            last = None
        self._last[topic] = sequence
        if not self._synced:
            queue = self._pending[topic]
            queue.append(item)
            if len(queue) > _MAX_FRAMES:  # a topic that never comes
                queue.popleft()
            return self._learn()
        if last is not None and sequence > last + 1:
            self._lose(topic, last + 1, sequence)
        fid = sequence - self._offsets[topic]
        if fid in self._dead:
            return self._drain()
        self._frames.setdefault(fid, {})[topic] = item
        if len(self._frames) > _MAX_FRAMES:
            self._resync("incomplete backlog")
            return []
        return self._drain()

    def _lose(self, topic: str, first: int, sequence: int) -> None:
        self.lost[topic] += sequence - first
        for seq in range(first, sequence):
            fid = seq - self._offsets[topic]
            if fid not in self._dead:
                self._dead.add(fid)
                self.dropped_frames += 1
            self._frames.pop(fid, None)

    def _drain(self) -> list[AssembledFrame]:
        out = []
        while self._synced and self._frames:
            fid = min(self._frames)
            if fid in self._dead:
                del self._frames[fid]
                continue
            parts = self._frames[fid]
            if len(parts) < len(REQUIRED):
                break
            del self._frames[fid]
            frame = self._build(parts)
            if frame == _STAMPS:
                # A publisher preempted mid-burst: the frame's identity is
                # exact by sequence, so drop it alone and carry on.
                self.dropped_frames += 1
                self.stamp_dropped += 1
                self._dead = {d for d in self._dead if d > fid}
                continue
            if isinstance(frame, str):
                self.dropped_frames += 1
                self._resync(f"frame {fid} failed the sanity checks")
                break
            self._dead = {d for d in self._dead if d > fid}
            out.append(frame)
        if out:
            self._in_error = False
        return out

    def _learn(self) -> list[AssembledFrame]:
        """Align the queue heads to one burst, then learn the offsets.

        The devkit publishes each burst from one handler, once per topic, in
        REQUIRED order (`autodrive_bridge.py`, `bridge`; Step 1's probes:
        `source_timestamp` rises along REQUIRED within every burst and the
        next burst's `throttle` is later than the last topic of this one,
        0 violations in 1848 bursts). So burst k is exactly the messages
        whose `source_timestamp` lies in [throttle_k, throttle_k+1): heads
        before the `throttle` head belong to an earlier burst, and a topic
        whose head is at or after the next `throttle` has no message in this
        one. Once the next `throttle` is queued the burst is verified
        whole; before that it is trusted only when its offsets equal those
        of the verified burst just before it.
        """
        out = []
        first = REQUIRED[0]
        rest = REQUIRED[1:]
        pending = self._pending
        while pending[first]:
            t0 = pending[first][0][1]
            for t in rest:
                queue = pending[t]
                while queue and queue[0][1] < t0:
                    queue.popleft()  # an earlier burst's message
            if not all(pending[t] for t in rest):
                break
            closed = len(pending[first]) > 1
            if closed:
                t1 = pending[first][1][1]
                if any(pending[t][0][1] >= t1 for t in rest):
                    pending[first].popleft()  # a topic missed this burst
                    self.learn_dropped += 1
                    self._candidate = None
                    continue
            base = pending[_LIDAR][0][2]
            offsets = {t: pending[t][0][2] - base for t in REQUIRED}
            if not closed and offsets != self._candidate:
                break  # cannot yet tell it from the next burst
            parts = {t: pending[t].popleft() for t in REQUIRED}
            frame = self._build(parts)
            if frame == _DATA:
                self.learn_dropped += 1
                self._candidate = None  # not a clean burst: start over
                continue
            # A burst that fails only the stamp checks has exact sequences,
            # so it still confirms the offsets; it just is not emitted.
            stamps_bad = frame == _STAMPS
            if stamps_bad:
                self.learn_stamp_dropped += 1
            if self._candidate != offsets:
                self._candidate = offsets
                continue
            self._offsets = offsets
            self._synced = True
            self.synced_at = base
            self._candidate = None
            for t in REQUIRED:  # whatever queued behind the burst
                for it in pending[t]:
                    fid = it[2] - offsets[t]
                    self._frames.setdefault(fid, {})[t] = it
                pending[t].clear()
            if stamps_bad:
                self.dropped_frames += 1
                self.stamp_dropped += 1
            else:
                out.append(frame)
            out.extend(self._drain())
            self._in_error = False
            break
        return out

    @staticmethod
    def _build(parts: dict[str, tuple]) -> AssembledFrame | str:
        sources = [p[1] for p in parts.values()]
        if (max(sources) - min(sources)) * 1e-9 > SKEW_S:
            return _STAMPS
        heads = [parts[t][3] for t in HEADERED]
        if (max(heads) - min(heads)) * 1e-9 > SKEW_S:
            return _STAMPS
        v = {t: parts[t][0] for t in REQUIRED}
        ips = tuple(v["ips"])
        position, velocity = v["odom"]
        if ips != tuple(position):
            return _DATA
        orientation, angular, linear = v["imu"]
        ranges, scan_time = v["lidar"]
        sample = core.DevkitSample(
            throttle=v["throttle"],
            steering=v["steering"],
            left_encoder=v["left_encoder"],
            right_encoder=v["right_encoder"],
            ips=ips,
            imu_orientation=tuple(orientation),
            imu_angular_velocity=tuple(angular),
            imu_linear_acceleration=tuple(linear),
            linear_velocity=tuple(velocity),
            ranges=ranges,
            scan_time=scan_time,
            lap_count=v["lap_count"],
            lap_time=v["lap_time"],
            last_lap_time=v["last_lap_time"],
            best_lap_time=v["best_lap_time"],
            collisions=v["collision_count"],
        )
        return AssembledFrame(sample, parts[_LIDAR][3])


class DevkitCommands:
    """Step -> (throttle, steering, reset_or_None); the reset hold is E7 (b).

    A reset the step fires publishes True once and is held until the session
    reports the reset's effect seen (`awaiting_reset` back to False), then
    False once. A reset fired while holding is the same hold.
    """

    def __init__(self) -> None:
        self.pending_reset = False

    def from_step(self, step, awaiting_reset: bool):
        reset = None
        if step.reset and not self.pending_reset:
            self.pending_reset = True
            reset = True
        elif self.pending_reset and not awaiting_reset:
            self.pending_reset = False
            reset = False
        return step.throttle, step.steering, reset


def shutdown_commands(pending_reset: bool):
    """Zero throttle and steering, and False if a reset True is held."""
    return 0.0, 0.0, (False if pending_reset else None)


def make_session(
    vehicle: bridge.VehicleValues,
    world_to_map: Pose2D,
    track,
    dt_source: str = "receipt",
) -> bridge.BridgeSession:
    """B's session: dt from receipt stamps only (owner D4, ruling E2)."""
    if dt_source != "receipt":
        raise ValueError(
            f"front end B refuses dt_source={dt_source!r}: owner D4 allows"
            " only receipt timing under the official bridge"
        )
    return bridge.BridgeSession(
        vehicle, world_to_map, track, dt_source="receipt"
    )
