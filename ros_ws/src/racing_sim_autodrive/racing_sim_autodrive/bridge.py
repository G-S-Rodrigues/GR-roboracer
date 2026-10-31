"""ROS-free session of our Socket.IO server: a frame in, a reply and states out.

Everything between "a `Bridge` frame arrived" and "publish these, send that
back" lives here so tier 1 can replay the recording through it: decoding,
the throttle loop, the reset handshake, the CoG ground truth, dead-reckoned
odometry and the track-relative state. The node (`server_node.py`) only
stamps, publishes and emits.

No clock (ADR 0006: the shell's receipt time is the source stamp, as in the
official devkit). Which dt the throttle loop and odometry integrate over is
`dt_source`:

- `"lap_time"` (default; development under front end A): the `V1 Lap Time`
  delta (`core.sim_dt`, gotcha 35). `received_at` is ignored.
- `"receipt"` (the race-time path): the difference of consecutive
  `received_at` stamps, the node clock at receipt. No sim-clock field is read
  for dt, since a race build may not offer one (owner ruling 2026-10-28).

Either way a missing dt means feedforward plus the held integral, never a
guessed step.

Reversing: any backward wheel motion makes `core.encoder_speed` None, so
while the car backs up the throttle is feedforward plus the held integral and
`/odom` holds its pose with `vx = 0`. That is acceptable for racing (the car
does not reverse) and is not fixed here.
"""

from __future__ import annotations

import logging
import math
import threading
from collections.abc import Mapping
from dataclasses import dataclass

from racing_common import CartesianPose as Pose2D

from racing_sim_autodrive import control, core

__all__ = [
    "BridgeSession",
    "CogState",
    "DriveRequest",
    "ScanGeometry",
    "StallWatch",
    "Step",
    "TrackState",
    "VehicleValues",
    "fallback_reply",
    "handle_reset",
    "handle_step_mode",
    "integrate_odometry",
    "scan_geometry",
]

_LOG = logging.getLogger(__name__)

# How many frames after a reset reply the session waits to see the sim reset
# before it logs an ERROR and gives control back. Read off the fixture: the
# recorded sim was answered `Reset True` for 59 consecutive replies, so the
# reset takes at least that many frames to be taken seriously, and the
# handshake here detects its effect (lap clock stops or encoders fall) 11
# frames after the first True in the recording. 120 frames is about twice
# the longest recorded True run and ten times the detection, so a slow
# reset is not cut short while a reset the sim ignored (one True may not
# suffice, to be verified live) costs seconds, not a lap, of blind control.
RESET_WAIT_FRAMES = 120

DT_SOURCES = ("lap_time", "receipt")
STEP_MODE_CONTINUOUS = 0  # racing_interfaces/SetStepMode.CONTINUOUS

# The official devkit's lidar (autodrive_bridge.py:173-180).
SCAN_ANGLE_MIN = -2.35619
SCAN_ANGLE_INCREMENT = 0.004363323
SCAN_RANGE_MIN = 0.06
SCAN_RANGE_MAX = 10.0
# Mirror of racing_sim_adapter contract.hpp kLaserFrame (there is no
# shared Python source for it). SIMAD-1220 parses the header and
# CHECK-2020 holds this name to it.
SCAN_FRAME_ID = "lidar"


@dataclass(frozen=True)
class VehicleValues:
    """The car's numbers. All required: this module has no defaults."""

    lr: float  # rear axle to CoG, m
    h: float  # CoG height, m
    wheel_radius: float
    throttle_gain: float
    max_steering_angle: float
    kp: float
    ki: float


@dataclass(frozen=True)
class DriveRequest:
    """The latest `/drive`; the session is given None before the first."""

    speed: float
    steering_angle: float


@dataclass(frozen=True)
class CogState:
    """Pose of the CoG (`base_link`, z = h) and its body twist."""

    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class TrackState:
    s: float
    d: float
    heading_error: float
    v_s: float
    v_d: float


@dataclass(frozen=True)
class ScanGeometry:
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass(frozen=True)
class Step:
    frame: core.SimFrame
    reply: dict[str, str]  # always present, the lidar-less frame too
    ground_truth: CogState  # map frame
    odometry: CogState  # odom frame: dead reckoning
    track_state: TrackState | None
    speed: float | None  # encoder speed
    dt: float | None
    throttle: float  # what `reply` formats; front end B publishes these
    steering: float  # normalized, [-1, 1]
    reset: bool


class StallWatch:
    """Decides when a connected sim has gone silent (ROS-free, `now` passed in).

    The sim is never left waiting (gotcha 29) and never reconnects (gotcha
    30), so a server that stops answering must say so. `check` returns one
    report per stall: a stall is a gap longer than `timeout_s` since the last
    handled frame (or the connect), and the next frame re-arms it.
    """

    def __init__(self, timeout_s: float) -> None:
        self._timeout_s = timeout_s
        self._connected = False
        self._last = 0.0
        self._reported = False
        self.entries = 0
        self.exits = 0
        self.emits = 0

    def connected(self, now: float) -> None:
        self._connected = True
        self._last = now
        self._reported = False

    def disconnected(self) -> None:
        self._connected = False

    def entered(self) -> None:
        self.entries += 1

    def emitted(self) -> None:
        self.emits += 1

    def exited(self, now: float) -> None:
        self.exits += 1
        self._last = now
        self._reported = False

    def check(self, now: float) -> str | None:
        gap = now - self._last
        if not self._connected or self._reported or gap <= self._timeout_s:
            return None
        self._reported = True
        return (
            f"no Bridge frame handled for {gap:.2f} s: "
            f"entered={self.entries} exited={self.exits} "
            f"emitted={self.emits}"
        )


def scan_geometry(n_beams: int) -> ScanGeometry:
    """The devkit's scan layout for `n_beams` (1081 on the real car)."""
    if n_beams < 2:
        raise ValueError(f"need at least 2 beams, got {n_beams}")
    return ScanGeometry(
        frame_id=SCAN_FRAME_ID,
        angle_min=SCAN_ANGLE_MIN,
        angle_max=SCAN_ANGLE_MIN + SCAN_ANGLE_INCREMENT * (n_beams - 1),
        angle_increment=SCAN_ANGLE_INCREMENT,
        range_min=SCAN_RANGE_MIN,
        range_max=SCAN_RANGE_MAX,
    )


def integrate_odometry(
    state: CogState, speed: float, yaw_rate: float, dt: float
) -> CogState:
    """One midpoint dead-reckoning step; body twist is (speed, 0, yaw_rate)."""
    mid = state.yaw + 0.5 * yaw_rate * dt
    return CogState(
        x=state.x + speed * math.cos(mid) * dt,
        y=state.y + speed * math.sin(mid) * dt,
        z=state.z,
        yaw=math.remainder(state.yaw + yaw_rate * dt, math.tau),
        vx=speed,
        vy=0.0,
        wz=yaw_rate,
    )


def _reply(throttle: float, steering: float, reset: bool) -> dict[str, str]:
    return {
        "V1 Throttle": f"{throttle:.4f}",
        "V1 Steering": f"{steering:.4f}",
        "V1 Reset": "True" if reset else "False",
    }


def fallback_reply() -> dict[str, str]:
    """What to send for a frame that could not be decoded: the sim is never
    left waiting (gotcha 29), commands nothing and does not reset."""
    return _reply(0.0, 0.0, False)


def handle_reset(scenario_id: str, configured_id: str) -> tuple[bool, str]:
    """`~/reset`: "" or the configured scenario; the seed is meaningless
    (`initial_pose_only`)."""
    if scenario_id not in ("", configured_id):
        return False, f"unknown scenario: {scenario_id}"
    return True, "reset"


def handle_step_mode(mode: int) -> tuple[bool, str]:
    """`~/step_mode`: only CONTINUOUS; the protocol cannot be stepped."""
    if mode == STEP_MODE_CONTINUOUS:
        return True, "continuous"
    return False, f"step_mode {mode} is not supported (step_mode=false)"


def _to_map(pose: Pose2D, world_to_map: Pose2D) -> tuple[float, float, float]:
    c, s = math.cos(world_to_map.psi), math.sin(world_to_map.psi)
    return (
        c * pose.x - s * pose.y + world_to_map.x,
        s * pose.x + c * pose.y + world_to_map.y,
        math.remainder(pose.psi + world_to_map.psi, math.tau),
    )


class BridgeSession:
    def __init__(
        self,
        vehicle: VehicleValues,
        world_to_map: Pose2D,
        track,
        dt_source: str = "lap_time",
    ) -> None:
        if dt_source not in DT_SOURCES:
            raise ValueError(f"dt_source {dt_source!r} not in {DT_SOURCES}")
        self._vehicle = vehicle
        self._world_to_map = world_to_map
        self._track = track
        self._dt_source = dt_source
        self._loop = control.ThrottleLoop(
            vehicle.throttle_gain, vehicle.kp, vehicle.ki
        )
        self._latch = control.ResetLatch()
        self._reset_level = False
        self._level_lock = threading.Lock()  # service thread vs gevent thread
        self._awaiting_reset = False  # replied True, sim not yet reset
        self._effect_seen = False
        self._waited = 0  # frames since the reset reply
        self._prev: core.SimFrame | None = None
        self._prev_received: float | None = None
        self._odometry: CogState | None = None

    @property
    def integral(self) -> float:
        """The throttle loop's integral (tests, diagnostics)."""
        return self._loop.integral

    @property
    def awaiting_reset(self) -> bool:
        """A reset was sent and the sim has not yet shown it (front end B)."""
        return self._awaiting_reset

    def request_reset(self) -> None:
        """From the `~/reset` service: the next reply carries Reset True."""
        with self._level_lock:
            self._reset_level = True

    def _dt(self, frame: core.SimFrame, received_at: float | None):
        if self._dt_source == "lap_time":
            return (
                None if self._prev is None else core.sim_dt(self._prev, frame)
            )
        if received_at is None or self._prev_received is None:
            return None
        dt = received_at - self._prev_received
        return dt if dt > 0.0 else None

    def on_frame(
        self,
        raw: Mapping[str, str],
        drive: DriveRequest | None,
        received_at: float | None = None,
    ) -> Step:
        """One `Bridge` frame -> the reply and the states to publish.

        Raises `core.FrameError` on a malformed frame; the caller then sends
        `fallback_reply()`. `received_at` (node clock, s) is used only with
        `dt_source="receipt"`.
        """
        return self.on_sim_frame(core.decode_frame(raw), drive, received_at)

    def on_sim_frame(
        self,
        frame: core.SimFrame,
        drive: DriveRequest | None,
        received_at: float | None = None,
    ) -> Step:
        """One decoded frame -> the reply and the states to publish.

        The seam front end B uses: it builds the `SimFrame` from the
        devkit's ROS values (`core.frame_from_devkit`) and has no raw dict.
        """
        vehicle = self._vehicle
        prev = self._prev
        dt = self._dt(frame, received_at)
        speed = (
            None
            if prev is None
            else core.encoder_speed(prev, frame, vehicle.wheel_radius, dt)
        )

        # ground truth: rear axle -> CoG (gotcha 26) -> map frame
        cog = core.rear_axle_to_cog(core.frame_pose(frame), vehicle.lr)
        x, y, yaw = _to_map(cog, self._world_to_map)
        truth = CogState(
            x, y, vehicle.h, yaw,
            frame.linear_velocity[0], frame.linear_velocity[1],
            frame.angular_velocity[2],
        )  # fmt: skip

        # a sent reset re-anchors odometry and the loop until the sim has
        # actually reset (the lap clock stops or the encoders fall back to 0,
        # then runs again): the recorded reset segment is tens of frames long
        sim_dt = None if prev is None else core.sim_dt(prev, frame)
        encoders_fell = prev is not None and any(
            c < p
            for p, c in zip(
                prev.encoder_angles, frame.encoder_angles, strict=True
            )
        )
        reanchor = prev is None
        if self._awaiting_reset:
            reanchor = True
            self._waited += 1
            if sim_dt is None or encoders_fell:
                self._effect_seen = True
            elif self._effect_seen:
                self._awaiting_reset = False
            elif self._waited >= RESET_WAIT_FRAMES:
                _LOG.error(
                    "the sim showed no reset in %d frames after Reset True;"
                    " resuming control (one Reset True may not suffice)",
                    RESET_WAIT_FRAMES,
                )
                self._awaiting_reset = False
        if reanchor:
            self._loop.reset()
            speed = None
            self._odometry = CogState(
                truth.x, truth.y, vehicle.h, truth.yaw, 0.0, 0.0, 0.0
            )
        elif speed is not None and dt is not None:
            self._odometry = integrate_odometry(
                self._odometry, speed, truth.wz, dt
            )
        else:
            self._odometry = CogState(
                self._odometry.x, self._odometry.y, vehicle.h,
                self._odometry.yaw, speed or 0.0, 0.0, truth.wz,
            )  # fmt: skip

        command = 0.0 if drive is None else drive.speed
        throttle = self._loop.update(command, speed, dt)
        steering = (
            0.0
            if drive is None
            else control.steering_to_normalized(
                drive.steering_angle, vehicle.max_steering_angle
            )
        )

        with self._level_lock:  # read and clear together: no request lost
            level, self._reset_level = self._reset_level, False
        fire = self._latch.next(level)
        if fire:
            self._latch.next(False)  # re-arm: one True per request
            self._loop.reset()
            self._awaiting_reset = True
            self._effect_seen = False
            self._waited = 0

        track_state = None
        if self._track is not None:
            frenet = self._track.to_frenet(Pose2D(truth.x, truth.y, truth.yaw))
            track_state = TrackState(
                s=frenet.s,
                d=frenet.d,
                heading_error=frenet.heading_error,
                v_s=truth.vx * math.cos(frenet.heading_error),
                v_d=truth.vx * math.sin(frenet.heading_error),
            )

        self._prev = frame
        self._prev_received = received_at
        return Step(
            frame=frame,
            reply=_reply(throttle, steering, fire),
            ground_truth=truth,
            odometry=self._odometry,
            track_state=track_state,
            speed=speed,
            dt=dt,
            throttle=throttle,
            steering=steering,
            reset=fire,
        )
