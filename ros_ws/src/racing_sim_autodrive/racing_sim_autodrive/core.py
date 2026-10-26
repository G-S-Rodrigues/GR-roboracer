"""ROS-free core of the AutoDRIVE backend: frames, pose, speed.

No network code, no clock: both front ends (our Socket.IO server and the
official devkit bridge) turn what they receive into a `SimFrame` here, so
neither duplicates a conversion. Vehicle values (rear-axle offset, wheel
radius, gains) are arguments; this module has no defaults for them.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from racing_common import CartesianPose as Pose2D

from racing_sim_autodrive.lidar import decode_lidar

__all__ = [
    "FrameError",
    "Pose2D",
    "SimFrame",
    "decode_frame",
    "yaw_from_quaternion",
]

LIDAR_KEY = "V1 LIDAR Range Array"


class FrameError(ValueError):
    """A frame is missing a key the core needs, or one is malformed."""


@dataclass(frozen=True)
class SimFrame:
    """One sim state sample. Compare fields, not frames: `ranges` is an array."""

    throttle_echo: float  # V1 Throttle, as the sim reports it
    steering_echo: float  # V1 Steering, rad
    encoder_angles: tuple[float, float]  # left, right, rad, accumulated
    encoder_ticks: tuple[int, int] | None  # None from the devkit path
    rear_axle: tuple[float, float, float]  # V1 Position: NOT the CoG
    orientation: tuple[float, float, float, float]  # x y z w
    yaw: float  # from the quaternion, wrapped to (-pi, pi]
    linear_velocity: tuple[float, float, float]  # body frame, as reported
    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]
    lidar_scan_rate: float
    ranges: np.ndarray | None  # None on the headless first frame
    lap_count: int
    lap_time: float  # the sim's own clock
    last_lap_time: float  # inf before a lap
    best_lap_time: float
    collisions: int


def _wrap(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    wrapped = math.remainder(angle, math.tau)
    return math.pi if wrapped == -math.pi else wrapped


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about z) of a unit quaternion, in (-pi, pi]."""
    return math.atan2(
        2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
    )


def _text(raw: Mapping[str, str], key: str) -> str:
    try:
        return raw[key]
    except KeyError:
        raise FrameError(f"missing {key!r}") from None


def _vec(raw: Mapping[str, str], key: str, count: int) -> tuple[float, ...]:
    try:
        values = tuple(float(t) for t in _text(raw, key).split())
    except ValueError as error:
        raise FrameError(f"{key!r} is not numeric: {error}") from None
    if len(values) != count:
        raise FrameError(f"{key!r} has {len(values)} values, want {count}")
    return values


def _scalar(raw: Mapping[str, str], key: str) -> float:
    return _vec(raw, key, 1)[0]


def decode_frame(raw: Mapping[str, str]) -> SimFrame:
    """Socket.IO `Bridge` payload (all strings) -> SimFrame.

    Only the lidar key may be absent (the first frame of a headless sim has
    none, gotcha 29). Any other missing key raises FrameError; extra keys
    (camera, Euler angles) are ignored.
    """
    quaternion = _vec(raw, "V1 Orientation Quaternion", 4)
    ticks = _vec(raw, "V1 Encoder Ticks", 2)
    angles = _vec(raw, "V1 Encoder Angles", 2)
    ranges = None
    if LIDAR_KEY in raw:
        try:
            ranges = decode_lidar(raw[LIDAR_KEY])
        except ValueError as error:
            raise FrameError(f"{LIDAR_KEY!r}: {error}") from None
    return SimFrame(
        throttle_echo=_scalar(raw, "V1 Throttle"),
        steering_echo=_scalar(raw, "V1 Steering"),
        encoder_angles=(angles[0], angles[1]),
        encoder_ticks=(int(ticks[0]), int(ticks[1])),
        rear_axle=_vec(raw, "V1 Position", 3),
        orientation=quaternion,
        yaw=_wrap(yaw_from_quaternion(*quaternion)),
        linear_velocity=_vec(raw, "V1 Linear Velocity", 3),
        angular_velocity=_vec(raw, "V1 Angular Velocity", 3),
        linear_acceleration=_vec(raw, "V1 Linear Acceleration", 3),
        lidar_scan_rate=_scalar(raw, "V1 LIDAR Scan Rate"),
        ranges=ranges,
        lap_count=int(_scalar(raw, "V1 Lap Count")),
        lap_time=_scalar(raw, "V1 Lap Time"),
        last_lap_time=_scalar(raw, "V1 Last Lap Time"),
        best_lap_time=_scalar(raw, "V1 Best Lap Time"),
        collisions=int(_scalar(raw, "V1 Collisions")),
    )
