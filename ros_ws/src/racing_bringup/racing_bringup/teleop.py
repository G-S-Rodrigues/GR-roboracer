"""The rate limiter behind `slam.launch.py drive:=teleop`. ROS-free.

Convention: a Twist's `linear.x` is the target speed in m/s and
`angular.z` is the target **steering angle** in rad, not a yaw rate. A yaw
rate has no steering angle at zero speed, and a keyboard starts from
standstill. Time is plain floats, so the core tests in milliseconds.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Limits:
    """How hard the car may be driven by hand; units in the field names."""

    max_speed: float  # m/s forward
    max_reverse_speed: float  # m/s, positive
    acceleration: float  # m/s^2 while |speed| grows
    deceleration: float  # m/s^2 while |speed| shrinks
    max_steering_angle: float  # rad
    steering_rate: float  # rad/s
    input_timeout: float  # s without input before the target is zero
    max_step: float  # s, the largest dt one step may use (gotcha #45)
    rate_hz: float  # Hz the node publishes /drive at


def _toward(current: float, target: float, step: float) -> float:
    """`current` moved toward `target` by at most `step`, never past it."""
    if abs(target - current) <= step:
        return target
    return current + step if target > current else current - step


class Limiter:
    """Holds the commanded speed and steering and ramps them to targets."""

    def __init__(
        self, limits: Limits, speed: float = 0.0, steering: float = 0.0
    ) -> None:
        self.limits = limits
        self.speed = speed
        self.steering = steering

    def step(
        self, target_speed: float, target_steering: float, dt: float
    ) -> tuple[float, float]:
        """Advance by `dt` s; a dt <= 0 holds, one over max_step is capped."""
        if dt <= 0.0:
            return self.speed, self.steering
        dt = min(dt, self.limits.max_step)
        limits = self.limits
        target = max(
            -limits.max_reverse_speed, min(limits.max_speed, target_speed)
        )
        if target * self.speed < 0.0:
            target = 0.0  # a sign change stops first
        growing = abs(target) > abs(self.speed)
        rate = limits.acceleration if growing else limits.deceleration
        self.speed = _toward(self.speed, target, rate * dt)
        bound = limits.max_steering_angle
        angle = max(-bound, min(bound, target_steering))
        self.steering = _toward(self.steering, angle, limits.steering_rate * dt)
        return self.speed, self.steering


def targets(
    linear_x: float, angular_z: float, age: float | None, timeout: float
) -> tuple[float, float]:
    """(speed, steering angle) wanted; (0, 0) when the input is older than
    `timeout` s or `age` is None (nothing received yet)."""
    if age is None or age > timeout:
        return 0.0, 0.0
    return linear_x, angular_z
