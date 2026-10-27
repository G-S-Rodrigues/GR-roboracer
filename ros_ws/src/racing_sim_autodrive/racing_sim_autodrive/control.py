"""Control helpers of the AutoDRIVE backend: throttle PI, steering, reset.

Vehicle values (throttle gain, max steering angle, PI gains) are arguments
and have no defaults here; part 4's vehicle file is where they live.
"""

from __future__ import annotations


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class ThrottleLoop:
    """Feedforward plus PI on speed, output throttle in [-limit, limit].

    u = clamp(v_cmd / gain + kp * e + I), e = v_cmd - v_meas.
    """

    def __init__(
        self, gain: float, kp: float, ki: float, limit: float = 1.0
    ) -> None:
        if not gain > 0.0:
            raise ValueError(f"throttle gain must be > 0, got {gain}")
        if not limit > 0.0:
            raise ValueError(f"limit must be > 0, got {limit}")
        self._gain = gain
        self._kp = kp
        self._ki = ki
        self._limit = limit
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update(
        self, v_cmd: float, v_meas: float | None, dt: float | None
    ) -> float:
        if v_cmd == 0.0:
            self.integral = 0.0
            return 0.0
        feedforward = v_cmd / self._gain
        if v_meas is None:
            return _clamp(feedforward + self.integral, self._limit)
        error = v_cmd - v_meas
        unclamped = feedforward + self._kp * error + self.integral
        if dt is not None and dt > 0.0:
            # conditional integration: do not wind up further into a limit
            into_limit = (unclamped >= self._limit and error > 0.0) or (
                unclamped <= -self._limit and error < 0.0
            )
            if not into_limit:
                self.integral += self._ki * error * dt
        return _clamp(
            feedforward + self._kp * error + self.integral, self._limit
        )


def steering_to_normalized(angle: float, max_angle: float) -> float:
    """Steering angle (rad, positive left) -> command in [-1, 1]."""
    if not max_angle > 0.0:
        raise ValueError(f"max_angle must be > 0, got {max_angle}")
    return _clamp(angle / max_angle, 1.0)


class ResetLatch:
    """Turns a requested level into one True per rising edge."""

    def __init__(self) -> None:
        self._previous = False

    def next(self, requested: bool) -> bool:
        fire = requested and not self._previous
        self._previous = requested
        return fire
