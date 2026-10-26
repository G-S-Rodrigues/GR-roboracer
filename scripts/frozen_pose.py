"""One definition of a frozen pose, shared by a live lap and the report.

Stdlib-only. ``scenario_runner`` (every tier-3/4 lap) and
``scripts.raceline_report`` (tier 5) both import it, so "frozen" means one
thing (gotcha 15's lesson about two definitions of a lap). It stays out of
``raceline_report.py`` because that module is a neutral path (ADR 0007 section
2) and no lap may depend on one.
"""

from __future__ import annotations

# A pose that moved less than this over the whole window is frozen. The gym
# keeps the old state on a collision, so a frozen pose repeats exactly.
FROZEN_EPSILON_M = 1e-3


def _wrap(delta: float, length: float) -> float:
    return (delta + length / 2.0) % length - length / 2.0


class FrozenPose:
    """Streaming test: has the pose (s and d) stood still for ``window_s``?

    ``update(t, s, d)`` takes one sample in time order and returns ``frozen``.
    The still stretch is anchored at the first sample of the run of samples
    within ``FROZEN_EPSILON_M`` of it (s wraps modulo ``track_length``); the
    pose is frozen once the stretch spans ``window_s`` of sample time.
    """

    def __init__(self, window_s: float, track_length: float) -> None:
        self.window_s = window_s
        self.track_length = track_length
        self.frozen = False
        self._anchor: tuple[float, float, float] | None = None

    def update(self, t: float, s: float, d: float) -> bool:
        anchor = self._anchor
        if (
            anchor is None
            or abs(_wrap(s - anchor[1], self.track_length)) >= FROZEN_EPSILON_M
            or abs(d - anchor[2]) >= FROZEN_EPSILON_M
        ):
            self._anchor = anchor = (t, s, d)
        self.frozen = t - anchor[0] >= self.window_s
        return self.frozen
