"""The qualification score (grill 33-35): 12 laps, the first a warm-up and the
last a cool-down, the ten between scored; every collision adds 10 s; more
than 10 collisions disqualify. ROS-free."""

from __future__ import annotations

from dataclasses import dataclass

LAPS = 12
WARM_UP = 1
COOL_DOWN = 1
PENALTY_S = 10.0
MAX_COLLISIONS = 10


@dataclass(frozen=True)
class Score:
    total: float | None  # None when disqualified
    scored_laps: list[float]
    dq: bool


def score(lap_times, collisions: int) -> Score:
    """Score a run from its lap durations and collision count."""
    laps = [float(t) for t in lap_times]
    if len(laps) < LAPS:
        raise ValueError(f"a qualification run is {LAPS} laps, got {len(laps)}")
    scored = laps[WARM_UP : LAPS - COOL_DOWN]
    if collisions > MAX_COLLISIONS:
        return Score(None, scored, True)
    return Score(sum(scored) + PENALTY_S * collisions, scored, False)
