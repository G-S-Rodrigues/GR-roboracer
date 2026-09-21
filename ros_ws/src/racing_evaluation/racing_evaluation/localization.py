"""ROS-free core: score a pose-estimate stream against ground truth.

Every measure here is taken over ground-truth samples, never over estimates:
an estimator that stops publishing leaves truth samples it did not score,
which are counted, and never contributes an error of zero (EVAL-1030).
"""

from __future__ import annotations

import bisect
import math
from collections import deque
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from typing import Protocol

import racing_common

# How far apart two estimates may be and still be interpolated between, and
# so how stale an estimate may be before a truth sample counts as
# unavailable. Not a measurement: a design bound, sized to the vehicle. At
# the reference stack's 3.0 m/s (config/vehicles/f1tenth_default.yaml) half
# a second is 1.5 m of travel - more than half the width of Spielberg's
# 2.2 m corridor, which no pose worth driving on is allowed to lag by.
DEFAULT_WINDOW = 0.5


@dataclass(frozen=True)
class Pose:
    """A planar pose at a time: seconds, metres, radians."""

    stamp: float
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class AlignedPair:
    """One truth sample and the estimate at its stamp, or None if there was
    no estimate within the window of it."""

    truth: Pose
    estimate: Pose | None


@dataclass(frozen=True)
class ErrorSummary:
    """An error measure over ground-truth samples.

    ``sample_count`` is every truth sample the measure covers; ``scored_count``
    the ones with an estimate. With nothing scored the statistics are NaN,
    never zero.
    """

    rmse: float
    mean: float
    maximum: float
    sample_count: int
    scored_count: int

    @property
    def availability(self) -> float:
        if self.sample_count == 0:
            return 0.0
        return self.scored_count / self.sample_count


@dataclass(frozen=True)
class AvailabilitySummary:
    """How much of [start, end] an estimate stream covered.

    ``availability`` is the fraction of the interval within ``window`` after
    some estimate; ``maximum_gap`` the longest stretch without one (seconds);
    ``stale_gap_count`` how many stretches exceeded the window.
    """

    availability: float
    maximum_gap: float
    stale_gap_count: int


@dataclass(frozen=True)
class ComponentError:
    """Estimate minus truth, track-relative: metres, metres, radians."""

    lateral: float
    longitudinal: float
    heading: float


class FrenetTrack(Protocol):
    """What the decomposition needs: racing_common.Track satisfies it."""

    def to_frenet(self, pose): ...

    def length(self) -> float: ...


def wrap_angle(angle: float) -> float:
    """Wrap to [-pi, pi]."""
    return math.remainder(angle, math.tau)


def _interpolate(before: Pose, after: Pose, stamp: float) -> Pose:
    span = after.stamp - before.stamp
    fraction = 0.0 if span <= 0.0 else (stamp - before.stamp) / span
    return Pose(
        stamp=stamp,
        x=before.x + fraction * (after.x - before.x),
        y=before.y + fraction * (after.y - before.y),
        yaw=before.yaw + fraction * wrap_angle(after.yaw - before.yaw),
    )


class Aligner:
    """Pairs each truth sample with the estimate interpolated to its stamp.

    Live, truth usually arrives before the estimate for the same instant. A
    truth sample therefore waits until an estimate at or after its stamp
    arrives, and is declared unavailable only once truth has advanced more
    than ``window`` past it. Estimates are assumed to arrive in stamp order.
    Pairs are emitted in truth order.
    """

    def __init__(self, window: float = DEFAULT_WINDOW) -> None:
        if window <= 0.0:
            raise ValueError("window must be positive")
        self._window = window
        self._estimates: list[Pose] = []
        self._stamps: list[float] = []
        self._pending: deque[Pose] = deque()
        self._latest_truth = -math.inf

    def add_estimate(self, pose: Pose) -> list[AlignedPair]:
        index = bisect.bisect_right(self._stamps, pose.stamp)
        self._stamps.insert(index, pose.stamp)
        self._estimates.insert(index, pose)
        return self._resolve(final=False)

    def add_truth(self, pose: Pose) -> list[AlignedPair]:
        self._pending.append(pose)
        self._latest_truth = max(self._latest_truth, pose.stamp)
        return self._resolve(final=False)

    def flush(self) -> list[AlignedPair]:
        """Resolve every waiting sample with the estimates received so far."""
        return self._resolve(final=True)

    def _estimate_at(
        self, stamp: float, final: bool
    ) -> tuple[bool, Pose | None]:
        """(decided, estimate) for one truth stamp."""
        index = bisect.bisect_left(self._stamps, stamp)
        if index < len(self._stamps) and self._stamps[index] == stamp:
            return True, self._estimates[index]
        has_before = index > 0
        has_after = index < len(self._stamps)
        if has_after:
            if not has_before:
                return True, None
            before = self._estimates[index - 1]
            after = self._estimates[index]
            if after.stamp - before.stamp > self._window:
                return True, None
            return True, _interpolate(before, after, stamp)
        if final or self._latest_truth - stamp > self._window:
            return True, None
        return False, None

    def _resolve(self, final: bool) -> list[AlignedPair]:
        pairs: list[AlignedPair] = []
        while self._pending:
            truth = self._pending[0]
            decided, estimate = self._estimate_at(truth.stamp, final)
            if not decided:
                break
            self._pending.popleft()
            pairs.append(AlignedPair(truth, estimate))
        self._discard_unneeded_estimates()
        return pairs

    def _discard_unneeded_estimates(self) -> None:
        # Keep the last estimate before the oldest waiting sample: it is
        # the "before" end of that sample's interpolation.
        horizon = (
            self._pending[0].stamp if self._pending else self._latest_truth
        )
        keep_from = max(bisect.bisect_left(self._stamps, horizon) - 1, 0)
        if keep_from:
            del self._stamps[:keep_from]
            del self._estimates[:keep_from]


def align(
    estimates: Iterable[Pose],
    truth: Iterable[Pose],
    window: float = DEFAULT_WINDOW,
) -> list[AlignedPair]:
    """Offline alignment: every estimate is known before truth is scored."""
    aligner = Aligner(window)
    for pose in sorted(estimates, key=lambda pose: pose.stamp):
        aligner.add_estimate(pose)
    pairs: list[AlignedPair] = []
    for pose in sorted(truth, key=lambda pose: pose.stamp):
        pairs.extend(aligner.add_truth(pose))
    pairs.extend(aligner.flush())
    return pairs


class RunningError:
    """``summarize`` kept incrementally, for a live stream of samples."""

    def __init__(self) -> None:
        self._sample_count = 0
        self._errors = 0
        self._squared_sum = 0.0
        self._sum = 0.0
        self._maximum = math.nan

    def add(self, error: float | None) -> None:
        """One truth sample: its error, or None if it was unavailable."""
        self._sample_count += 1
        if error is None:
            return
        self._errors += 1
        self._squared_sum += error * error
        self._sum += error
        self._maximum = (
            error if math.isnan(self._maximum) else max(self._maximum, error)
        )

    def summary(self) -> ErrorSummary:
        if not self._errors:
            return ErrorSummary(
                math.nan, math.nan, math.nan, self._sample_count, 0
            )
        return ErrorSummary(
            rmse=math.sqrt(self._squared_sum / self._errors),
            mean=self._sum / self._errors,
            maximum=self._maximum,
            sample_count=self._sample_count,
            scored_count=self._errors,
        )


def summarize(errors: Sequence[float], sample_count: int) -> ErrorSummary:
    """Statistics over the scored errors of ``sample_count`` truth samples."""
    running = RunningError()
    for error in errors:
        running.add(error)
    for _ in range(sample_count - len(errors)):
        running.add(None)
    return running.summary()


def position_error(estimate: Pose, truth: Pose) -> float:
    return math.hypot(estimate.x - truth.x, estimate.y - truth.y)


def absolute_trajectory_error(
    estimates: Iterable[Pose],
    truth: Iterable[Pose],
    window: float = DEFAULT_WINDOW,
) -> ErrorSummary:
    """Position error at every truth sample, in the truth frame."""
    pairs = align(estimates, truth, window)
    errors = [
        position_error(pair.estimate, pair.truth)
        for pair in pairs
        if pair.estimate is not None
    ]
    return summarize(errors, len(pairs))


def _relative(start: Pose, end: Pose) -> tuple[float, float, float]:
    """``end`` expressed in ``start``'s frame."""
    dx = end.x - start.x
    dy = end.y - start.y
    cosine = math.cos(start.yaw)
    sine = math.sin(start.yaw)
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        wrap_angle(end.yaw - start.yaw),
    )


def relative_pose_error(
    estimates: Iterable[Pose],
    truth: Iterable[Pose],
    delta: float,
    window: float = DEFAULT_WINDOW,
) -> ErrorSummary:
    """Translation error of the motion over ``delta`` seconds, in the
    vehicle frame: drift per interval, blind to a constant offset."""
    if delta <= 0.0:
        raise ValueError("delta must be positive")
    pairs = align(estimates, truth, window)
    stamps = [pair.truth.stamp for pair in pairs]
    # Float stamps: a partner delta later may sit a rounding error early.
    tolerance = 1e-9
    errors: list[float] = []
    sample_count = 0
    for start, pair in enumerate(pairs):
        end = bisect.bisect_left(
            stamps, pair.truth.stamp + delta - tolerance, lo=start + 1
        )
        if end == len(pairs):
            break
        sample_count += 1
        partner = pairs[end]
        if pair.estimate is None or partner.estimate is None:
            continue
        true_x, true_y, true_yaw = _relative(pair.truth, partner.truth)
        est_x, est_y, _ = _relative(pair.estimate, partner.estimate)
        # The estimated motion expressed in the true motion's end frame.
        cosine = math.cos(true_yaw)
        sine = math.sin(true_yaw)
        dx = est_x - true_x
        dy = est_y - true_y
        errors.append(
            math.hypot(cosine * dx + sine * dy, -sine * dx + cosine * dy)
        )
    return summarize(errors, sample_count)


def lateral_longitudinal_heading_error(
    estimate: Pose, truth: Pose, track: FrenetTrack
) -> ComponentError:
    """Estimate minus truth in the track's Frenet frame.

    Longitudinal error wraps at the track length, so an estimate just past
    the start line is slightly ahead, not a lap behind.
    """
    estimated = track.to_frenet(
        racing_common.CartesianPose(estimate.x, estimate.y, estimate.yaw)
    )
    true = track.to_frenet(
        racing_common.CartesianPose(truth.x, truth.y, truth.yaw)
    )
    length = track.length()
    return ComponentError(
        lateral=estimated.d - true.d,
        longitudinal=math.remainder(estimated.s - true.s, length),
        heading=wrap_angle(estimate.yaw - truth.yaw),
    )


def availability(
    estimates: Iterable[Pose],
    window: float,
    start: float,
    end: float,
) -> AvailabilitySummary:
    """Coverage of [start, end] by an estimate stream (EVAL-1030).

    The interval comes from ground truth, not from the estimates: a stream
    that stops early must not define the span it is judged over.
    """
    if end <= start:
        raise ValueError("end must be after start")
    stamps = sorted(
        pose.stamp for pose in estimates if start <= pose.stamp <= end
    )
    boundaries = [start, *stamps, end]
    covered = 0.0
    maximum_gap = 0.0
    stale_gap_count = 0
    for index in range(len(boundaries) - 1):
        gap = boundaries[index + 1] - boundaries[index]
        maximum_gap = max(maximum_gap, gap)
        if gap > window:
            stale_gap_count += 1
        # Time before the first estimate is uncovered; after one, the
        # window it stays fresh for.
        if index > 0:
            covered += min(gap, window)
    return AvailabilitySummary(
        availability=covered / (end - start),
        maximum_gap=maximum_gap,
        stale_gap_count=stale_gap_count,
    )
