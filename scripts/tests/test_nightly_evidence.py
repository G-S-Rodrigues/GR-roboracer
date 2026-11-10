"""CHECK-2378: the real nightly evidence gives the named verdicts.

Three trimmed recordings of the red nightlies (scripts/tests/fixtures/nightly,
README there) are replayed through the committed functions: ``FrozenPose``
and ``error_at`` for the two ACC-4040 collisions, ``estimator_start`` for the
late MOLA estimate. What the keyword would say is checked against the real
numbers. The one-off replay of every passing recording (none freezes) is in
the N1 PR, because those recordings are 25-210 MB.
"""

from __future__ import annotations

import gzip
import json
import shutil
import sys
from pathlib import Path

import pytest

from scripts import compare_localization
from scripts.frozen_pose import FROZEN_WINDOW_SECONDS, FrozenPose

LIB_ROOT = Path(__file__).resolve().parents[2] / "tests" / "lib"
if str(LIB_ROOT) not in sys.path:
    sys.path.insert(0, str(LIB_ROOT))

from racing_test_keywords import keywords  # noqa: E402
from racing_test_keywords.scenario_runner import RunOutcome  # noqa: E402

FIXTURES = Path(__file__).parent / "fixtures" / "nightly"
SPIELBERG_LENGTH = 343.3
STATE = "/ground_truth/track_relative_state"


def gunzip(name: str, tmp_path: Path) -> Path:
    target = tmp_path / name.removesuffix(".gz")
    with gzip.open(FIXTURES / name, "rb") as source, target.open("wb") as out:
        shutil.copyfileobj(source, out)
    return target


def freeze_of(events, until: float | None = None):
    """``FrozenPose`` over the recorded truth state, as the live stop does."""
    pose = FrozenPose(2 * FROZEN_WINDOW_SECONDS, SPIELBERG_LENGTH)
    for event in events:
        if event["topic"] != STATE:
            continue
        t = compare_localization._stamp(event)
        if until is not None and t > until:
            break
        pose.update(t, event["payload"]["s"], event["payload"]["d"])
    return pose.frozen_at


def verdict(events, frozen_at, tmp_path: Path) -> str:
    """The text ``Lap Completion Should Be  true`` raises for this freeze."""
    recording = tmp_path / "recording.jsonl"
    recording.write_text(
        "\n".join(json.dumps(event) for event in events) + "\n"
    )
    library = keywords.RacingTestKeywords()
    outcome = RunOutcome(lap_completed=False)
    outcome.frozen_at = frozen_at
    outcome.stop_context = {"text": "", "latched": False}
    library._metrics = outcome
    library._recording = recording
    with pytest.raises(AssertionError) as raised:
        library.lap_completion_should_be("true")
    return str(raised.value)


COLLISIONS = [
    # fixture, anchor t, s, d, estimate lon, lat at the freeze
    (
        "acc4040-raceline-20261104-slam2026.jsonl.gz",
        64.69,
        34.5,
        -0.87,
        "+0.35",
        "-0.01",
    ),
    (
        "acc4040-raceline-20261110-slam7.jsonl.gz",
        110.51,
        173.8,
        -0.88,
        "+1.21",
        "+0.07",
    ),
]


@pytest.mark.parametrize("name,t,s,d,lon,lat", COLLISIONS)
def test_check_2378_the_nightly_collisions_are_named(
    tmp_path, name, t, s, d, lon, lat
):
    events = compare_localization.read_recording(gunzip(name, tmp_path))
    frozen_at = freeze_of(events)
    assert frozen_at is not None
    assert (round(frozen_at[0], 2), round(frozen_at[1], 1)) == (t, s)
    assert round(frozen_at[2], 2) == d
    text = verdict(events, frozen_at, tmp_path)
    assert (
        f"ground truth frozen at sim {t:.2f} s, s {s}, d {d}: "
        "the gym holds its state on a collision"
    ) in text
    assert f"estimate lon {lon} m, lat {lat} m at the freeze" in text


@pytest.mark.parametrize("name,t,s,d,lon,lat", COLLISIONS)
def test_check_2378_the_replay_can_fail(tmp_path, name, t, s, d, lon, lat):
    """With the freeze cut off, nothing is reported: the test above is not
    passing on a fixture that always freezes."""
    events = compare_localization.read_recording(gunzip(name, tmp_path))
    before_the_window_is_full = t + 2 * FROZEN_WINDOW_SECONDS - 0.5
    assert freeze_of(events, until=before_the_window_is_full) is None


def test_check_2378_the_late_mola_start_is_named(tmp_path):
    events = compare_localization.read_recording(
        gunzip("acc4510-mola-20261110-slam7.jsonl.gz", tmp_path)
    )
    start = compare_localization.estimator_start(events)
    assert start["first_estimate_stamp"] == pytest.approx(1.03)
    assert start["travel_before_first_estimate"] == pytest.approx(2.5, abs=0.05)
    # an estimator that started at once would read 0.0, not None
    prompt = [
        e
        for e in events
        if not (
            e["topic"] == "/localization/odom"
            and compare_localization._stamp(e) < 0.2
        )
    ]
    assert compare_localization.estimator_start(prompt)[
        "first_estimate_stamp"
    ] == pytest.approx(1.03)
