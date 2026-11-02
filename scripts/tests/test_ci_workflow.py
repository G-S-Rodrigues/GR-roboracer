"""Tests for .github/workflows/ci.yaml: what each job needs before check.sh."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
INIT = "git submodule update --init --depth 1 third_party/GR-roboracer-data"


def _jobs() -> dict:
    document = yaml.safe_load(
        (REPO_ROOT / ".github" / "workflows" / "ci.yaml").read_text()
    )
    return document["jobs"]


def _check_jobs() -> list[str]:
    return sorted(
        name
        for name, job in _jobs().items()
        if any("check.sh" in str(s.get("run", "")) for s in job["steps"])
    )


def test_check_2340_the_workflow_has_check_jobs() -> None:
    """CHECK-2340 is never vacuous: the PR gate and the nightly run check.sh."""
    assert len(_check_jobs()) >= 2


@pytest.mark.parametrize("job", _check_jobs())
def test_check_2340_every_check_job_initialises_the_data_submodule(
    job: str,
) -> None:
    """CHECK-2340: a job that runs `check.sh` reads the maps and racelines in
    third_party/GR-roboracer-data (ADR 0008), so it must initialise that
    submodule after checkout and before `check.sh`. Without the step the job
    fails closed at the first scenario load, naming the init command; the
    guard is that the step exists, not that GitHub serves the pinned SHA."""
    steps = _jobs()[job]["steps"]
    runs = [str(step.get("run", "")) for step in steps]
    check = [i for i, text in enumerate(runs) if "check.sh" in text]
    init = [i for i, text in enumerate(runs) if INIT in text]
    assert init, f"{job} never runs: {INIT}"
    assert init[0] < check[0], f"{job} runs check.sh before the submodule init"
