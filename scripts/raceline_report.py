"""The tier-5 raceline comparison report and regeneration helpers.

ROS-free: standard library, ``yaml`` and ``numpy``. Only SIM-5050 adds
``racing_common`` (the track length a lap is counted on, repo-gotchas #15).
Stub: every function raises until its CHECK-14xx test is green.
"""

from __future__ import annotations

from pathlib import Path


def raceline_geometry(csv_path: Path) -> dict:
    raise NotImplementedError("raceline_geometry")


def read_recording_tolerant(path: Path) -> list[dict]:
    raise NotImplementedError("read_recording_tolerant")


def read_manifest(path: Path) -> dict:
    raise NotImplementedError("read_manifest")


def run_outcome(events, observer_text, track_length, frozen_window_s) -> dict:
    raise NotImplementedError("run_outcome")


def summarise(runs: list[dict], lines: dict | None = None) -> list[dict]:
    raise NotImplementedError("summarise")


def structural_failures(runs: list[dict], expected: dict) -> list[str]:
    raise NotImplementedError("structural_failures")


def compare_lines(committed: Path, fresh: Path) -> dict:
    raise NotImplementedError("compare_lines")


def normalise_command(command: str) -> str:
    raise NotImplementedError("normalise_command")


def provenance_diff(committed: dict, fresh: dict) -> dict:
    raise NotImplementedError("provenance_diff")


def regeneration_argv(doc: dict, out_dir: Path) -> list[str]:
    raise NotImplementedError("regeneration_argv")


def upstream_reference(root: Path, track: str) -> dict | None:
    raise NotImplementedError("upstream_reference")


def render(rows, regeneration, reference=None) -> str:
    raise NotImplementedError("render")
