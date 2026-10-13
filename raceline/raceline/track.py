"""The canonical track file (``format_version: 1``) as a ROS-free type.

This duplicates what ``racing_common``'s C++ ``Track::from_yaml``
(``track.cpp:59-100``) knows about the format, on purpose: ``racing_common`` is
not importable in this environment (RACELINE-1020). RACELINE-1100 pins this
module to the committed ``config/tracks/spielberg.yaml``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class Track:
    name: str
    xy: np.ndarray  # (N, 2), closed loop, no repeated closing point
    curvature: np.ndarray  # (N,), 1/m, + = left turn
    w_left: np.ndarray  # (N,), m, > 0
    w_right: np.ndarray  # (N,), m, > 0


def _validate(xy, widths_left, widths_right) -> None:
    """The rules ``track.cpp`` enforces, plus two stricter ones.

    Stricter than ``track.cpp`` (which accepts 2 points and open tracks):
    fewer than 3 points is rejected here, and so is ``closed: false``.
    """
    if len(xy) < 3:
        raise ValueError(f"a track needs at least 3 points, got {len(xy)}")
    if np.any(np.asarray(widths_left) <= 0.0) or np.any(
        np.asarray(widths_right) <= 0.0
    ):
        raise ValueError("track widths must be positive")
    seg = np.roll(xy, -1, axis=0) - xy  # includes last -> first
    if np.any(np.hypot(seg[:, 0], seg[:, 1]) <= 0.0):
        raise ValueError("track has a zero-length segment")


def load_track(path: Path) -> Track:
    doc = yaml.safe_load(Path(path).read_text())
    if doc.get("format_version") != 1:
        raise ValueError(f"unsupported format_version {doc.get('format_version')}")
    meta = doc.get("metadata") or {}
    for key in ("name", "closed", "units"):
        if key not in meta:
            raise ValueError(f"metadata.{key} is missing")
    if meta["units"] != "meters":
        raise ValueError(f"units must be meters, got {meta['units']}")
    if meta["closed"] is not True:
        raise ValueError("only closed tracks are supported")
    raw = doc["centerline"]
    if any(len(row) != 5 for row in raw):
        raise ValueError("every centerline row needs 5 values")
    rows = np.asarray(raw, dtype=float)
    _validate(rows[:, :2], rows[:, 3], rows[:, 4])
    return Track(
        name=doc["metadata"]["name"],
        xy=rows[:, :2],
        curvature=rows[:, 2],
        w_left=rows[:, 3],
        w_right=rows[:, 4],
    )


def write_track(track: Track, path: Path, source: str = "") -> Path:
    _validate(track.xy, track.w_left, track.w_right)
    path = Path(path)
    lines = [f"# {s}" for s in source.splitlines()]
    lines += [
        "format_version: 1",
        "metadata:",
        f"  name: {track.name}",
        "  closed: true",
        "  units: meters",
        "centerline:",
        "  # x, y, curvature, left width, right width",
    ]
    for (x, y), k, wl, wr in zip(
        track.xy, track.curvature, track.w_left, track.w_right
    ):
        lines.append(f"  - [{x:.9f}, {y:.9f}, {k:.9f}, {wl:.6f}, {wr:.6f}]")
    path.write_text("\n".join(lines) + "\n")
    return path
