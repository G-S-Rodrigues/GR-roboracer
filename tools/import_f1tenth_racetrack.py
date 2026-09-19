#!/usr/bin/env python3
"""Convert one f1tenth/f1tenth_racetracks track into this repo's track schema.

One-shot, committed so the conversion is reproducible and reviewable. Input is
the upstream pair, at a pinned revision:

    https://github.com/f1tenth/f1tenth_racetracks  (GPL-3.0)
    <Track>/<Track>_centerline.csv   # x_m, y_m, w_tr_right_m, w_tr_left_m
    <Track>/<Track>_raceline.csv     # s_m; x_m; y_m; psi_rad; kappa_radpm; ...

Output is `config/tracks/<name>.yaml` (`analytic_circle.yaml`'s schema). The
gym map assets are then generated from that file by `sim/track_importer.py`,
exactly as `sim/rollout.py` regenerates them, so the live sim and the headless
golden ray-march the same world.

Two traps this script exists to get right, both silent if wrong:

- The upstream centerline is **right then left** width; the canonical schema
  is left then right. Swapping them is invisible on a symmetric corridor and
  wrong everywhere else.
- Curvature comes from the raceline's `kappa` column (nearest raceline
  sample), never from finite differences of the coarsely sampled centerline,
  which are noisy. Nothing at runtime reads it today; a wrong value would
  still be invisible the day something does.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def _rows(path: Path, delimiter: str) -> list[list[float]]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        rows.append([float(value) for value in stripped.split(delimiter)])
    return rows


def convert(
    centerline_csv: Path, raceline_csv: Path, name: str, source: str = ""
) -> str:
    """Return the canonical track YAML for one upstream track."""
    centerline = _rows(centerline_csv, ",")
    raceline = _rows(raceline_csv, ";")
    if len(centerline) < 3 or len(raceline) < 3:
        raise ValueError("a closed track needs at least three points")
    if any(len(row) != 4 for row in centerline):
        raise ValueError("centerline rows must be x, y, w_right, w_left")
    if any(len(row) != 7 for row in raceline):
        raise ValueError("raceline rows must be s, x, y, psi, kappa, vx, ax")

    lines = [f"# {line}" if line else "#" for line in source.splitlines()]
    lines += [
        "format_version: 1",
        "metadata:",
        f"  name: {name}",
        "  closed: true",
        "  units: meters",
        "centerline:",
        "  # x, y, curvature, left width, right width",
    ]
    for x, y, width_right, width_left in centerline:
        nearest = min(
            raceline,
            key=lambda row, x=x, y=y: math.hypot(row[1] - x, row[2] - y),
        )
        kappa = nearest[4]
        lines.append(
            f"  - [{x:.9f}, {y:.9f}, {kappa:.9f}, "
            f"{width_left:.6f}, {width_right:.6f}]"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("centerline_csv", type=Path)
    parser.add_argument("raceline_csv", type=Path)
    parser.add_argument("--name", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--source", default="", help="provenance, written as a header comment"
    )
    arguments = parser.parse_args()
    arguments.output.write_text(
        convert(
            arguments.centerline_csv,
            arguments.raceline_csv,
            arguments.name,
            arguments.source,
        ),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
