"""Closed-polyline geometry shared by extraction, optimization and verification.

Moved out of ``extract.py`` unchanged (part 4 Step 3) so the verifier and the
optimizer can use it without importing the map extractor.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def resample_closed(pts: np.ndarray, step: float) -> np.ndarray:
    closed = np.vstack([pts, pts[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(round(s[-1] / step)), 3)
    t = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.column_stack(
        [np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])]
    )


def signed_area(xy: np.ndarray) -> float:
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def tangents(xy: np.ndarray) -> np.ndarray:
    t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    return t / np.hypot(t[:, 0], t[:, 1])[:, None]


def nearest_distance(
    xy: np.ndarray, polyline: np.ndarray, step: float
) -> np.ndarray:
    """Distance from each point of ``xy`` to ``polyline`` at ``step``."""
    d, _ = cKDTree(resample_closed(polyline, step)).query(xy)
    return d


def crossings(
    wall: np.ndarray,
) -> list[tuple[int, int, np.ndarray]]:
    """Every crossing between two non-adjacent segments of a closed polyline.

    Returns ``(i, j, xy)`` with ``i < j``: segment ``i`` runs from point ``i``
    to ``i + 1``, and the last segment closes onto point 0. Candidate pairs
    come from the segment midpoints within the longest segment's length (two
    crossing segments' midpoints can be no farther apart than that), then an
    exact segment-segment test.
    """
    n = len(wall)
    start, end = wall, np.roll(wall, -1, axis=0)
    mid = (start + end) / 2.0
    radius = float(np.hypot(*(end - start).T).max())
    found = []
    for i, j in sorted(cKDTree(mid).query_pairs(radius)):
        if j - i <= 1 or (i == 0 and j == n - 1):
            continue
        d1, d2 = end[i] - start[i], end[j] - start[j]
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-15:
            continue
        w = start[j] - start[i]
        t = (w[0] * d2[1] - w[1] * d2[0]) / den
        u = (w[0] * d1[1] - w[1] * d1[0]) / den
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            found.append((i, j, start[i] + t * d1))
    return found


def remove_folds(wall: np.ndarray, max_loop: float) -> np.ndarray:
    """Cut the small loops out of an offset of a simple closed curve.

    Where an offset's distance exceeds the inside radius it grows a
    swallowtail; sampling noise grows tiny ones too. At each crossing the
    polyline splits into two loops. The one with fewer points is cut and
    replaced by the crossing point if every one of its points lies within
    ``max_loop`` of the crossing; otherwise the wall passes over itself and
    ``ValueError`` ("self-intersect") is raised. Winding is not a criterion.
    A wall with no crossing is returned unchanged.
    """
    current = wall
    for _ in range(len(wall)):
        found = crossings(current)
        if not found:
            return current
        i, j, x = found[0]
        inner = current[i + 1 : j + 1]
        outer = np.vstack([current[j + 1 :], current[: i + 1]])
        small = inner if len(inner) <= len(outer) else outer
        reach = np.hypot(*(small - x).T).max() if len(small) else 0.0
        if reach > max_loop:
            raise ValueError(
                f"wall segments {i} and {j} self-intersect at "
                f"({x[0]:.3f}, {x[1]:.3f}); the smaller loop reaches "
                f"{reach:.3f} m, over the {max_loop:.3f} m bound"
            )
        if small is inner:
            current = np.vstack([current[: i + 1], x, current[j + 1 :]])
        else:
            current = np.vstack([x, inner])
    raise ValueError("wall still self-intersects after removing its folds")
