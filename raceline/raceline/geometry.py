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
