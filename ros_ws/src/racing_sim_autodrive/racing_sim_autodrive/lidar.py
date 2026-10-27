"""Decoding of the sim's `V1 LIDAR Range Array` field."""

import base64
import gzip

import numpy as np


def decode_lidar(field: str) -> np.ndarray:
    """base64(gzip("\\n"-joined floats)) -> float64 ranges, shape (N,).

    N is whatever the payload carries. `inf` (no return) is kept; NaN and
    negative ranges are not ranges and raise ValueError.
    """
    text = gzip.decompress(base64.b64decode(field)).decode("utf-8")
    tokens = [t for t in text.split("\n") if t.strip()]
    ranges = np.array([float(t) for t in tokens], dtype=np.float64)
    if np.isnan(ranges).any():
        raise ValueError("lidar ranges contain NaN")
    if (ranges < 0).any():
        raise ValueError("lidar ranges contain a negative value")
    return ranges
