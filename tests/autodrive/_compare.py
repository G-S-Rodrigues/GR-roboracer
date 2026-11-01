"""Message flattening and pairwise comparison (not a test)."""

import math

from rosidl_runtime_py.convert import message_to_ordereddict

# Times derived from the receipt: A stamps on arrival, B takes the devkit's,
# and `valid_until` is the stamp plus a validity.
TIMES = ("stamp", "valid_until")


def flatten(value, path="", out=None):
    """(path, leaf) pairs of a message without its times (TIMES): they differ
    by construction between front ends."""
    if out is None:
        out = []
    if isinstance(value, dict):
        for key, item in value.items():
            if key in TIMES:
                continue
            flatten(item, f"{path}.{key}", out)
    elif isinstance(value, (list, tuple)) or hasattr(value, "tolist"):
        for index, item in enumerate(
            value.tolist() if hasattr(value, "tolist") else value
        ):
            flatten(item, f"{path}[{index}]", out)
    else:
        out.append((path, value))
    return out


def leaves(message):
    return flatten(message_to_ordereddict(message))


def differs(a, b, tolerance):
    """None when equal, else a description of the first difference."""
    la, lb = leaves(a), leaves(b)
    if [p for p, _ in la] != [p for p, _ in lb]:
        return "different fields"
    for (path, x), (_, y) in zip(la, lb, strict=True):
        if isinstance(x, (int, float)) and not isinstance(x, bool):
            if x == y or (math.isnan(x) and math.isnan(y)):
                continue
            if math.isinf(x) or math.isinf(y) or abs(x - y) > tolerance:
                return f"{path}: {x!r} vs {y!r}"
        elif x != y:
            return f"{path}: {x!r} vs {y!r}"
    return None


def first_difference(first, second, tolerance):
    """(index, description) of the first pair that differs, else None."""
    for index, (a, b) in enumerate(zip(first, second, strict=False)):
        why = differs(a, b, tolerance)
        if why:
            return index, why
    return None
