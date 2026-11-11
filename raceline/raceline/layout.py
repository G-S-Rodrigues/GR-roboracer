"""Where a committed raceline record lives in the data repo (ADR 0009, D2).

``<tracks>/<Map>/racelines/<optimizer>/<limit set>/`` holds ``raceline.csv``,
``raceline.yaml``, ``limits.md`` and, for ``tum_mintime``, ``reference.csv``.
The limit-set grammar is owned by the data repo's README.
"""

import re
from pathlib import Path

LIMIT_SET_RE = re.compile(
    r"^(gym|autodrive|car|[a-z0-9]+)-(nominal|model|identified|tuned)-"
    r"[a-z0-9]+-(peak|sliding|const|p\d{2})(-[a-z0-9]+)?-v[1-9]\d*$"
)
RACELINE_CSV = "raceline.csv"
RACELINE_YAML = "raceline.yaml"
REFERENCE_CSV = "reference.csv"
LIMITS_MD = "limits.md"


def check_limit_set(name: str) -> str:
    """Return ``name`` or raise ``ValueError`` if it is not a limit-set name."""
    if not LIMIT_SET_RE.fullmatch(name):
        raise ValueError(
            f"{name!r} is not a limit-set name "
            "(<sim>-<source>-<caps>-<grip>[-<qualifier>]-v<N>, e.g. "
            "gym-nominal-std-const-v1)"
        )
    return name


def record_dir(
    out_dir: Path, map_key: str, optimizer: str, limit_set: str
) -> Path:
    return Path(out_dir) / map_key / "racelines" / optimizer / limit_set
