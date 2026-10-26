"""A small Dockerfile reader shared by the image guards.

Not a Dockerfile parser: it joins continuation lines, splits a RUN's shell
text at operators, and groups instructions by `FROM ... AS name` stage.
"""

from __future__ import annotations

import re
import shlex
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DOCKERFILE = REPO_ROOT / "docker" / "Dockerfile"
OPERATORS = {"&&", "||", ";", "|"}
FROM_LINE = re.compile(
    r"^FROM\s+(?:--\S+\s+)*(?P<parent>\S+)(?:\s+AS\s+(?P<name>\S+))?\s*$",
    re.IGNORECASE,
)


@dataclass
class Stage:
    parent: str
    instructions: list[tuple[int, str]] = field(default_factory=list)


def instructions(path: Path = DOCKERFILE) -> list[tuple[int, str]]:
    """(first line number, text) per instruction, continuations joined."""
    out: list[tuple[int, str]] = []
    buf: list[str] = []
    start = 0
    for number, raw in enumerate(path.read_text().splitlines(), 1):
        if not buf:
            start = number
        line = raw.rstrip()
        if line.endswith("\\"):
            buf.append(line[:-1])
            continue
        buf.append(line)
        out.append((start, " ".join(buf)))
        buf = []
    return out


def segments(text: str) -> list[list[str]]:
    """Split a RUN's shell command into commands at shell operators."""
    tokens = shlex.split(text.removeprefix("RUN").strip())
    out: list[list[str]] = [[]]
    for token in tokens:
        if token in OPERATORS:
            out.append([])
        else:
            out[-1].append(token)
    return [s for s in out if s]


def stages(path: Path = DOCKERFILE) -> dict[str, Stage]:
    """Stage name -> Stage, in file order. The FROM line is not included."""
    out: dict[str, Stage] = {}
    current: Stage | None = None
    for number, text in instructions(path):
        match = FROM_LINE.match(text.strip())
        if match:
            name = match.group("name") or match.group("parent")
            current = Stage(parent=match.group("parent"))
            out[name] = current
        elif current is not None and text.strip():
            current.instructions.append((number, text))
    return out
