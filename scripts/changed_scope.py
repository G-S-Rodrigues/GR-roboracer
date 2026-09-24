#!/usr/bin/env python3
"""What `check.sh --fast` should lint and test, given the branch's changes.

`scope_for` is the pure rule set. `changed_paths`/`resolve` reach git, the
environment and the filesystem so `check.sh` gets a real scope; `render_shell`
turns that into bash `check.sh` can `eval`. See CHECK-1010..1100 and
`docs/agents/repo-gotchas.md` #23.
"""

from __future__ import annotations

from dataclasses import dataclass

CPP_EXTENSIONS = (".cpp", ".hpp", ".h", ".cc", ".cxx")

# Duplicated deliberately from GR-rebase/SKILL.md's whole-tree trigger list
# (cross-repo reuse is impossible in prose); see the plan's Reuse decisions
# and repo-gotchas.md #23 for what can drift.
WHOLE_TREE_FILES = (
    "scripts/check.sh",
    ".pre-commit-config.yaml",
    "setup.sh",
    ".clang-tidy",
)
WHOLE_TREE_BASENAMES = ("package.xml", "CMakeLists.txt")
WHOLE_TREE_PREFIXES = ("docker/", "config/", "tests/golden/")


@dataclass(frozen=True)
class Scope:
    """What `--fast` should cover for one run."""

    whole_tree: bool
    packages: frozenset[str]
    cpp: tuple[str, ...]
    py: tuple[str, ...]


def _is_whole_tree_trigger(path: str) -> bool:
    if path in WHOLE_TREE_FILES:
        return True
    basename = path.rsplit("/", 1)[-1]
    if basename in WHOLE_TREE_BASENAMES:
        return True
    return any(path.startswith(prefix) for prefix in WHOLE_TREE_PREFIXES)


def _package_for(path: str) -> str | None:
    parts = path.split("/")
    if len(parts) >= 4 and parts[0] == "ros_ws" and parts[1] == "src":
        return parts[2]
    return None


def scope_for(changed_paths: list[str]) -> Scope:
    """The pure rules mapping changed repo-relative paths to a Scope."""
    if not changed_paths:
        return Scope(whole_tree=True, packages=frozenset(), cpp=(), py=())

    if any(_is_whole_tree_trigger(path) for path in changed_paths):
        return Scope(whole_tree=True, packages=frozenset(), cpp=(), py=())

    packages: set[str] = set()
    cpp: list[str] = []
    py: list[str] = []

    for path in changed_paths:
        pkg = _package_for(path)
        if pkg is not None:
            packages.add(pkg)
            if path.endswith(CPP_EXTENSIONS):
                cpp.append(path)
        if path.endswith(".py") and (
            path.startswith(("sim/", "tools/", "tests/", "scripts/"))
            or pkg is not None
        ):
            py.append(path)

    return Scope(
        whole_tree=False,
        packages=frozenset(packages),
        cpp=tuple(sorted(cpp)),
        py=tuple(sorted(py)),
    )
