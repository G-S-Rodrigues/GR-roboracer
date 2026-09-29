#!/usr/bin/env python3
"""What `check.sh --fast` should lint and test, given the branch's changes.

`scope_for` is the pure rule set. `changed_paths`/`resolve` reach git, the
environment and the filesystem so `check.sh` gets a real scope; `render_shell`
turns that into bash `check.sh` can `eval`. See CHECK-1010..1130 and
`docs/agents/repo-gotchas.md` #23.
"""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
from collections.abc import Callable
from dataclasses import dataclass

# The extensions scripts/check.sh hands to clang-format and clang-tidy.
CPP_EXTENSIONS = (".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx")

# The spec's nine whole-tree triggers plus .clang-format and ruff.toml.
# GR-rebase/SKILL.md keeps a shorter list of five, so the two differ; see
# the open topic 2026-09-24-whole-tree-trigger-list and
# repo-gotchas.md #23.
WHOLE_TREE_FILES = (
    "scripts/check.sh",
    ".pre-commit-config.yaml",
    "setup.sh",
    ".clang-tidy",
    ".clang-format",
    "ruff.toml",
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


RunGit = Callable[[list[str]], str]


def _lines(output: str) -> list[str]:
    return [line for line in output.splitlines() if line]


def changed_paths(run_git: RunGit) -> list[str]:
    """The branch's changed set: merge-base diff, staged diff and
    untracked new files, deduplicated. `run_git` raises on failure."""
    base = run_git(["merge-base", "main", "HEAD"]).strip()
    paths: set[str] = set()
    paths.update(_lines(run_git(["diff", "--name-only", "--no-renames", base])))
    paths.update(
        _lines(
            run_git(["diff", "--name-only", "--no-renames", "--cached", base])
        )
    )
    paths.update(
        _lines(run_git(["ls-files", "--others", "--exclude-standard"]))
    )
    return sorted(paths)


def resolve(
    env: dict[str, str],
    run_git: RunGit,
    exists: Callable[[str], bool],
) -> tuple[Scope, str]:
    """The scope for this run, plus a one-line reason for the log."""
    if env.get("CHECK_SCOPE") == "all":
        return (
            Scope(whole_tree=True, packages=frozenset(), cpp=(), py=()),
            "CHECK_SCOPE=all",
        )

    try:
        changed = changed_paths(run_git)
    except subprocess.CalledProcessError as exc:
        return (
            Scope(whole_tree=True, packages=frozenset(), cpp=(), py=()),
            f"git failed: {exc}",
        )

    scope = scope_for(changed)
    if scope.whole_tree:
        return scope, "whole-tree trigger in changed set"

    scope = Scope(
        whole_tree=False,
        packages=scope.packages,
        cpp=tuple(p for p in scope.cpp if exists(p)),
        py=tuple(p for p in scope.py if exists(p)),
    )
    return scope, "scoped to changed packages"


def render_shell(scope: Scope, reason: str) -> str:
    """Render `scope` as bash `check.sh` can `eval`."""
    lines = [
        f"scope_whole_tree={1 if scope.whole_tree else 0}",
        "scope_packages=("
        + " ".join(shlex.quote(p) for p in sorted(scope.packages))
        + ")",
        "scope_cpp=(" + " ".join(shlex.quote(p) for p in scope.cpp) + ")",
        "scope_py=(" + " ".join(shlex.quote(p) for p in scope.py) + ")",
    ]
    if scope.packages:
        select = ["--packages-above", *sorted(scope.packages)]
        lines.append(
            "scope_colcon_select=("
            + " ".join(shlex.quote(a) for a in select)
            + ")"
        )
    else:
        lines.append("scope_colcon_select=()")
    lines.append(f"scope_reason={shlex.quote(reason)}")
    return "\n".join(lines) + "\n"


def main() -> None:
    def run_git(args: list[str]) -> str:
        return subprocess.run(
            ["git", *args],
            cwd=os.getcwd(),
            check=True,
            capture_output=True,
            text=True,
        ).stdout

    scope, reason = resolve(dict(os.environ), run_git, os.path.exists)
    sys.stdout.write(render_shell(scope, reason))


if __name__ == "__main__":
    main()
