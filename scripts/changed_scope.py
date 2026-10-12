#!/usr/bin/env python3
"""What `check.sh --fast` should lint and test, given the branch's changes.

`scope_for` is the pure rule set. `changed_paths`/`resolve` reach git, the
environment and the filesystem so `check.sh` gets a real scope; `render_shell`
turns that into bash `check.sh` can `eval`. See CHECK-1010..1130 and
`docs/agents/repo-gotchas.md` #23.
"""

from __future__ import annotations

import os
import re
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ElementTree
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path

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


DEPEND_TAGS = (
    "depend",
    "build_depend",
    "build_export_depend",
    "exec_depend",
    "test_depend",
)


def read_manifests(root: Path) -> dict[str, set[str]]:
    """Package name -> in-tree packages it depends on, from every
    `ros_ws/src/*/package.xml`. Parsed here, not asked of colcon, because
    this runs before ROS is sourced (D9); CHECK-1140 pins it to colcon."""
    declared: dict[str, set[str]] = {}
    for manifest in sorted((root / "ros_ws" / "src").glob("*/package.xml")):
        tree = ElementTree.parse(manifest).getroot()
        name = (tree.findtext("name") or "").strip()
        wanted = {
            (element.text or "").strip()
            for tag in DEPEND_TAGS
            for element in tree.findall(tag)
        }
        declared[name] = wanted
    return {
        name: {dep for dep in deps if dep in declared and dep != name}
        for name, deps in declared.items()
    }


def reverse_dependents(
    manifests: dict[str, set[str]],
) -> dict[str, set[str]]:
    """Package -> every package that depends on it, transitively."""
    direct: dict[str, set[str]] = {name: set() for name in manifests}
    for name, deps in manifests.items():
        for dep in deps:
            direct[dep].add(name)
    closure: dict[str, set[str]] = {}
    for name in manifests:
        seen: set[str] = set()
        pending = list(direct[name])
        while pending:
            current = pending.pop()
            if current not in seen:
                seen.add(current)
                pending.extend(direct[current])
        seen.discard(name)
        closure[name] = seen
    return closure


@dataclass(frozen=True)
class Implementation:
    """One implementation of one role in config/reference_stack.yaml."""

    role: str
    name: str
    package: str
    default: bool
    paths: tuple[str, ...]
    verified_by: tuple[str, ...]  # tier-3/4 IDs; a YAML scalar is one ID

    @property
    def key(self) -> str:
        return f"{self.role}={self.name}"

    @property
    def tag(self) -> str:
        """The Robot tag this implementation's tier-4 cases carry."""
        return f"impl:{self.key}"


@dataclass(frozen=True)
class StackMap:
    """The reference stack plus the tests discovered on disk."""

    implementations: tuple[Implementation, ...]
    reference_lap: str
    neutral_paths: tuple[str, ...]
    system_tests: Mapping[str, str]  # SIM-3080 -> tests/system/test_sim_...py
    acceptance_tags: Mapping[str, frozenset[str]]  # ACC-4030 -> impl: tags


def _as_tuple(value: object) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    return tuple(str(item) for item in value)  # type: ignore[attr-defined]


def stack_from_dict(
    data: Mapping[str, object],
    system_tests: Mapping[str, str],
    acceptance_tags: Mapping[str, frozenset[str]],
) -> StackMap:
    """Build a StackMap from parsed YAML and already-discovered tests."""
    implementations: list[Implementation] = []
    roles = data["roles"]
    assert isinstance(roles, Mapping)
    for role, entry in roles.items():
        entries = [
            (entry, True),
            *[(a, False) for a in entry.get("alternatives", ())],
        ]
        for fields, default in entries:
            implementations.append(
                Implementation(
                    role=role,
                    name=fields["implementation"],
                    package=fields["package"],
                    default=default,
                    paths=_as_tuple(fields.get("paths")),
                    verified_by=_as_tuple(fields.get("verified_by")),
                )
            )
    return StackMap(
        implementations=tuple(implementations),
        reference_lap=str(data.get("reference_lap", "")),
        neutral_paths=_as_tuple(data.get("neutral_paths")),
        system_tests=dict(system_tests),
        acceptance_tags=dict(acceptance_tags),
    )


SYSTEM_TEST_NAME = re.compile(r"test_sim_(\d{4})_[^/]*\.py")
ACCEPTANCE_ID = re.compile(r"ACC-4\d{3}")


def discover_system_tests(root: Path) -> dict[str, str]:
    """Tier-3 ID -> repo-relative file, from `test_sim_NNNN_*.py` names.

    Fails closed: a test file the pattern cannot read, or an ID found
    twice, raises rather than being skipped by the scoped run."""
    found: dict[str, str] = {}
    system = root / "tests" / "system"
    for path in sorted(system.glob("**/test_*.py")):
        relative = path.relative_to(root).as_posix()
        match = SYSTEM_TEST_NAME.fullmatch(path.name)
        if match is None or path.parent != system:
            raise ValueError(
                f"tier-3 test not discoverable as SIM-NNNN: {relative}"
            )
        test_id = f"SIM-{match.group(1)}"
        if test_id in found:
            raise ValueError(
                f"duplicate {test_id}: {found[test_id]}, {relative}"
            )
        found[test_id] = relative
    return found


def discover_acceptance_tags(root: Path) -> dict[str, frozenset[str]]:
    """Tier-4 ID -> union of its cases' `impl:` tags, via Robot's parser."""
    directory = root / "tests" / "acceptance"
    if not directory.is_dir():
        return {}
    from robot.api import TestSuiteBuilder  # lazy: not needed by --fast

    suite = TestSuiteBuilder().build(str(directory))
    tags: dict[str, frozenset[str]] = {}
    for case in suite.all_tests:
        match = ACCEPTANCE_ID.search(case.doc or "")
        if match is None:
            raise ValueError(f"tier-4 case without an ACC-4NNN ID: {case.name}")
        test_id = match.group(0)
        own = frozenset(str(t) for t in case.tags if str(t).startswith("impl:"))
        if test_id in tags and tags[test_id] & own:
            raise ValueError(
                f"duplicate {test_id} with overlapping tags: {case.name}"
            )
        tags[test_id] = tags.get(test_id, frozenset()) | own
    return tags


def load_stack(root: Path) -> StackMap:
    """config/reference_stack.yaml plus the tests discovered under root."""
    import yaml  # lazy: only --ready and its tests need it

    data = yaml.safe_load(
        (root / "config" / "reference_stack.yaml").read_text(encoding="utf-8")
    )
    return stack_from_dict(
        data, discover_system_tests(root), discover_acceptance_tags(root)
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
