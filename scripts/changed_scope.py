#!/usr/bin/env python3
"""What `check.sh --fast` should lint and test, given the branch's changes.

`scope_for` is the pure rule set. `changed_paths`/`resolve` reach git, the
environment and the filesystem so `check.sh` gets a real scope; `render_shell`
turns that into bash `check.sh` can `eval`. See CHECK-1010..1130 and
`docs/agents/repo-gotchas.md` #23.
"""

from __future__ import annotations

import argparse
import fnmatch
import os
import re
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ElementTree
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

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
            path.startswith(
                ("sim/", "tools/", "tests/", "scripts/", "raceline/")
            )
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
SYSTEM_TEST_PATH = re.compile(r"tests/system/test_sim_(\d{4})_[^/]*\.py")


def discover_system_tests(root: Path) -> dict[str, str]:
    """Tier-3 ID -> repo-relative file, from `test_sim_NNNN_*.py` names.

    Fails closed: any .py file (other than conftest/__init__) the pattern
    cannot read, or an ID found twice, raises rather than being skipped
    by the scoped run."""
    found: dict[str, str] = {}
    system = root / "tests" / "system"
    for path in sorted(system.glob("**/*.py")):
        if path.name in ("conftest.py", "__init__.py"):
            continue
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
        if not own:
            raise ValueError(f"tier-4 case without an impl: tag: {case.name}")
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
    stack = stack_from_dict(
        data, discover_system_tests(root), discover_acceptance_tags(root)
    )
    for impl in stack.implementations:
        if not impl.verified_by:
            raise ValueError(f"{impl.key} claims no test (empty verified_by)")
    return stack


# Files that are whole tree for tiers 3-4 beyond the shared triggers (D4):
# the map itself and the composition code every implementation runs inside.
READY_WHOLE_TREE_FILES = (
    "config/reference_stack.yaml",
    "ros_ws/src/racing_bringup/launch/sim_pure_pursuit.launch.py",
    "ros_ws/src/racing_bringup/racing_bringup/support_node.py",
    "ros_ws/src/racing_bringup/racing_bringup/scenario_parameters.py",
    "ros_ws/src/racing_bringup/setup.py",
)


@dataclass(frozen=True)
class ReadyScope:
    """What `--ready` should run in tiers 3-4. Under `whole_tree` the
    other fields are empty, because the caller runs everything."""

    whole_tree: bool
    reason: str
    implementations: tuple[str, ...]
    tests: tuple[str, ...]
    why: tuple[str, ...]


def _whole(reason: str) -> ReadyScope:
    return ReadyScope(
        whole_tree=True,
        reason=reason,
        implementations=(),
        tests=(),
        why=(),
    )


def _matches(path: str, globs: tuple[str, ...]) -> bool:
    return any(fnmatch.fnmatchcase(path, glob) for glob in globs)


def select_ready(
    changed: list[str],
    stack: StackMap,
    deps: Mapping[str, set[str]],
) -> ReadyScope:
    """The tier-3/4 tests `--ready` runs for a changed set (ADR 0007).

    Rules apply per path, first match decides: whole-tree files and
    triggers, neutral paths, a changed tier-3 file, then implementation
    `paths`. Anything unplaced is whole tree - the map fails closed."""
    if not changed:
        return _whole("empty changed set")
    if not stack.reference_lap:
        return _whole("reference_stack.yaml names no reference_lap")

    selected: set[str] = set()
    explicit: set[str] = set()
    why: list[str] = []
    for path in changed:
        if path in READY_WHOLE_TREE_FILES:
            return _whole(f"whole-tree file: {path}")
        if not path.startswith("config/") and _is_whole_tree_trigger(path):
            return _whole(f"whole-tree trigger: {path}")
        if _matches(path, stack.neutral_paths):
            continue
        system = SYSTEM_TEST_PATH.fullmatch(path)
        if system is not None:
            test_id = f"SIM-{system.group(1)}"
            explicit.add(test_id)
            why.append(f"{path} -> {test_id}")
            continue
        owners = [i for i in stack.implementations if _matches(path, i.paths)]
        if not owners:
            kind = "config path" if path.startswith("config/") else "path"
            return _whole(f"unmapped {kind}: {path}")
        package = _package_for(path)
        dependents = deps.get(package, set()) if package else set()
        for impl in stack.implementations:
            if impl in owners or impl.package in dependents:
                selected.add(impl.key)
                why.append(f"{path} -> {impl.key}")

    claimed = {t for i in stack.implementations for t in i.verified_by}
    discovered = set(stack.system_tests) | set(stack.acceptance_tags)
    for test_id in sorted(discovered - claimed - {stack.reference_lap}):
        return _whole(f"unclaimed test: {test_id}")

    chosen = {stack.reference_lap} | explicit
    for impl in stack.implementations:
        if impl.key in selected:
            chosen.update(impl.verified_by)
    for test_id in sorted(chosen - discovered):
        return _whole(f"claimed test not found: {test_id}")

    return ReadyScope(
        whole_tree=False,
        reason="scoped to the map",
        implementations=tuple(sorted(selected)),
        tests=tuple(sorted(chosen)),
        why=tuple(sorted(set(why))),
    )


def owners_of(test_id: str, stack: StackMap) -> list[str]:
    """Keys of the implementations answerable for `test_id`. The reference
    lap exercises every default, so it names them all (D7)."""
    if test_id == stack.reference_lap:
        return sorted(i.key for i in stack.implementations if i.default)
    return sorted(
        i.key for i in stack.implementations if test_id in i.verified_by
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


LoadStack = Callable[[], tuple[StackMap, Mapping[str, set[str]]]]


def resolve_ready(
    env: dict[str, str],
    run_git: RunGit,
    load: LoadStack,
) -> tuple[ReadyScope, StackMap | None]:
    """The tier-3/4 selection for this run. Every way of not knowing -
    CHECK_SCOPE=all, git failing, an empty set, an unreadable map - is
    whole tree, never a smaller run."""
    if env.get("CHECK_SCOPE") == "all":
        return _whole("CHECK_SCOPE=all"), None
    try:
        changed = changed_paths(run_git)
    except subprocess.CalledProcessError as exc:
        return _whole(f"git failed: {exc}"), None
    try:
        stack, deps = load()
    except Exception as exc:  # yaml, robot import, missing file, ValueError
        return _whole(f"reference_stack.yaml unreadable: {exc}"), None
    return select_ready(changed, stack, deps), stack


def render_ready_shell(scope: ReadyScope, stack: StackMap | None) -> str:
    """Render `scope` as bash `check.sh` can `eval`."""
    files: list[str] = []
    tags: list[str] = []
    if not scope.whole_tree and stack is not None:
        files = [
            stack.system_tests[t]
            for t in scope.tests
            if t in stack.system_tests
        ]
        tags = sorted(
            impl.tag
            for impl in stack.implementations
            if impl.key in scope.implementations
            and any(t in stack.acceptance_tags for t in impl.verified_by)
        )
        line = (
            f"implementations {', '.join(scope.implementations) or 'none'}; "
            f"tests {', '.join(scope.tests)}; "
            f"because {'; '.join(scope.why) or 'no mapped path'}"
        )
    else:
        line = f"whole tree ({scope.reason})"
    return (
        "\n".join(
            [
                f"ready_whole_tree={1 if scope.whole_tree else 0}",
                f"ready_line={shlex.quote(line)}",
                "ready_system_tests=("
                + " ".join(shlex.quote(f) for f in files)
                + ")",
                "ready_robot_tags=("
                + " ".join(shlex.quote(t) for t in tags)
                + ")",
            ]
        )
        + "\n"
    )


@dataclass(frozen=True)
class GitDirLocation:
    """A worktree's git dir found by `locate_git_dir`."""

    git_dir: Path
    work_tree: Path
    gitfile_target: str


def _default_owner(path: Path) -> int:
    return path.stat().st_uid


def locate_git_dir(
    start: Path,
    env: Mapping[str, str],
    *,
    owner: Callable[[Path], int] = _default_owner,
) -> GitDirLocation | None:
    """Find the git dir of the worktree holding `start` when its `.git` file
    cannot be followed (it names a host path, or git distrusts the owner).

    Explicit `GIT_DIR` skips git's ownership check, so trust is only
    extended from a common repo `A` that git itself opens, to a worktree
    nested inside it: `A/.git/worktrees/<name>` whose back-pointer names this
    worktree at the same position under `A`. Anything else is None, and the
    caller runs plain git (which fails closed to whole tree)."""
    if "GIT_DIR" in env:
        return None
    try:
        start = start.absolute()
        work = next(
            (d for d in (start, *start.parents) if (d / ".git").exists()),
            None,
        )
        if work is None or (work / ".git").is_dir():
            return None
        first = (work / ".git").read_text().splitlines()[:1]
        if not first or not first[0].startswith("gitdir: "):
            return None
        target = first[0][len("gitdir: ") :].strip()
        name = PurePosixPath(target).name
        target_abs = (
            Path(target) if Path(target).is_absolute() else work / target
        )
        found = None
        for common in work.parents:
            cand = common / ".git" / "worktrees" / name
            if not (cand.is_dir() and (cand / "gitdir").is_file()):
                continue
            back = PurePosixPath((cand / "gitdir").read_text().strip())
            rel = work.relative_to(common).parts
            if back.parts[-len(rel) - 1 :] == (*rel, ".git"):
                found = (common, cand)
                break
        if found is None:
            return None
        common, cand = found
        if target_abs.exists() and target_abs.resolve() != cand.resolve():
            return None
        # Trust gate: all of it before any git runs with GIT_DIR.
        common_git = (common / ".git").resolve()
        probe = subprocess.run(
            ["git", "-C", str(common), "rev-parse", "--absolute-git-dir"],
            env=dict(env),
            capture_output=True,
            text=True,
        )
        if probe.returncode != 0:
            return None
        if Path(probe.stdout.strip()).resolve() != common_git:
            return None
        if not (cand / "commondir").is_file():
            return None
        pointed = (cand / (cand / "commondir").read_text().strip()).resolve()
        if pointed != common_git:
            return None
        want = owner(common / ".git")
        for path in (
            cand,
            cand / "commondir",
            cand / "gitdir",
            work / ".git",
        ):
            if owner(path) != want:
                return None
        return GitDirLocation(
            git_dir=cand, work_tree=work, gitfile_target=target
        )
    except (OSError, ValueError, UnicodeDecodeError):
        return None


def make_run_git(
    cwd: Path, env: Mapping[str, str]
) -> tuple[RunGit, GitDirLocation | None]:
    """The `run_git` for this process, and where it found the git dir when
    the worktree's own `.git` file could not be followed (else None)."""
    location = locate_git_dir(cwd, env)
    git_env = dict(env)
    if location is not None:
        git_env["GIT_DIR"] = str(location.git_dir)
        git_env["GIT_WORK_TREE"] = str(location.work_tree)

    def run_git(git_args: list[str]) -> str:
        return subprocess.run(
            ["git", *git_args],
            cwd=cwd,
            env=git_env,
            check=True,
            capture_output=True,
            text=True,
        ).stdout

    return run_git, location


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--ready", action="store_true", help="print the --ready selection"
    )
    group.add_argument(
        "--owners", metavar="ID", help="print the implementations owning ID"
    )
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]

    run_git, location = make_run_git(Path.cwd(), os.environ)
    if location is not None:
        sys.stderr.write(
            f"changed_scope: git dir {location.git_dir} "
            f"(the worktree's .git names {location.gitfile_target})\n"
        )

    if args.owners is not None:
        owners = owners_of(args.owners, load_stack(root))
        if not owners:
            sys.stderr.write(f"{args.owners}: claimed by no implementation\n")
            return 1
        sys.stdout.write("\n".join(owners) + "\n")
        return 0

    if args.ready:

        def load() -> tuple[StackMap, Mapping[str, set[str]]]:
            return load_stack(root), reverse_dependents(read_manifests(root))

        ready, stack = resolve_ready(dict(os.environ), run_git, load)
        sys.stdout.write(render_ready_shell(ready, stack))
        return 0

    scope, reason = resolve(dict(os.environ), run_git, os.path.exists)
    sys.stdout.write(render_shell(scope, reason))
    return 0


if __name__ == "__main__":
    sys.exit(main())
