"""Tests for scripts.changed_scope: what `check.sh --fast` scopes to."""

from __future__ import annotations

import subprocess

import pytest

from scripts.changed_scope import (
    Scope,
    changed_paths,
    render_shell,
    resolve,
    scope_for,
)


def test_check_1010_python_only_change_scopes_to_one_package() -> None:
    """CHECK-1010: a Python-only change scopes to one package, not the
    whole tree."""
    scope = scope_for(["ros_ws/src/racing_bringup/launch/x.py"])

    assert scope.whole_tree is False
    assert scope.packages == frozenset({"racing_bringup"})
    assert scope.cpp == ()
    assert scope.py == ("ros_ws/src/racing_bringup/launch/x.py",)


def test_check_1020_cpp_change_scopes_to_one_package() -> None:
    """CHECK-1020: a C++ header change scopes to its package only."""
    path = "ros_ws/src/racing_metrics/include/racing_metrics/x.hpp"
    scope = scope_for([path])

    assert scope.whole_tree is False
    assert scope.packages == frozenset({"racing_metrics"})
    assert scope.cpp == (path,)


def test_check_1030_racing_common_change_does_not_expand_dependents() -> None:
    """CHECK-1030: scope_for names racing_common only; colcon expands
    dependents."""
    scope = scope_for(["ros_ws/src/racing_common/src/track.cpp"])

    assert scope.packages == frozenset({"racing_common"})


@pytest.mark.parametrize(
    "path",
    [
        "scripts/check.sh",
        ".pre-commit-config.yaml",
        "setup.sh",
        ".clang-tidy",
        "ros_ws/src/racing_bringup/package.xml",
        "ros_ws/src/racing_bringup/CMakeLists.txt",
        "docker/Dockerfile",
        "config/reference_stack.yaml",
        "tests/golden/baseline.json",
    ],
)
def test_check_1040_whole_tree_triggers(path: str) -> None:
    """CHECK-1040: each whole-tree trigger forces whole_tree, one case
    per entry."""
    scope = scope_for([path])

    assert scope.whole_tree is True


def test_check_1050_empty_changed_set_is_whole_tree() -> None:
    """CHECK-1050: an empty changed set must never mean 'lint nothing'."""
    scope = scope_for([])

    assert scope.whole_tree is True


def test_check_1060_paths_outside_ros_ws_src() -> None:
    """CHECK-1060: sim/ contributes py but no package; docs/ contributes
    neither."""
    scope = scope_for(["sim/rollout.py", "docs/agents/testing.md"])

    assert scope.whole_tree is False
    assert scope.packages == frozenset()
    assert scope.py == ("sim/rollout.py",)
    assert scope.cpp == ()


def test_check_1070b_shallow_ros_ws_src_path_names_no_package() -> None:
    """L2: a path at ros_ws/src/<file> depth (no package segment) names no
    package, e.g. a stray ros_ws/src/.gitkeep."""
    scope = scope_for(["ros_ws/src/.gitkeep"])

    assert scope.whole_tree is False
    assert scope.packages == frozenset()


def test_check_1070_near_misses_do_not_force_whole_tree() -> None:
    """CHECK-1070: near-miss paths do not trip the whole-tree triggers."""
    scope = scope_for(
        [
            "ros_ws/src/racing_bringup/config/x.yaml",
            "docs/config/x.md",
            "scripts/check.sh.bak",
        ]
    )

    assert scope.whole_tree is False


def _fake_run_git(calls: dict[tuple[str, ...], str]):
    def run_git(args: list[str]) -> str:
        return calls[tuple(args)]

    return run_git


def test_check_1080_changed_paths_is_deduplicated_union() -> None:
    """CHECK-1080: changed_paths unions merge-base diff, cached diff and
    untracked files; resolve drops a non-existent cpp path but keeps its
    package."""
    base = "abc123"
    run_git = _fake_run_git(
        {
            ("merge-base", "main", "HEAD"): base + "\n",
            ("diff", "--name-only", "--no-renames", base): "a.py\nshared.py\n",
            (
                "diff",
                "--name-only",
                "--no-renames",
                "--cached",
                base,
            ): "shared.py\nb.cpp\n",
            ("ls-files", "--others", "--exclude-standard"): "c.py\n",
        }
    )

    result = changed_paths(run_git)

    assert sorted(result) == ["a.py", "b.cpp", "c.py", "shared.py"]

    missing_cpp = "ros_ws/src/racing_metrics/src/gone.cpp"
    run_git2 = _fake_run_git(
        {
            ("merge-base", "main", "HEAD"): base + "\n",
            ("diff", "--name-only", "--no-renames", base): missing_cpp,
            ("diff", "--name-only", "--no-renames", "--cached", base): "",
            ("ls-files", "--others", "--exclude-standard"): "",
        }
    )
    scope, _reason = resolve({}, run_git2, exists=lambda p: False)

    assert scope.whole_tree is False
    assert scope.cpp == ()
    assert scope.packages == frozenset({"racing_metrics"})


def test_check_1090_git_failure_and_check_scope_all_force_whole_tree() -> None:
    """CHECK-1090: a git failure and CHECK_SCOPE=all both force whole_tree;
    CHECK_SCOPE=all never calls git."""

    def raising_run_git(args: list[str]) -> str:
        raise subprocess.CalledProcessError(1, args)

    scope, reason = resolve({}, raising_run_git, exists=lambda p: True)

    assert scope.whole_tree is True
    assert "git" in reason.lower()

    def unexpected_run_git(args: list[str]) -> str:
        raise AssertionError("git should not be called when CHECK_SCOPE=all")

    scope, reason = resolve(
        {"CHECK_SCOPE": "all"}, unexpected_run_git, exists=lambda p: True
    )

    assert scope.whole_tree is True


def test_check_1100_render_shell() -> None:
    """CHECK-1100: render_shell emits scope_colcon_select with
    --packages-above only, quotes paths with spaces, and marks whole-tree
    scopes."""
    scope = Scope(
        whole_tree=False,
        packages=frozenset({"racing_common", "racing_metrics"}),
        cpp=(),
        py=(),
    )
    rendered = render_shell(scope, "reason with space")

    select_line = next(
        line for line in rendered.splitlines() if "scope_colcon_select=" in line
    )
    assert "--packages-select" not in select_line
    assert set(
        select_line.split("--packages-above ")[1].strip("()").split()
    ) == {
        "racing_common",
        "racing_metrics",
    }
    assert (
        "'reason with space'" in rendered or '"reason with space"' in rendered
    )

    whole = render_shell(
        Scope(whole_tree=True, packages=frozenset(), cpp=(), py=()), "mode --ci"
    )
    assert "scope_whole_tree=1" in whole


def test_check_1110_moving_a_file_out_of_a_package_keeps_the_package(
    tmp_path,
) -> None:
    """CHECK-1110: git's rename detection lists only a moved file's new
    path; the package it left must still be in scope. Real git, because
    the fake cannot express rename detection."""

    def git(*args: str) -> str:
        return subprocess.run(
            ["git", "-C", str(tmp_path), *args],
            check=True,
            capture_output=True,
            text=True,
        ).stdout

    git("init", "-q", "-b", "main")
    git("config", "user.email", "t@example.com")
    git("config", "user.name", "t")
    git("config", "commit.gpgsign", "false")
    src = tmp_path / "ros_ws/src/racing_sim_gym_jax/laser.py"
    src.parent.mkdir(parents=True)
    src.write_text("".join(f"line {i}\n" for i in range(40)))
    git("add", "-A")
    git("commit", "-q", "-m", "base")
    git("checkout", "-q", "-b", "feature")
    (tmp_path / "sim").mkdir()
    git("mv", "ros_ws/src/racing_sim_gym_jax/laser.py", "sim/laser.py")
    git("commit", "-q", "-m", "move")

    scope, _reason = resolve({}, lambda args: git(*args), exists=lambda p: True)

    assert scope.whole_tree is False
    assert scope.packages == frozenset({"racing_sim_gym_jax"})
