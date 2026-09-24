"""Tests for scripts.changed_scope: what `check.sh --fast` scopes to."""

from __future__ import annotations

import pytest

from scripts.changed_scope import scope_for


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
