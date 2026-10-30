"""Tests for scripts.changed_scope: what `check.sh --fast` scopes to."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest

from scripts.changed_scope import (
    READY_WHOLE_TREE_FILES,
    ReadyScope,
    Scope,
    StackMap,
    changed_paths,
    load_stack,
    owners_of,
    read_manifests,
    render_ready_shell,
    render_shell,
    resolve,
    resolve_ready,
    reverse_dependents,
    scope_for,
    select_ready,
    stack_from_dict,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


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


def test_check_1290_raceline_python_is_linted_with_no_package() -> None:
    """CHECK-1290: raceline/ is a Python root with no ROS package, so a
    raceline/*.py edit is linted under --fast and is not whole tree."""
    scope = scope_for(
        [
            "raceline/raceline/tum.py",
            "raceline/tests/test_smoke.py",
            "raceline/README.md",
        ]
    )

    assert scope.whole_tree is False
    assert scope.packages == frozenset()
    assert scope.py == (
        "raceline/raceline/tum.py",
        "raceline/tests/test_smoke.py",
    )


_ABSENT = object()  # a git call that exits 1 (e.g. a missing ref)
_FAILED = object()  # a git call that exits 128
_PROBE = ("show-ref", "--verify", "--quiet", "refs/remotes/origin/main")


def _fake_run_git(calls: dict[tuple[str, ...], object]):
    seen: list[tuple[str, ...]] = []

    def run_git(args: list[str]) -> str:
        seen.append(tuple(args))
        value = calls[tuple(args)]
        if value is _ABSENT:
            raise subprocess.CalledProcessError(1, args)
        if value is _FAILED:
            raise subprocess.CalledProcessError(128, args)
        return value  # type: ignore[return-value]

    run_git.seen = seen  # type: ignore[attr-defined]
    return run_git


def test_check_1080_changed_paths_is_deduplicated_union() -> None:
    """CHECK-1080: changed_paths unions merge-base diff, cached diff and
    untracked files; resolve drops a non-existent cpp path but keeps its
    package."""
    base = "abc123"
    run_git = _fake_run_git(
        {
            _PROBE: _ABSENT,
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
            _PROBE: _ABSENT,
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


@pytest.mark.parametrize(
    "ext", [".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"]
)
def test_check_1120_every_extension_check_sh_formats_is_cpp(
    ext: str,
) -> None:
    """CHECK-1120: the classifier matches the extensions check.sh hands
    to clang-format and clang-tidy."""
    path = f"ros_ws/src/racing_common/src/x{ext}"

    scope = scope_for([path])

    assert scope.packages == frozenset({"racing_common"})
    assert scope.cpp == (path,)


@pytest.mark.parametrize("path", [".clang-format", "ruff.toml"])
def test_check_1130_lint_config_files_are_whole_tree_triggers(
    path: str,
) -> None:
    """CHECK-1130: a change to only the formatter or linter config runs
    the whole tree."""
    assert scope_for([path]).whole_tree is True


def test_check_1140_reverse_closure_matches_colcon() -> None:
    """CHECK-1140: the reverse dependency closure parsed from package.xml
    is what `colcon --packages-above` selects (D9), so the parse that runs
    before ROS is sourced cannot drift from colcon's own graph."""
    assert reverse_dependents({"a": set(), "b": {"a"}, "c": {"b"}}) == {
        "a": {"b", "c"},
        "b": {"c"},
        "c": set(),
    }

    assert shutil.which("colcon") is not None, (
        "CHECK-1140 needs colcon on PATH to cross-check the package.xml "
        "graph; run scripts/tests inside the dev container"
    )
    manifests = read_manifests(REPO_ROOT)
    dependents = reverse_dependents(manifests)
    assert set(manifests) == set(dependents) and manifests
    for package in manifests:
        listed = subprocess.run(
            [
                "colcon",
                "list",
                "--base-paths",
                "ros_ws/src",
                "--packages-above",
                package,
                "--names-only",
            ],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.split()
        assert {package} | dependents[package] == set(listed), package


SYNTHETIC_MAP = {
    "schema_version": 1,
    "reference_lap": "SIM-3020",
    "neutral_paths": ["docs/*"],
    "roles": {
        "controller": {
            "implementation": "pp",
            "package": "pkg_pp",
            "paths": ["ros_ws/src/pkg_pp/*"],
            "verified_by": ["SIM-3030"],
        },
        "pose_source": {
            "implementation": "truth",
            "package": "pkg_truth",
            "alternatives": [
                {
                    "implementation": "amcl",
                    "package": "nav2_amcl",
                    "paths": ["config/loc/amcl.yaml"],
                    "verified_by": "SIM-3080",
                }
            ],
        },
    },
}


def test_check_1150_the_real_map_loads_with_its_shape() -> None:
    """CHECK-1150: the real map loads; each implementation has its key and
    its derived Robot tag; discovery reads tier-3 IDs from file names and
    tier-4 IDs and tags from racing.robot."""
    stack = load_stack(REPO_ROOT)

    keys = {impl.key for impl in stack.implementations}
    assert len(stack.implementations) == 11
    assert "planner=optimized_raceline" in keys
    assert "pose_source=amcl" in keys
    assert "pose_source=slam_toolbox" in keys
    assert "pose_source=mola" in keys
    amcl = next(i for i in stack.implementations if i.name == "amcl")
    assert amcl.tag == "impl:pose_source=amcl"
    assert amcl.default is False
    assert set(amcl.verified_by) == {"SIM-3080", "ACC-4030", "ACC-4040"}
    assert set(stack.system_tests) == {
        f"SIM-{n}" for n in range(3010, 3111, 10)
    } | {"SIM-3510", "SIM-3530", "SIM-3610"}
    assert set(stack.acceptance_tags) == {
        "ACC-4010",
        "ACC-4020",
        "ACC-4030",
        "ACC-4040",
        "ACC-4510",
    }


def test_check_1150_synthetic_map_reads_scalars_and_defaults() -> None:
    """CHECK-1150: a scalar verified_by is one ID, a missing field reads as
    empty, and alternatives are not defaults."""
    stack = stack_from_dict(
        SYNTHETIC_MAP,
        {"SIM-3030": "t30.py", "SIM-3080": "t80.py"},
        {"ACC-4030": frozenset({"impl:pose_source=amcl"})},
    )

    assert isinstance(stack, StackMap)
    by_key = {i.key: i for i in stack.implementations}
    assert by_key["pose_source=amcl"].verified_by == ("SIM-3080",)
    assert by_key["pose_source=amcl"].default is False
    assert by_key["pose_source=truth"].verified_by == ()
    assert by_key["pose_source=truth"].paths == ()
    assert by_key["controller=pp"].default is True
    assert stack.neutral_paths == ("docs/*",)


def _layout(tmp_path: Path, robot: str | None = None, **tests: str) -> Path:
    """A minimal repo: the map, tier-3 files named by `tests` (relative
    name -> contents) and optionally a tier-4 suite."""
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "reference_stack.yaml").write_text(
        "schema_version: 1\nreference_lap: SIM-3020\nroles:\n"
        "  controller:\n    implementation: pp\n    package: pkg_pp\n"
        "    verified_by: [SIM-3020]\n",
        encoding="utf-8",
    )
    (tmp_path / "tests" / "system").mkdir(parents=True)
    (tmp_path / "tests" / "system" / "test_sim_3020_ok.py").write_text("")
    for name, body in tests.items():
        target = tmp_path / "tests" / "system" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(body)
    if robot is not None:
        (tmp_path / "tests" / "acceptance").mkdir(parents=True)
        (tmp_path / "tests" / "acceptance" / "t.robot").write_text(robot)
    return tmp_path


ROBOT_HEAD = "*** Test Cases ***\n"


def test_check_1280_clean_layout_loads(tmp_path: Path) -> None:
    """CHECK-1280: the control for the cases below - a well-formed layout
    loads, and ACC-4030's two cases with disjoint tags are legal."""
    root = _layout(
        tmp_path,
        ROBOT_HEAD
        + "A\n    [Documentation]    ACC-4030\n    [Tags]    impl:x=a\n"
        "    Log    a\n"
        "B\n    [Documentation]    ACC-4030\n    [Tags]    impl:x=b\n"
        "    Log    b\n",
    )

    stack = load_stack(root)

    assert stack.acceptance_tags["ACC-4030"] == frozenset(
        {"impl:x=a", "impl:x=b"}
    )
    assert stack.system_tests == {
        "SIM-3020": "tests/system/test_sim_3020_ok.py"
    }


@pytest.mark.parametrize(
    ("files", "robot", "needle"),
    [
        ({"test_foo.py": ""}, None, "test_foo.py"),
        ({"sub/test_sim_3990_x.py": ""}, None, "test_sim_3990_x.py"),
        ({"test_sim_3020_again.py": ""}, None, "SIM-3020"),
        (
            {},
            ROBOT_HEAD + "C\n    [Documentation]    none\n    Log    c\n",
            "C",
        ),
        (
            {},
            ROBOT_HEAD
            + "A\n    [Documentation]    ACC-4030\n    [Tags]    impl:x=a\n"
            "    Log    a\n"
            "B\n    [Documentation]    ACC-4030\n    [Tags]    impl:x=a\n"
            "    Log    b\n",
            "ACC-4030",
        ),
        (
            {},
            ROBOT_HEAD
            + "A\n    [Documentation]    ACC-4030\n    [Tags]    impl:x=a\n"
            "    Log    a\n"
            "B\n    [Documentation]    ACC-4030\n    Log    b\n",
            "impl:",
        ),
        ({"foo_test.py": ""}, None, "foo_test.py"),
        ({"helper.py": ""}, None, "helper.py"),
    ],
    ids=[
        "no-id-file",
        "nested-file",
        "duplicate-sim",
        "no-acc-id",
        "dup-acc",
        "no-impl-tag",
        "suffix-test-file",
        "other-py-file",
    ],
)
def test_check_1280_discovery_fails_closed(
    tmp_path: Path, files: dict[str, str], robot: str | None, needle: str
) -> None:
    """CHECK-1280: a test discovery cannot see, or sees twice, raises
    ValueError naming it, so resolve_ready can run the whole tree instead
    of silently skipping it (R9)."""
    root = _layout(tmp_path, robot, **files)

    with pytest.raises(ValueError, match=needle):
        load_stack(root)


def test_check_1280_conftest_and_init_are_not_tests(tmp_path: Path) -> None:
    """CHECK-1280: the two files pytest collects nothing from are skipped."""
    root = _layout(tmp_path, None, **{"conftest.py": "", "__init__.py": ""})

    assert set(load_stack(root).system_tests) == {"SIM-3020"}


def test_check_1280_an_implementation_without_verified_by_raises(
    tmp_path: Path,
) -> None:
    """CHECK-1280: an implementation claiming no test would be selected by
    no change to it, so load_stack rejects the map (C3)."""
    root = _layout(tmp_path)
    path = root / "config" / "reference_stack.yaml"
    path.write_text(
        path.read_text(encoding="utf-8").replace(
            "    verified_by: [SIM-3020]\n", ""
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="pp"):
        load_stack(root)


SELECT_MAP = {
    "schema_version": 1,
    "reference_lap": "SIM-3020",
    "neutral_paths": ["docs/*", "README.md", "scripts/tests/*"],
    "roles": {
        "controller": {
            "implementation": "pp",
            "package": "pkg_pp",
            "paths": ["ros_ws/src/pkg_pp/*"],
            "verified_by": ["SIM-3030", "ACC-4020"],
        },
        "pose_source": {
            "implementation": "truth",
            "package": "pkg_truth",
            "paths": ["ros_ws/src/pkg_truth/*"],
            "verified_by": ["SIM-3040"],
            "alternatives": [
                {
                    "implementation": "amcl",
                    "package": "nav2_amcl",
                    "paths": ["config/loc/amcl.yaml", "ros_ws/src/pkg_sim/*"],
                    "verified_by": ["SIM-3080", "ACC-4030"],
                },
                {
                    "implementation": "slam",
                    "package": "slam_toolbox",
                    "paths": ["config/loc/slam.yaml", "ros_ws/src/pkg_sim/*"],
                    "verified_by": ["SIM-3070", "ACC-4030"],
                },
            ],
        },
    },
}
SELECT_SYSTEM = {
    f"SIM-{n}": f"tests/system/test_sim_{n}_x.py"
    for n in (3020, 3030, 3040, 3070, 3080)
}
SELECT_TAGS = {
    "ACC-4020": frozenset(),
    "ACC-4030": frozenset({"impl:pose_source=amcl", "impl:pose_source=slam"}),
}
SELECT_DEPS = {
    "pkg_sim": {"pkg_pp"},
    "pkg_pp": set(),
    "pkg_truth": set(),
}


def _select(paths: list[str], **overrides) -> ReadyScope:
    system = overrides.get("system", SELECT_SYSTEM)
    tags = overrides.get("tags", SELECT_TAGS)
    data = overrides.get("data", SELECT_MAP)
    return select_ready(paths, stack_from_dict(data, system, tags), SELECT_DEPS)


def test_check_1160_an_owned_path_selects_only_its_tests() -> None:
    """CHECK-1160: a path one implementation owns selects its claims plus
    the reference lap, and says why."""
    scope = _select(["config/loc/amcl.yaml"])

    assert scope.whole_tree is False
    assert scope.implementations == ("pose_source=amcl",)
    assert scope.tests == ("ACC-4030", "SIM-3020", "SIM-3080")
    assert scope.why == ("config/loc/amcl.yaml -> pose_source=amcl",)


def test_check_1170_a_shared_package_selects_its_dependents() -> None:
    """CHECK-1170: a path owned by two implementations selects both, and
    every implementation whose package depends on the changed package."""
    scope = _select(["ros_ws/src/pkg_sim/x.py"])

    assert scope.implementations == (
        "controller=pp",
        "pose_source=amcl",
        "pose_source=slam",
    )
    assert {"SIM-3070", "SIM-3080", "SIM-3030", "ACC-4030"} <= set(scope.tests)
    assert "SIM-3040" not in scope.tests


def test_check_1180_an_unmapped_path_is_whole_tree() -> None:
    """CHECK-1180: a path nothing places widens to the whole tree and the
    reason names it."""
    scope = _select(["ros_ws/src/pkg_unowned/src/x.cpp"])

    assert scope.whole_tree is True
    assert "unmapped path: ros_ws/src/pkg_unowned/src/x.cpp" in scope.reason
    assert scope.implementations == scope.tests == scope.why == ()


@pytest.mark.parametrize(
    "path",
    [
        "scripts/check.sh",
        "ros_ws/src/pkg_pp/package.xml",
        "docker/Dockerfile",
        "tests/golden/baseline.json",
        "config/reference_stack.yaml",
        "ros_ws/src/racing_bringup/launch/sim_pure_pursuit.launch.py",
        "ros_ws/src/racing_bringup/racing_bringup/support_node.py",
        "ros_ws/src/racing_bringup/racing_bringup/scenario_parameters.py",
        "ros_ws/src/racing_bringup/setup.py",
        "config/vehicles/f1tenth_default.yaml",
    ],
)
def test_check_1190_triggers_and_composition_files_are_whole_tree(
    path: str,
) -> None:
    """CHECK-1190: whole-tree triggers, each D4 composition file and an
    unmapped config/ path run everything."""
    assert _select([path]).whole_tree is True


def test_check_1190_ready_whole_tree_files_are_the_d4_files() -> None:
    """CHECK-1190: the D4 list is exactly the five files the plan names."""
    assert set(READY_WHOLE_TREE_FILES) == {
        "config/reference_stack.yaml",
        "ros_ws/src/racing_bringup/launch/sim_pure_pursuit.launch.py",
        "ros_ws/src/racing_bringup/racing_bringup/support_node.py",
        "ros_ws/src/racing_bringup/racing_bringup/scenario_parameters.py",
        "ros_ws/src/racing_bringup/setup.py",
    }


def test_check_1200_a_mapped_config_path_selects_its_implementation() -> None:
    """CHECK-1200: unlike scope_for, a config/ path the map places is not
    whole tree for tiers 3-4."""
    scope = _select(["config/loc/slam.yaml"])

    assert scope.whole_tree is False
    assert scope.implementations == ("pose_source=slam",)


def test_check_1210_the_reference_lap_is_always_selected() -> None:
    """CHECK-1210: a neutral-only change selects SIM-3020 alone."""
    scope = _select(["docs/agents/testing.md", "scripts/tests/x.py"])

    assert scope.whole_tree is False
    assert scope.implementations == ()
    assert scope.tests == ("SIM-3020",)


def test_check_1220_unclaimed_and_missing_tests_are_whole_tree() -> None:
    """CHECK-1220: a discovered test nobody claims, or a claim with no
    test, widens even a neutral-only change."""
    unclaimed = _select(
        ["docs/x.md"], system={**SELECT_SYSTEM, "SIM-3999": "t.py"}
    )
    assert unclaimed.whole_tree is True
    assert "unclaimed test: SIM-3999" in unclaimed.reason

    missing = {k: v for k, v in SELECT_SYSTEM.items() if k != "SIM-3080"}
    gone = _select(["config/loc/amcl.yaml"], system=missing)
    assert gone.whole_tree is True
    assert "claimed test not found: SIM-3080" in gone.reason


def test_check_1220_empty_set_and_missing_reference_lap() -> None:
    """CHECK-1220: an empty changed set and a map without a reference
    lap are both whole tree."""
    assert _select([]).whole_tree is True
    assert _select(
        ["docs/x.md"], data={**SELECT_MAP, "reference_lap": ""}
    ).whole_tree


def test_check_1230_a_changed_system_test_selects_its_id() -> None:
    """CHECK-1230: editing tests/system/test_sim_3030_*.py selects SIM-3030
    without selecting its claimant's other tests."""
    scope = _select(["tests/system/test_sim_3030_x.py"])

    assert scope.whole_tree is False
    assert scope.implementations == ()
    assert scope.tests == ("SIM-3020", "SIM-3030")


def test_check_1240_owners_of_names_the_claimants() -> None:
    """CHECK-1240: a test's owners are the implementations that claim it;
    the reference lap is owned by every default; an unknown ID by none."""
    stack = stack_from_dict(SELECT_MAP, SELECT_SYSTEM, SELECT_TAGS)

    assert owners_of("SIM-3080", stack) == ["pose_source=amcl"]
    assert owners_of("ACC-4030", stack) == [
        "pose_source=amcl",
        "pose_source=slam",
    ]
    assert owners_of("SIM-3020", stack) == [
        "controller=pp",
        "pose_source=truth",
    ]
    assert owners_of("SIM-9999", stack) == []


def _eval_shell(rendered: str, expression: str) -> str:
    """Run `rendered` through bash's eval and print `expression`."""
    return subprocess.run(
        ["bash", "-c", f"set -u; eval {shlex.quote(rendered)}; {expression}"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def test_check_1250_render_ready_shell() -> None:
    """CHECK-1250: scoped output lists test files, Robot tags and the
    `ready scope:` summary; whole tree renders empty arrays; both eval."""
    stack = stack_from_dict(SELECT_MAP, SELECT_SYSTEM, SELECT_TAGS)
    scope = select_ready(
        ["config/loc/amcl.yaml"], stack, {"pkg_sim": {"pkg_pp"}}
    )

    rendered = render_ready_shell(scope, stack)

    assert _eval_shell(rendered, 'echo "$ready_whole_tree"').strip() == "0"
    files = _eval_shell(rendered, 'printf "%s\\n" "${ready_system_tests[@]}"')
    assert files.split() == [
        "tests/system/test_sim_3020_x.py",
        "tests/system/test_sim_3080_x.py",
    ]
    tags = _eval_shell(rendered, 'printf "%s\\n" "${ready_robot_tags[@]}"')
    assert tags.split() == ["impl:pose_source=amcl"]
    line = _eval_shell(rendered, 'echo "$ready_line"')
    assert "pose_source=amcl" in line
    assert "SIM-3080" in line

    whole = render_ready_shell(select_ready([], stack, {}), None)
    assert _eval_shell(whole, 'echo "$ready_whole_tree"').strip() == "1"
    assert _eval_shell(whole, 'echo "${#ready_system_tests[@]}"').strip() == "0"
    assert "whole tree (empty changed set)" in _eval_shell(
        whole, 'echo "$ready_line"'
    )


def _git_returning(paths: str):
    base = "abc"
    return _fake_run_git(
        {
            _PROBE: _ABSENT,
            ("merge-base", "main", "HEAD"): base,
            ("diff", "--name-only", "--no-renames", base): paths,
            ("diff", "--name-only", "--no-renames", "--cached", base): "",
            ("ls-files", "--others", "--exclude-standard"): "",
        }
    )


def test_check_1260_resolve_ready_falls_back_to_whole_tree() -> None:
    """CHECK-1260: CHECK_SCOPE=all, a git failure, an empty set and a map
    load failure are all whole tree; a healthy run is scoped."""
    stack = stack_from_dict(SELECT_MAP, SELECT_SYSTEM, SELECT_TAGS)

    def load():
        return stack, SELECT_DEPS

    def broken():
        raise ModuleNotFoundError("No module named 'robot'")

    def failing_git(args: list[str]) -> str:
        raise subprocess.CalledProcessError(1, args)

    def unexpected_git(args: list[str]) -> str:
        raise AssertionError("git should not be called")

    scope, _ = resolve_ready({"CHECK_SCOPE": "all"}, unexpected_git, load)
    assert scope.whole_tree and "CHECK_SCOPE=all" in scope.reason

    scope, _ = resolve_ready({}, failing_git, load)
    assert scope.whole_tree and "git failed" in scope.reason

    scope, _ = resolve_ready({}, _git_returning(""), load)
    assert scope.whole_tree and "empty changed set" in scope.reason

    scope, _ = resolve_ready({}, _git_returning("docs/x.md\n"), broken)
    assert scope.whole_tree
    assert "reference_stack.yaml unreadable" in scope.reason

    scope, got = resolve_ready({}, _git_returning("docs/x.md\n"), load)
    assert not scope.whole_tree and got is stack


def _cli(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, "scripts/changed_scope.py", *args],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )


def test_check_1270_cli_owners_and_ready() -> None:
    """CHECK-1270: an unknown test exits 1 saying nobody claims it, and
    `--ready` prints something bash can eval."""
    unknown = _cli("--owners", "SIM-9999")
    assert unknown.returncode == 1
    assert "claimed by no implementation" in unknown.stderr

    ready = _cli("--ready")
    assert ready.returncode == 0
    flag = _eval_shell(ready.stdout, 'echo "$ready_whole_tree"').strip()
    assert flag in {"0", "1"}


def test_check_1300_raceline_paths_are_neutral_on_the_real_map() -> None:
    """CHECK-1300: raceline/ is offline (its output reaches a lap only through
    committed files), so on the real map its paths select only the reference
    lap, not the whole tree."""
    stack = load_stack(REPO_ROOT)
    deps = reverse_dependents(read_manifests(REPO_ROOT))

    scope = select_ready(
        [
            "raceline/raceline/tum.py",
            "raceline/pyproject.toml",
            "raceline/uv.lock",
            "raceline/README.md",
        ],
        stack,
        deps,
    )

    assert scope.whole_tree is False
    assert scope.tests == ("SIM-3020",)


def test_check_1530_raceline_report_is_neutral_frozen_pose_is_not() -> None:
    """CHECK-1530: raceline_report.py is read only by tier 5 and its tier-1
    test, so on the real map it selects only the reference lap. The frozen-pose
    rule a lap imports lives in frozen_pose.py, which stays whole tree."""
    stack = load_stack(REPO_ROOT)
    deps = reverse_dependents(read_manifests(REPO_ROOT))

    report = select_ready(["scripts/raceline_report.py"], stack, deps)
    assert report.whole_tree is False
    assert report.tests == ("SIM-3020",)

    frozen = select_ready(["scripts/frozen_pose.py"], stack, deps)
    assert frozen.whole_tree is True


def test_check_1310_gym_limits_vehicle_and_ggv_paths_are_neutral() -> None:
    """CHECK-1310: no gate test launches the gym-limits vehicle and no node
    reads a GGV table, so on the real map their paths select only the
    reference lap. The default vehicle's yaml stays whole tree (CHECK-1190)."""
    stack = load_stack(REPO_ROOT)
    deps = reverse_dependents(read_manifests(REPO_ROOT))

    scope = select_ready(
        [
            "config/vehicles/f1tenth_gym_limits.yaml",
            "config/vehicles/f1tenth_gym_limits/ggv.csv",
            "config/vehicles/f1tenth_default/ggv.csv",
        ],
        stack,
        deps,
    )

    assert scope.whole_tree is False
    assert scope.tests == ("SIM-3020",)


def test_check_1420_the_planner_alternative_owns_the_committed_racelines() -> (
    None
):
    """CHECK-1420: the planner's default is `track_centerline`, the committed
    racelines belong to the `optimized_raceline` alternative that SIM-3110
    verifies, and a change to one selects that test rather than the whole
    tree."""
    stack = load_stack(REPO_ROOT)
    deps = reverse_dependents(read_manifests(REPO_ROOT))

    import yaml

    planner = yaml.safe_load(
        (REPO_ROOT / "config" / "reference_stack.yaml").read_text(
            encoding="utf-8"
        )
    )["roles"]["planner"]
    assert planner["implementation"] == "track_centerline"
    alternatives = {a["implementation"]: a for a in planner["alternatives"]}
    assert alternatives["optimized_raceline"]["verified_by"] == ["SIM-3110"]

    scope = select_ready(
        ["config/racelines/spielberg/f1tenth-default-v1/tum_mincurv_iqp.csv"],
        stack,
        deps,
    )

    assert scope.whole_tree is False
    assert set(scope.tests) == {"SIM-3020", "SIM-3110"}


def test_check_1170_real_map_instrument_couplings() -> None:
    """CHECK-1170: on the real map the sim feeds every estimator (R1) and
    racing_evaluation is their measuring instrument (R2), so each path
    selects the estimators' tests though neither claims them."""
    stack = load_stack(REPO_ROOT)
    deps = reverse_dependents(read_manifests(REPO_ROOT))

    sim = select_ready(
        ["ros_ws/src/racing_sim_gym_jax/racing_sim_gym_jax/laser.py"],
        stack,
        deps,
    )
    assert {"SIM-3070", "SIM-3080"} <= set(sim.tests)

    evaluation = select_ready(
        ["ros_ws/src/racing_evaluation/racing_evaluation/node.py"],
        stack,
        deps,
    )
    assert {"SIM-3070", "SIM-3080", "ACC-4030"} <= set(evaluation.tests)
    assert evaluation.whole_tree is False


def test_check_1270_real_map_owners_of_amcl_localizing() -> None:
    """CHECK-1270: `--owners SIM-3080` prints exactly the amcl claimant."""
    result = _cli("--owners", "SIM-3080")

    assert result.returncode == 0
    assert result.stdout == "pose_source=amcl\n"


# --- worktrees seen through a different mount (CHECK-1350..1390) ----------

_PKG_FILE = "ros_ws/src/racing_common/racing_common/x.py"


def _git(cwd: Path, env: dict[str, str], *args: str) -> str:
    return subprocess.run(
        ["git", "-c", "commit.gpgsign=false", *args],
        cwd=cwd,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def _git_layout(
    tmp_path: Path,
    *,
    gitfile: str = "absolute",
    move: bool = True,
    foreign_owner: bool = False,
    trust_common: bool = True,
) -> tuple[Path, Path, dict[str, str]]:
    """A real repo with a worktree made under `host/repo`; `move` renames the
    repo to `ws` (the container's mount path). Returns (repo, worktree, env)
    where `env` is what changed_scope would inherit in the container."""
    base = {
        k: v for k, v in os.environ.items() if k in {"PATH", "LANG", "LC_ALL"}
    }
    base |= {
        "HOME": str(tmp_path),
        "GIT_CONFIG_GLOBAL": "/dev/null",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_AUTHOR_NAME": "t",
        "GIT_AUTHOR_EMAIL": "t@example.com",
        "GIT_COMMITTER_NAME": "t",
        "GIT_COMMITTER_EMAIL": "t@example.com",
    }
    repo = tmp_path / "host" / "repo"
    repo.mkdir(parents=True)
    _git(repo, base, "init", "-q", "-b", "main")
    (repo / "README.md").write_text("x\n")
    (repo / ".gitignore").write_text(".scratch/\n")
    _git(repo, base, "add", ".")
    _git(repo, base, "commit", "-q", "-m", "init")
    _git(repo, base, "worktree", "add", "-q", ".scratch/wt", "-b", "feat")
    wt = repo / ".scratch" / "wt"
    (wt / _PKG_FILE).parent.mkdir(parents=True)
    (wt / _PKG_FILE).write_text("x = 1\n")
    _git(wt, base, "add", ".")
    _git(wt, base, "commit", "-q", "-m", "feature")
    if gitfile == "relative":
        (wt / ".git").write_text("gitdir: ../../.git/worktrees/wt\n")
    if move:
        ws = tmp_path / "ws"
        repo.rename(ws)
    else:
        ws = repo
    env = dict(base)
    if foreign_owner:
        env["GIT_TEST_ASSUME_DIFFERENT_OWNER"] = "1"
    if trust_common:
        env |= {
            "GIT_CONFIG_COUNT": "1",
            "GIT_CONFIG_KEY_0": "safe.directory",
            "GIT_CONFIG_VALUE_0": str(ws),
        }
    return ws, ws / ".scratch" / "wt", env


def _plain_merge_base(wt: Path, env: dict[str, str]) -> int:
    return subprocess.run(
        ["git", "-C", str(wt), "merge-base", "main", "HEAD"],
        env=env,
        capture_output=True,
        text=True,
    ).returncode


@pytest.mark.parametrize(
    ("gitfile", "foreign_owner"),
    [("absolute", False), ("relative", True), ("absolute", True)],
)
def test_check_1350_a_host_worktree_seen_through_the_mount_is_scoped(
    tmp_path: Path, gitfile: str, foreign_owner: bool
) -> None:
    """CHECK-1350: a worktree made on the host, seen under another path (and
    maybe another owner), is scoped to its changes, not whole tree."""
    from scripts.changed_scope import make_run_git

    ws, wt, env = _git_layout(
        tmp_path, gitfile=gitfile, foreign_owner=foreign_owner
    )
    assert _plain_merge_base(wt, env) == 128  # the control: plain git fails

    run_git, loc = make_run_git(wt, env)
    scope, reason = resolve(env, run_git, exists=lambda p: (wt / p).exists())

    assert scope.whole_tree is False, reason
    assert scope.packages == frozenset({"racing_common"})
    assert loc is not None
    assert loc.git_dir.resolve() == (ws / ".git" / "worktrees" / "wt").resolve()


def test_check_1360_a_dev_side_worktree_and_the_main_checkout(
    tmp_path: Path,
) -> None:
    """CHECK-1360: a gitfile naming an existing dir is located to that dir;
    the main checkout and an inherited GIT_DIR get no location."""
    from scripts.changed_scope import locate_git_dir, make_run_git

    ws, wt, env = _git_layout(tmp_path, move=False)
    expected = (ws / ".git" / "worktrees" / "wt").resolve()

    loc = locate_git_dir(wt, env)
    assert loc is not None and loc.git_dir.resolve() == expected
    assert loc.work_tree == wt

    run_git, got = make_run_git(wt, env)
    assert got is None  # plain git works here, so nothing is located
    scope, _ = resolve(env, run_git, exists=lambda p: (wt / p).exists())
    assert scope.packages == frozenset({"racing_common"})

    assert locate_git_dir(ws, env) is None
    assert locate_git_dir(wt, env | {"GIT_DIR": str(expected)}) is None


def _foreign_repo(tmp_path: Path, env: dict[str, str]) -> tuple[Path, Path]:
    """A repo whose config runs a script (core.fsmonitor); returns (repo,
    marker). The marker exists only if that script ran."""
    foreign = tmp_path / "foreign"
    foreign.mkdir()
    _git(foreign, env, "init", "-q", "-b", "main")
    marker = tmp_path / "marker"
    hook = tmp_path / "hook.sh"
    hook.write_text(f"#!/bin/sh\ntouch {marker}\n")
    hook.chmod(0o755)
    _git(foreign, env, "config", "core.fsmonitor", str(hook))
    return foreign, marker


def _assert_no_location_and_whole_tree(
    wt: Path, env: dict[str, str], **kwargs: object
) -> None:
    from scripts.changed_scope import locate_git_dir, make_run_git

    assert locate_git_dir(wt, env, **kwargs) is None  # type: ignore[arg-type]
    run_git, loc = make_run_git(wt, env)
    assert loc is None
    scope, reason = resolve(env, run_git, exists=lambda p: True)
    assert scope.whole_tree is True
    assert "git failed" in reason


def test_check_1370_a_fake_nested_common_repo_is_not_trusted(
    tmp_path: Path,
) -> None:
    """CHECK-1370: a fake `A/.git` inside a trusted repo, whose commondir
    names a foreign repo, gives no location and runs nothing foreign."""
    ws, _wt, env = _git_layout(tmp_path)
    foreign, marker = _foreign_repo(tmp_path, env)
    meta = ws / "sub" / ".git" / "worktrees" / "wt2"
    meta.mkdir(parents=True)
    fake = ws / "sub" / ".scratch" / "wt2"
    fake.mkdir(parents=True)
    (meta / "gitdir").write_text(f"{fake}/.git\n")
    (meta / "commondir").write_text(f"{foreign}/.git\n")
    (fake / ".git").write_text("gitdir: /nowhere/.git/worktrees/wt2\n")

    _assert_no_location_and_whole_tree(fake, env)

    assert not marker.exists()


def test_check_1370_a_commondir_outside_the_common_repo(
    tmp_path: Path,
) -> None:
    """CHECK-1370: a commondir that does not resolve to A/.git is refused."""
    ws, wt, env = _git_layout(tmp_path)
    foreign, marker = _foreign_repo(tmp_path, env)
    meta = ws / ".git" / "worktrees" / "wt"
    (meta / "commondir").write_text(f"{foreign}/.git\n")

    _assert_no_location_and_whole_tree(wt, env)

    assert not marker.exists()


@pytest.mark.parametrize(
    "which", ["metadata", "commondir", "gitdir", "gitfile"]
)
def test_check_1370_a_uid_mismatch_is_refused(
    tmp_path: Path, which: str
) -> None:
    """CHECK-1370: any of the four checked paths not owned by the owner of
    A/.git is refused; without the injection the same layout is located."""
    from scripts.changed_scope import locate_git_dir

    ws, wt, env = _git_layout(tmp_path)
    meta = ws / ".git" / "worktrees" / "wt"
    odd = {
        "metadata": meta,
        "commondir": meta / "commondir",
        "gitdir": meta / "gitdir",
        "gitfile": wt / ".git",
    }[which]

    def owner(p: Path) -> int:
        uid = p.stat().st_uid
        return uid + 1 if p == odd else uid

    assert locate_git_dir(wt, env) is not None  # the positive control
    assert locate_git_dir(wt, env, owner=owner) is None


def test_check_1370_a_fake_common_dir_without_head_is_not_trusted(
    tmp_path: Path,
) -> None:
    """CHECK-1370: `ws/sub/.git` with no HEAD is not a repo, so git's probe
    answers with the parent's; the answer must equal A/.git, or a fake A whose
    commondir is `../..` would run its own core.fsmonitor."""
    ws, _wt, env = _git_layout(tmp_path)
    _foreign, marker = _foreign_repo(tmp_path, env)
    fake_git = ws / "sub" / ".git"
    (fake_git / "objects").mkdir(parents=True)
    (fake_git / "refs").mkdir()
    (fake_git / "config").write_text(
        f"[core]\n\tfsmonitor = {tmp_path / 'hook.sh'}\n"
    )
    meta = fake_git / "worktrees" / "wt2"
    meta.mkdir(parents=True)
    fake = ws / "sub" / ".scratch" / "wt2"
    fake.mkdir(parents=True)
    (meta / "HEAD").write_text("ref: refs/heads/main\n")
    (meta / "gitdir").write_text(f"{fake}/.git\n")
    (meta / "commondir").write_text("../..\n")
    (fake / ".git").write_text("gitdir: /nowhere/.git/worktrees/wt2\n")

    _assert_no_location_and_whole_tree(fake, env)

    assert not marker.exists()


def test_check_1370_a_commondir_symlink_loop_is_refused(
    tmp_path: Path,
) -> None:
    """CHECK-1370: a commondir naming a symlink loop gives no location, not
    a RuntimeError traceback."""
    ws, wt, env = _git_layout(tmp_path)
    meta = ws / ".git" / "worktrees" / "wt"
    (meta / "loop").symlink_to(meta / "loop")
    (meta / "commondir").write_text("loop\n")

    _assert_no_location_and_whole_tree(wt, env)


@pytest.mark.parametrize("target_name", ["..", ".", ""])
def test_check_1370_a_gitfile_ending_in_a_dot_name_is_refused(
    tmp_path: Path, target_name: str
) -> None:
    """CHECK-1370: a gitdir ending in `..`, `.` or nothing would make the
    'metadata dir' A/.git or worktrees/ itself; no location."""
    import os

    ws, _wt, env = _git_layout(tmp_path)
    fake = ws / ".scratch" / "wt2"
    fake.mkdir()
    (fake / ".git").write_text(
        f"gitdir: /nowhere/.git/worktrees/{target_name}\n"
    )
    cand = ws / ".git" / "worktrees" / target_name
    (cand / "gitdir").write_text(f"{fake}/.git\n")
    rel = os.path.relpath(ws / ".git", cand)
    (cand / "commondir").write_text(rel + "\n")

    from scripts.changed_scope import locate_git_dir

    assert locate_git_dir(fake, env) is None


@pytest.mark.parametrize(
    "name",
    [
        "GIT_COMMON_DIR",
        "GIT_INDEX_FILE",
        "GIT_OBJECT_DIRECTORY",
        "GIT_ALTERNATE_OBJECT_DIRECTORIES",
    ],
)
def test_check_1370_an_inherited_git_location_variable_disables_it(
    tmp_path: Path, name: str
) -> None:
    """CHECK-1370: like GIT_DIR, these mean the caller already chose where
    git looks, and would override what the gate validated."""
    from scripts.changed_scope import locate_git_dir

    ws, wt, env = _git_layout(tmp_path)
    assert locate_git_dir(wt, env) is not None  # the positive control
    value = str(ws / ".git") if name == "GIT_COMMON_DIR" else str(tmp_path)
    assert locate_git_dir(wt, env | {name: value}) is None


def test_check_1370_the_locator_is_not_consulted_on_a_healthy_repo(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """CHECK-1370: when plain git opens the checkout, the ownership bypass is
    never run and the CLI prints no `git dir` line."""
    import scripts.changed_scope as module

    _ws, wt, env = _git_layout(tmp_path, move=False)

    def boom(*args: object, **kwargs: object) -> None:
        raise AssertionError("locate_git_dir consulted on a healthy repo")

    monkeypatch.setattr(module, "locate_git_dir", boom)
    run_git, loc = module.make_run_git(wt, env)
    assert loc is None
    assert run_git(["rev-parse", "--is-inside-work-tree"]).strip() == "true"

    result = subprocess.run(
        [sys.executable, str(REPO_ROOT / "scripts" / "changed_scope.py")],
        cwd=wt,
        env=env,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0 and "git dir" not in result.stderr


def test_check_1370_owners_does_not_probe_git(tmp_path: Path) -> None:
    """CHECK-1370: `--owners` neither spawns the git probe nor prints the
    located-git-dir line, even where the locator would fire."""
    _ws, wt, env = _git_layout(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "scripts" / "changed_scope.py"),
            "--owners",
            "SIM-3080",
        ],
        cwd=wt,
        env=env,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "git dir" not in result.stderr


def test_check_1370_unmatched_metadata_is_refused(tmp_path: Path) -> None:
    """CHECK-1370: no worktrees/<name> among the ancestors, a back-pointer
    naming another position, or a malformed gitfile: no location."""
    ws, wt, env = _git_layout(tmp_path)
    meta = ws / ".git" / "worktrees" / "wt"

    meta.rename(meta.with_name("renamed"))
    _assert_no_location_and_whole_tree(wt, env)
    meta.with_name("renamed").rename(meta)

    (meta / "gitdir").write_text("/elsewhere/.scratch/other/.git\n")
    _assert_no_location_and_whole_tree(wt, env)

    (wt / ".git").write_text("garbage\n")
    _assert_no_location_and_whole_tree(wt, env)


def test_check_1370_an_untrusted_common_repo_is_refused(
    tmp_path: Path,
) -> None:
    """CHECK-1370: a common repo git itself would not open (foreign owner,
    no safe.directory) gives no location; --ready is whole tree."""
    from scripts.changed_scope import locate_git_dir, make_run_git

    _ws, wt, env = _git_layout(tmp_path, foreign_owner=True, trust_common=False)
    assert locate_git_dir(wt, env) is None

    run_git, _loc = make_run_git(wt, env)

    def load():
        raise AssertionError("the map is not read when git fails")

    scope, _ = resolve_ready(env, run_git, load)
    assert scope.whole_tree is True
    assert "git failed" in scope.reason


# --- the base is origin/main (CHECK-1380) --------------------------------

_ORIGIN_MB = ("merge-base", "refs/remotes/origin/main", "HEAD")
_MAIN_MB = ("merge-base", "main", "HEAD")


def test_check_1380_origin_main_is_the_base_when_it_exists() -> None:
    """CHECK-1380: origin/main wins; local main only when the lookup exits 1,
    and then the label says so."""
    from scripts.changed_scope import merge_base

    run_git = _fake_run_git({_PROBE: "", _ORIGIN_MB: "abc\n"})
    assert merge_base(run_git) == ("origin/main", "abc")

    run_git = _fake_run_git({_PROBE: _ABSENT, _MAIN_MB: "def\n"})
    assert merge_base(run_git) == ("main (no origin/main)", "def")


def test_check_1380_a_failed_base_is_whole_tree_without_fallback() -> None:
    """CHECK-1380: origin/main present but its merge-base failing, or the
    lookup itself failing (not 'absent'), is whole tree and never retries
    against local main."""

    def load():
        raise AssertionError("the map is not read when git fails")

    for calls in (
        {_PROBE: "", _ORIGIN_MB: _ABSENT},
        {_PROBE: _FAILED},
    ):
        run_git = _fake_run_git(calls)
        scope, reason = resolve({}, run_git, exists=lambda p: True)
        assert scope.whole_tree and "git failed" in reason
        assert _MAIN_MB not in run_git.seen  # type: ignore[attr-defined]

        run_git = _fake_run_git(calls)
        ready, _ = resolve_ready({}, run_git, load)
        assert ready.whole_tree and "git failed" in ready.reason
        assert _MAIN_MB not in run_git.seen  # type: ignore[attr-defined]


def test_check_1380_a_stale_local_main_does_not_widen_the_changed_set(
    tmp_path: Path,
) -> None:
    """CHECK-1380: with local main two commits behind origin/main, those two
    commits' files are not in the branch's changed set."""
    from scripts.changed_scope import make_run_git

    ws, wt, env = _git_layout(tmp_path, move=False)
    for name in ("a", "b"):
        (ws / "docs").mkdir(exist_ok=True)
        (ws / "docs" / f"{name}.md").write_text(name)
        _git(ws, env, "add", ".")
        _git(ws, env, "commit", "-q", "-m", name)
    tip = _git(ws, env, "rev-parse", "HEAD").strip()
    _git(ws, env, "update-ref", "refs/remotes/origin/main", tip)
    _git(ws, env, "reset", "-q", "--hard", "HEAD~2")
    _git(wt, env, "checkout", "-q", "-b", "feat2", "refs/remotes/origin/main")
    (wt / "docs" / "c.md").write_text("c")
    _git(wt, env, "add", ".")
    _git(wt, env, "commit", "-q", "-m", "c")

    run_git, _loc = make_run_git(wt, env)

    assert changed_paths(run_git) == ["docs/c.md"]


# --- the reason line names the base (CHECK-1390) --------------------------


def test_check_1390_the_cli_in_a_host_worktree_names_base_and_git_dir(
    tmp_path: Path,
) -> None:
    """CHECK-1390: the CLI, run in a moved host-style worktree, is scoped,
    its reason names the base, and stderr names the located git dir."""
    ws, wt, env = _git_layout(tmp_path)
    _git(ws, env, "update-ref", "refs/remotes/origin/main", "main")

    result = subprocess.run(
        [sys.executable, str(REPO_ROOT / "scripts" / "changed_scope.py")],
        cwd=wt,
        env=env,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    out = result.stdout
    assert _eval_shell(out, 'echo "$scope_whole_tree"').strip() == "0"
    assert "base origin/main" in _eval_shell(out, 'echo "$scope_reason"')
    assert "git dir" in result.stderr
    assert "worktrees/wt" in result.stderr


def test_check_1390_ready_line_appends_the_base() -> None:
    """CHECK-1390: render_ready_shell appends `; base ...` on scoped and
    whole-by-rule lines, and nothing when the base is empty."""
    stack = stack_from_dict(SELECT_MAP, SELECT_SYSTEM, SELECT_TAGS)
    scoped = select_ready(
        ["config/loc/amcl.yaml"], stack, {"pkg_sim": {"pkg_pp"}}
    )
    whole = select_ready([], stack, {})

    def line(scope: ReadyScope) -> str:
        rendered = render_ready_shell(scope, stack)
        return _eval_shell(rendered, 'echo "$ready_line"').strip()

    base = "origin/main abc"
    assert line(replace(scoped, base=base)).endswith(f"; base {base}")
    assert line(replace(whole, base=base)).endswith(f"; base {base})")
    assert line(scoped) == line(replace(scoped, base=""))
    assert "base" not in line(scoped).split("because")[-1]
    assert "base" not in line(whole)
