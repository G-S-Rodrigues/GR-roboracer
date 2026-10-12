"""Tests for scripts.changed_scope: what `check.sh --fast` scopes to."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from scripts.changed_scope import (
    READY_WHOLE_TREE_FILES,
    ReadyScope,
    Scope,
    StackMap,
    changed_paths,
    load_stack,
    read_manifests,
    render_shell,
    resolve,
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

    if shutil.which("colcon") is None:
        pytest.skip("colcon is not on PATH")
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
    assert len(stack.implementations) == 8
    assert "pose_source=amcl" in keys
    assert "pose_source=slam_toolbox" in keys
    amcl = next(i for i in stack.implementations if i.name == "amcl")
    assert amcl.tag == "impl:pose_source=amcl"
    assert amcl.default is False
    assert set(stack.system_tests) == {
        f"SIM-{n}" for n in range(3010, 3101, 10)
    }
    assert set(stack.acceptance_tags) == {
        "ACC-4010",
        "ACC-4020",
        "ACC-4030",
        "ACC-4040",
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
        "  controller:\n    implementation: pp\n    package: pkg_pp\n",
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
    ],
    ids=["no-id-file", "nested-file", "duplicate-sim", "no-acc-id", "dup-acc"],
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
