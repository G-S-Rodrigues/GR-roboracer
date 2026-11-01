"""The lidar's TF frame is named once and every copy of it agrees.

CHECK-2010 walks the whole repo and fails on any text that names a frame
with the old word. CHECK-2020 holds the three Python mirrors and the URDF
link of contract.hpp's `kLaserFrame` to it. Device identifiers (`laser_x`,
`LaserMount`, the `racing_sim_gym_jax.laser` module) keep the word and are
pinned as not flagged. Nothing here imports ROS.
"""

from __future__ import annotations

import ast
import os
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
THIS_FILE = Path(__file__).resolve()
HEADER = (
    REPO_ROOT
    / "ros_ws/src/racing_sim_adapter/include/racing_sim_adapter/contract.hpp"
)
DESCRIPTION = REPO_ROOT / "ros_ws/src/racing_vehicle_description"
URDF = DESCRIPTION / "urdf/f1tenth.urdf"
# The lidar mesh of f1tenth.urdf and of autodrive_roboracer.urdf.
LIDAR_MESHES = ("hokuyo.stl", "autodrive_roboracer/lidar.stl")
# Module-level `NAME = "..."` mirrors of the contract's scan frame.
MIRRORS = (
    ("ros_ws/src/racing_sim_adapter/test/conformance.py", "SCAN_FRAME"),
    ("ros_ws/src/racing_sim_gym_jax/racing_sim_gym_jax/laser.py", "SCAN_FRAME"),
    (
        "ros_ws/src/racing_sim_autodrive/racing_sim_autodrive/bridge.py",
        "SCAN_FRAME_ID",
    ),
)
EXPECTED = "lidar"

# The old word, split so this file cannot flag itself.
OLD = "la" + "ser"
FORBIDDEN = [
    re.compile(p.replace("OLD", OLD))
    for p in (
        r"\"OLD\"",
        r"`OLD`",
        r"'OLD'",
        r"-> ?OLD\b",
        r"\bOLD ?->",
        r"_to_OLD\b",
        r"(?i)frame[ _-]?(id)?\s*[:=]\s*[\"']?OLD\b",
        r"--(child-)?frame-id\s+OLD\b",
        r"tf2_echo\s+\S+\s+OLD\b",
        r"(?i)\bframe\s+OLD\b",
    )
]

SKIP_DIRS = {
    "third_party",
    "build",
    "install",
    "log",
    ".scratch",
    ".git",
    "__pycache__",
    ".venv",
    "tum_env",
}
SKIP_SUFFIXES = {".rosbag", ".db3", ".pyc"}


# Empty since part 8's addendum regenerated the raceline records (2026-10-31);
# an entry is a reviewed exception, keyed by (file, exact line).
ALLOWED = frozenset()


def _flags(line: str) -> bool:
    return any(p.search(line) for p in FORBIDDEN)


def _allowed(rel: str, line: str) -> bool:
    return (rel, line) in ALLOWED


def _tracked(root: Path) -> list[Path] | None:
    """`git ls-files` under `root`, or None when git cannot answer (a host
    worktree seen from dev, gotcha 23)."""
    try:
        out = subprocess.run(
            ["git", "-C", str(root), "ls-files", "-z"],
            check=True,
            capture_output=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    return [root / n.decode() for n in out.split(b"\0") if n]


def _walked(root: Path):
    """Fallback: every file on disk, minus `.claude/` except its skills."""
    for dirpath, dirnames, filenames in os.walk(root):
        rel = Path(dirpath).relative_to(root).parts
        dirnames[:] = [
            d
            for d in dirnames
            if d not in SKIP_DIRS
            and not (d.startswith(".") and d.endswith("_cache"))
        ]
        if rel == (".claude",):
            dirnames[:] = [d for d in dirnames if d == "skills"]
            continue
        for name in filenames:
            yield Path(dirpath) / name


def _text_files(root: Path = REPO_ROOT, use_git: bool = True):
    """Text files of the tracked tree; with no git, the walk fallback."""
    paths = _tracked(root) if use_git else None
    for path in _walked(root) if paths is None else paths:
        if any(part in SKIP_DIRS for part in path.relative_to(root).parts):
            continue
        if path.suffix in SKIP_SUFFIXES or path.resolve() == THIS_FILE:
            continue
        if path.is_symlink() or not path.is_file():
            continue
        data = path.read_bytes()
        if b"\0" in data:
            continue
        try:
            yield path, data.decode("utf-8")
        except UnicodeDecodeError:
            continue


FLAGGED = [
    '"OLD"',
    "`OLD`",
    "'OLD'",
    "rear_axle -> OLD",
    "OLD -> base_link",
    "frame_id: OLD",
    "rear_axle_to_OLD",
    "Fixed Frame: OLD",
    "cloud_frame: OLD",
    "--child-frame-id OLD",
    "tf2_echo base_link OLD",
    "stamped in frame OLD",
]
NOT_FLAGGED = [
    "OLD_x: 0.275",
    "LaserMount(",
    "racing_sim_gym_jax.OLD.cast_scan",
    "from .OLD import (",
    "sensor_msgs/LaserScan",
    "lidar_topic_type: LaserScan",
    "OLD_grey",
    "OLD_max_range",
    'OLD_model_type: "likelihood_field"',
    "the OLD mount",
    "hokuyo.stl",
    "lidar_sensor_labels",
    'kLaserFrame{"lidar"}',
    "test_urdf_has_canonical_vehicle_and_OLD_frames",
    "add_launch_test(test/test_adapter_OLD_origin.py",
    (
        "The lidar's TF frame is lidar, renamed from OLD on 2026-10-28 to "
        "match the AutoDRIVE devkit; device identifiers (OLD_x, LaserMount, "
        "the racing_sim_gym_jax.OLD module) keep the old word, and a bag or "
        "RViz config recorded before the rename still names OLD."
    ),
]


@pytest.mark.parametrize("line", FLAGGED)
def test_check_2010_each_forbidden_form_is_flagged(line: str) -> None:
    assert _flags(line.replace("OLD", OLD)), line


@pytest.mark.parametrize("line", NOT_FLAGGED)
def test_check_2010_device_names_are_not_flagged(line: str) -> None:
    assert not _flags(line.replace("OLD", OLD)), line


def test_check_2010_the_allowlist_is_exact() -> None:
    """An allowlisted line is exempt only in its own file, byte for byte."""
    for rel, line in ALLOWED:
        assert _flags(line), "allowlisted text no longer needs the exemption"
        assert _allowed(rel, line)
        assert not _allowed("docs/elsewhere.md", line)
        assert not _allowed(rel, line + " ")
        assert (REPO_ROOT / rel).read_text().count(line + "\n") == 1


def test_check_2010_no_frame_is_named_laser() -> None:
    """CHECK-2010: no tracked text names the lidar frame with the old word."""
    hits = []
    for path, text in _text_files():
        for number, line in enumerate(text.splitlines(), 1):
            rel = path.relative_to(REPO_ROOT).as_posix()
            if _flags(line) and not _allowed(rel, line):
                hits.append(
                    f"{path.relative_to(REPO_ROOT)}:{number}: {line.strip()}"
                )
    assert not hits, f"{len(hits)} lines:\n" + "\n".join(hits)


def _assigned(path: Path, name: str) -> str:
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if (
            isinstance(node, ast.Assign)
            and any(
                isinstance(t, ast.Name) and t.id == name for t in node.targets
            )
            and isinstance(node.value, ast.Constant)
        ):
            return node.value.value
    raise AssertionError(f"{path}: no module-level {name} = '...'")


def _mapped_urdfs() -> list[Path]:
    """Every URDF named in urdf/descriptions.yaml (CHECK-2020 loops)."""
    mapping = yaml.safe_load(
        (DESCRIPTION / "urdf/descriptions.yaml").read_text()
    )
    return [
        DESCRIPTION / "urdf" / name for name in sorted(set(mapping.values()))
    ]


def test_check_2020_every_frame_mirror_equals_the_contract() -> None:
    """CHECK-2020: contract.hpp's kLaserFrame, the Python mirrors and the
    URDF link carrying the lidar mesh all name the same frame."""
    match = re.search(
        r"kLaserFrame\s*\{\s*\"([^\"]+)\"\s*\}", HEADER.read_text()
    )
    assert match, "contract.hpp has no kLaserFrame"
    frame = match.group(1)
    assert frame == EXPECTED, f"kLaserFrame {frame!r} != {EXPECTED!r}"
    for rel, name in MIRRORS:
        assert _assigned(REPO_ROOT / rel, name) == frame, rel
    for urdf in _mapped_urdfs():
        root = ET.parse(urdf).getroot()
        sensors = [
            link.attrib["name"]
            for link in root.findall("link")
            if any(
                m.attrib.get("filename", "").endswith(LIDAR_MESHES)
                for m in link.iter("mesh")
            )
        ]
        assert sensors == [frame], (
            f"{urdf.name}: lidar link {sensors} != {frame!r}"
        )
        joint = root.find(f"./joint[@name='rear_axle_to_{frame}']")
        assert joint is not None, f"{urdf.name}: no joint rear_axle_to_{frame}"
        assert joint.find("child").attrib["link"] == frame, urdf.name


def _git_init(root: Path) -> None:
    for args in (["init", "-q"], ["config", "user.email", "t@t"]):
        subprocess.run(["git", "-C", str(root), *args], check=True)
    subprocess.run(
        ["git", "-C", str(root), "config", "user.name", "t"], check=True
    )


def test_check_2010_only_tracked_files_are_walked(tmp_path: Path) -> None:
    """An untracked or git-ignored file naming the old frame is not read."""
    bad = f'frame_id: "{OLD}"\n'
    _git_init(tmp_path)
    (tmp_path / ".gitignore").write_text("ignored.txt\n")
    (tmp_path / "tracked.txt").write_text(bad)
    (tmp_path / "ignored.txt").write_text(bad)
    (tmp_path / "untracked.txt").write_text(bad)
    subprocess.run(
        ["git", "-C", str(tmp_path), "add", ".gitignore", "tracked.txt"],
        check=True,
    )
    seen = {p.name for p, _ in _text_files(tmp_path)}
    assert seen == {".gitignore", "tracked.txt"}


def test_check_2010_without_git_local_claude_files_are_skipped(
    tmp_path: Path,
) -> None:
    """With no git to ask, `.claude/` is skipped except `.claude/skills/`."""
    (tmp_path / ".claude/skills").mkdir(parents=True)
    (tmp_path / ".claude/settings.local.json").write_text("{}")
    (tmp_path / ".claude/skills/a.md").write_text("x")
    (tmp_path / "b.md").write_text("x")
    seen = {p.name for p, _ in _text_files(tmp_path, use_git=False)}
    assert seen == {"a.md", "b.md"}
