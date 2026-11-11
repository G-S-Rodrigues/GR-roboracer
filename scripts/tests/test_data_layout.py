"""The data repo's track layout (D2, ADR 0009 amendment): CHECK-2950..2953.

Reads the `third_party/GR-roboracer-data` submodule. A layout nobody checks
drifts: a map with no PNG or no README looks fine until someone has to say
where it came from.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
TRACKS = REPO_ROOT / "third_party" / "GR-roboracer-data" / "tracks"
MAP_METHODS = ("slam_toolbox", "mola", "far_lio")
README_FIELDS = (
    "Simulator:",
    "Odometry:",
    "Resolution:",
    "Accuracy vs ground truth:",
    "Command:",
    "Date:",
    "GR-roboracer commit:",
)


def _tracks() -> list[Path]:
    if not TRACKS.is_dir():
        pytest.fail(
            f"{TRACKS} is absent; run `git submodule update --init "
            "third_party/GR-roboracer-data` on the host"
        )
    found = sorted(p for p in TRACKS.iterdir() if p.is_dir())
    assert found, f"no track under {TRACKS}"
    return found


def _map_folders(track: Path) -> list[Path]:
    """Every folder that holds one map: slam_toolbox/<variant>, mola, ..."""
    maps = track / "maps"
    folders: list[Path] = []
    if not maps.is_dir():
        return folders
    for method in sorted(p for p in maps.iterdir() if p.is_dir()):
        if method.name == "slam_toolbox":
            folders.extend(sorted(p for p in method.iterdir() if p.is_dir()))
        else:
            folders.append(method)
    return folders


def test_check_2950_the_ground_truth_sits_at_the_track_folder() -> None:
    """CHECK-2950: no `tracks/<M>/maps/<M>` doubling; the simulator's own map
    is `tracks/<M>/<M>.yaml` with its image beside it."""
    for track in _tracks():
        name = track.name
        assert not (track / "maps" / name).exists(), (
            f"{track}/maps/{name} is the pre-D2 doubled folder"
        )
        ground_truth = track / f"{name}.yaml"
        assert ground_truth.is_file(), f"{ground_truth} is missing"
        image = yaml.safe_load(ground_truth.read_text(encoding="utf-8"))[
            "image"
        ]
        assert (track / image).is_file(), f"{ground_truth}: no {image}"


def test_check_2951_every_map_folder_is_a_known_method() -> None:
    """CHECK-2951: the children of `maps/` are mapping methods; slam_toolbox
    has `res_NNNN` variants, NNNN the resolution in millimetres."""
    for track in _tracks():
        maps = track / "maps"
        if not maps.is_dir():
            continue
        for child in maps.iterdir():
            assert child.name in MAP_METHODS, (
                f"{child}: not one of {MAP_METHODS}"
            )
        slam = maps / "slam_toolbox"
        if not slam.is_dir():
            continue
        for variant in sorted(slam.iterdir()):
            match = re.fullmatch(r"res_(\d{4})", variant.name)
            assert match, f"{variant}: expected res_NNNN"
            for grid in variant.glob("*.yaml"):
                resolution = yaml.safe_load(grid.read_text(encoding="utf-8"))[
                    "resolution"
                ]
                assert round(float(resolution) * 1000) == int(match.group(1)), (
                    f"{grid}: resolution {resolution} is not {variant.name}"
                )


def test_check_2952_every_map_folder_has_a_png_it_names() -> None:
    """CHECK-2952: a PNG to look at in every map folder, and every map yaml
    that has an `image:` names a file that exists beside it."""
    for track in _tracks():
        for folder in _map_folders(track):
            assert list(folder.glob("*.png")), f"{folder}: no PNG"
            for grid in folder.glob("*.yaml"):
                document = yaml.safe_load(grid.read_text(encoding="utf-8"))
                if not isinstance(document, dict) or "image" not in document:
                    continue  # MOLA's provenance yaml has no image
                assert (folder / document["image"]).is_file(), (
                    f"{grid}: no {document['image']}"
                )


def test_check_2953_every_map_folder_says_where_it_came_from() -> None:
    """CHECK-2953: a README.md with the provenance fields. The value may be
    "not measured" for accuracy only; the numbers are not checked here."""
    for track in _tracks():
        for folder in _map_folders(track):
            readme = folder / "README.md"
            assert readme.is_file(), f"{folder}: no README.md"
            text = readme.read_text(encoding="utf-8")
            missing = [f for f in README_FIELDS if f not in text]
            assert not missing, f"{readme}: no {missing}"
