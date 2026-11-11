"""track_data: every derived path of a track's committed data (D2)."""

from pathlib import Path

import pytest
import yaml
from racing_bringup import track_data

HINT = "git submodule update --init third_party/GR-roboracer-data"


def _record(folder: Path, **fields) -> None:
    folder.mkdir(parents=True)
    (folder / "raceline.csv").write_text("x,y\n", encoding="utf-8")
    document = {
        "track": "spielberg",
        "optimizer": "tum_mintime",
        "vehicle_version": "f1tenth-default-v1",
        "limit_set": folder.name,
        "verdict": {"ok": True},
    }
    document.update(fields)
    (folder / "raceline.yaml").write_text(
        yaml.safe_dump(document), encoding="utf-8"
    )


def _tree(tmp_path: Path) -> tuple[Path, Path]:
    """A tracks root in the D2 layout; returns (root, Spielberg's folder)."""
    root = tmp_path / "tracks"
    track = root / "Spielberg"
    for rel in (
        "maps/slam_toolbox/res_0050/Spielberg.yaml",
        "maps/mola/Spielberg.mm",
        "maps/mola/Spielberg.yaml",
    ):
        path = track / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("x", encoding="utf-8")
    _record(track / "racelines" / "tum_mintime" / "gym-nominal-std-const-v1")
    return root, track


def test_bringup_2600_the_survey_grid_is_the_slam_toolbox_variant(
    tmp_path: Path,
) -> None:
    """BRINGUP-2600: the track folder is `<maps>/<Map>` and amcl's grid is
    its slam_toolbox survey at 0.050 m."""
    root, track = _tree(tmp_path)
    assert track_data.track_dir(root, "Spielberg") == track
    assert (
        track_data.survey_grid(track)
        == track / "maps" / "slam_toolbox" / "res_0050" / "Spielberg.yaml"
    )


def test_bringup_2601_the_mola_map_is_found_or_names_its_script(
    tmp_path: Path,
) -> None:
    """BRINGUP-2601 (BRINGUP-1500's contract, moved): `mola_files` returns the
    .mm and its provenance, and an unbuilt map names the script."""
    root, track = _tree(tmp_path)
    mm, provenance = track_data.mola_files(track)
    assert Path(mm) == track / "maps" / "mola" / "Spielberg.mm"
    assert Path(provenance) == track / "maps" / "mola" / "Spielberg.yaml"
    with pytest.raises(FileNotFoundError, match="mola_map_track.sh"):
        track_data.mola_files(root / "Nowhere")


def test_bringup_2602_the_raceline_is_found_by_vehicle_version(
    tmp_path: Path,
) -> None:
    """BRINGUP-2602: the one `racelines/<optimizer>/*/raceline.csv` whose
    sidecar names the vehicle version; the folder name is not the version."""
    _, track = _tree(tmp_path)
    found = track_data.optimized_raceline(
        track, "tum_mintime", "f1tenth-default-v1", track_stem="spielberg"
    )
    assert (
        found
        == track
        / "racelines"
        / "tum_mintime"
        / "gym-nominal-std-const-v1"
        / "raceline.csv"
    )
    with pytest.raises(ValueError, match="track is 'spielberg'"):
        track_data.optimized_raceline(
            track, "tum_mintime", "f1tenth-default-v1", track_stem="porto"
        )


def test_bringup_2603_no_match_says_no_committed_raceline(
    tmp_path: Path,
) -> None:
    """BRINGUP-2603: no line for the optimizer is a FileNotFoundError; a line
    whose sidecar records another version is a ValueError (the sidecar
    disagrees with the vehicle asked for, as BRINGUP-1160 pins)."""
    _, track = _tree(tmp_path)
    with pytest.raises(
        FileNotFoundError, match="no committed raceline for vehicle_version"
    ):
        track_data.optimized_raceline(
            track, "tum_mincurv_iqp", "f1tenth-default-v1"
        )
    with pytest.raises(
        ValueError, match="no committed raceline for vehicle_version"
    ):
        track_data.optimized_raceline(track, "tum_mintime", "other-v1")


def test_bringup_2604_two_sets_for_one_version_name_both(
    tmp_path: Path,
) -> None:
    """BRINGUP-2604: the day two limit sets share a vehicle version the
    lookup is ambiguous, and it says so naming both folders (Q7)."""
    _, track = _tree(tmp_path)
    _record(track / "racelines" / "tum_mintime" / "gym-model-std-const-v1")
    with pytest.raises(ValueError) as error:
        track_data.optimized_raceline(
            track, "tum_mintime", "f1tenth-default-v1"
        )
    assert "gym-nominal-std-const-v1" in str(error.value)
    assert "gym-model-std-const-v1" in str(error.value)


def test_bringup_2605_the_init_hint_only_when_the_root_is_absent(
    tmp_path: Path,
) -> None:
    """BRINGUP-2605: a missing raceline names the submodule init only when
    the tracks root is absent (an uninitialised submodule)."""
    root, track = _tree(tmp_path)
    with pytest.raises(FileNotFoundError) as present:
        track_data.optimized_raceline(
            track, "tum_mincurv_iqp", "f1tenth-default-v1"
        )
    assert HINT not in str(present.value)

    import shutil

    shutil.rmtree(root)
    with pytest.raises(FileNotFoundError) as absent:
        track_data.optimized_raceline(
            track, "tum_mincurv_iqp", "f1tenth-default-v1"
        )
    assert HINT in str(absent.value)
