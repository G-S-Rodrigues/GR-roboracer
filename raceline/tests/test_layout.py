"""Tier 1: the committed racelines' layout (RACELINE-1600..1603, 1620).

Reads the data repo (``third_party/GR-roboracer-data``): every optimizer folder
under ``racelines/`` holds one folder per limit set, and the set names one
vehicle version and one set of limit tables.
"""

from pathlib import Path

import pytest
import yaml
from raceline.layout import LIMIT_SET_RE
from raceline.vehicle import load_vehicle

from raceline import limits

REPO = Path(__file__).resolve().parents[2]
TRACKS = REPO / "third_party" / "GR-roboracer-data" / "tracks"
OPTIMIZERS = ("tum_mincurv_iqp", "tum_mintime")


def _sets() -> list[Path]:
    found = sorted(TRACKS.glob("*/racelines/*/*"))
    assert found, (
        f"no raceline folders under {TRACKS}; run "
        "`git submodule update --init third_party/GR-roboracer-data`"
    )
    return found


def _record(folder: Path) -> dict:
    return yaml.safe_load((folder / "raceline.yaml").read_text())


def test_raceline_1600_folders_are_optimizers_holding_limit_sets():
    sets = _sets()
    for racelines in sorted(TRACKS.glob("*/racelines")):
        for child in racelines.iterdir():
            assert child.name in OPTIMIZERS, child
    for folder in sets:
        assert LIMIT_SET_RE.fullmatch(folder.name), folder


def test_raceline_1601_a_set_folder_holds_exactly_its_files():
    for folder in _sets():
        want = {"raceline.csv", "raceline.yaml", "limits.md"}
        if folder.parent.name == "tum_mintime":
            want.add("reference.csv")
        assert {p.name for p in folder.iterdir()} == want, folder


def test_raceline_1602_the_record_agrees_with_its_folder_and_is_portable():
    for folder in _sets():
        doc = _record(folder)
        assert doc["limit_set"] == folder.name, folder
        assert doc["optimizer"] == folder.parent.name, folder
        assert "--out-dir" not in doc["command"], folder
        paths = [
            doc["source"]["path"],
            doc["vehicle"]["path"],
            (doc.get("ground_truth") or {}).get("path"),
            (doc.get("ground_truth_map") or {}).get("path"),
            doc["parameters"].get("reference", {}).get("path"),
        ]
        for path in paths:
            assert path is None or not Path(path).is_absolute(), (folder, path)
        assert f"--limit-set {folder.name}" in doc["command"], folder


def test_raceline_1603_one_limit_set_is_one_version_and_one_set_of_tables():
    by_set: dict[str, list[dict]] = {}
    for folder in _sets():
        by_set.setdefault(folder.name, []).append(_record(folder))
    assert len(by_set) >= 3
    keys = (
        "ggv_sha256",
        "ax_max_machines_sha256",
        "dynamics_sha256",
        "tyres_sha256",
    )
    for name, docs in by_set.items():
        assert len({d["vehicle_version"] for d in docs}) == 1, name
        assert len({d["vehicle"]["path"] for d in docs}) == 1, name
        for key in keys:
            values = {d["vehicle"][key] for d in docs if key in d["vehicle"]}
            assert len(values) <= 1, (name, key)
    versions = [docs[0]["vehicle_version"] for docs in by_set.values()]
    assert len(set(versions)) == len(versions)  # no two sets share a version


@pytest.mark.parametrize(
    "folder",
    [
        pytest.param(f, id=f"{f.parent.name}/{f.name}")
        for f in sorted(TRACKS.glob("*/racelines/*/*"))
    ],
)
def test_raceline_1620_the_committed_limits_md_is_current_and_has_provenance(
    folder,
):
    doc = _record(folder)
    vehicle = load_vehicle(REPO / doc["vehicle"]["path"])
    text = (folder / "limits.md").read_text()
    assert limits.render(doc, vehicle) in text, folder
    assert limits.has_provenance(text), f"{folder}: write ## Provenance"
    provenance = text.split(limits.PROVENANCE, 1)[1].lower()
    for word in ("simulator", "source", "caps", "grip"):
        assert word in provenance, (folder, word)
