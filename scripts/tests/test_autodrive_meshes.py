"""CHECK-2030: the AutoDRIVE meshes carry their licence and match NOTICE.

The model is imported at full resolution (owner, V1 D3): 937,506 triangles as
binary STL, split per link and colour, about 47 MB. Nothing here imports ROS.
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

PACKAGE = (
    Path(__file__).resolve().parents[2]
    / "ros_ws/src/racing_vehicle_description"
)
MESHES = PACKAGE / "meshes"
DIR = MESHES / "autodrive_roboracer"
FBX_TRIANGLES = 937506
# GitHub warns at 50 MB per file.
MAX_BYTES = 50_000_000


def _stls() -> list[Path]:
    files = sorted(DIR.glob("*.stl"))
    assert files, f"no STL under {DIR}"
    return files


def test_check_2030_attribution_is_present() -> None:
    """The BSD-2 text and a NOTICE section naming the source and every hash."""
    licence = MESHES / "LICENSE-autodrive"
    assert licence.is_file(), f"missing {licence}"
    text = licence.read_text()
    assert "BSD 2-Clause License" in text
    assert "Copyright (c) 2021, Tinker Twins" in text
    notice = (MESHES / "NOTICE.md").read_text()
    section = notice.split("## Imported from AutoDRIVE", 1)
    assert len(section) == 2, "NOTICE.md has no 'Imported from AutoDRIVE'"
    body = section[1]
    for needle in (
        "AutoDRIVE-Ecosystem/AutoDRIVE",
        "75b7df5216524a3c6b3150d698139190c6c44f8f",
        "Assets/Models/Vehicle/F1TENTH/F1TENTH.fbx",
        "BSD 2-Clause",
        "LICENSE-autodrive",
        "import_autodrive_meshes.py",
    ):
        assert needle in body, needle
    for stl in _stls():
        digest = hashlib.sha256(stl.read_bytes()).hexdigest()
        assert f"`{stl.name}`: `{digest}`" in body, f"{stl.name}: hash"
    listed = set(re.findall(r"- `([\w.]+\.stl)`: `[0-9a-f]{64}`", body))
    assert listed == {p.name for p in _stls()}


def test_check_2030_meshes_are_whole_binary_stl_under_the_cap() -> None:
    """Every STL is binary (84 + 50 n bytes), under 50 MB, and together they
    hold exactly the FBX's triangles: none dropped, none written twice."""
    total = 0
    for stl in _stls():
        data = stl.read_bytes()
        count = int.from_bytes(data[80:84], "little")
        assert len(data) == 84 + 50 * count, f"{stl.name} is not binary STL"
        assert data[:5].lower() != b"solid", stl.name
        assert len(data) < MAX_BYTES, f"{stl.name}: {len(data)} B"
        total += count
    assert total == FBX_TRIANGLES, total
