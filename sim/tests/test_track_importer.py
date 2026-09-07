from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

from sim.track_importer import import_track


def test_importer_writes_the_four_gym_map_formats(tmp_path: Path) -> None:
    source = Path("config/tracks/analytic_circle.yaml")
    track_dir = import_track(source, tmp_path, resolution=0.1)

    expected = {
        "analytic_circle.yaml",
        "analytic_circle.png",
        "analytic_circle_centerline.csv",
        "analytic_circle_raceline.csv",
    }
    assert {path.name for path in track_dir.iterdir()} == expected

    map_metadata = yaml.safe_load(
        (track_dir / "analytic_circle.yaml").read_text(encoding="utf-8")
    )
    assert map_metadata["image"] == "analytic_circle.png"
    assert map_metadata["resolution"] == pytest.approx(0.1)
    assert len(map_metadata["origin"]) == 3

    centerline = np.loadtxt(
        track_dir / "analytic_circle_centerline.csv", delimiter=","
    )
    raceline = np.loadtxt(
        track_dir / "analytic_circle_raceline.csv", delimiter=";"
    )
    assert centerline.shape == (8, 4)
    assert raceline.shape == (8, 7)
    np.testing.assert_allclose(centerline[0], [10.0, 0.0, 2.0, 3.0])
    assert set(np.unique(Image.open(track_dir / "analytic_circle.png"))) == {
        0,
        255,
    }
