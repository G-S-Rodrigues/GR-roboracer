from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

from sim.track_importer import import_track


def _world_pixel(
    image: Image.Image,
    metadata: dict,
    point: np.ndarray,
) -> int:
    origin_x, origin_y, _ = metadata["origin"]
    resolution = metadata["resolution"]
    pixel_x = round((float(point[0]) - origin_x) / resolution)
    pixel_y = (
        image.height - 1 - round((float(point[1]) - origin_y) / resolution)
    )
    return int(image.getpixel((pixel_x, pixel_y)))


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


def test_closed_loop_has_no_occupied_closing_seam(tmp_path: Path) -> None:
    source = Path("config/tracks/analytic_circle.yaml")
    track_dir = import_track(source, tmp_path, resolution=0.05)
    metadata = yaml.safe_load(
        (track_dir / "analytic_circle.yaml").read_text(encoding="utf-8")
    )
    image = Image.open(track_dir / "analytic_circle.png")
    canonical = yaml.safe_load(source.read_text(encoding="utf-8"))
    centerline = np.asarray(canonical["centerline"], dtype=float)[:, :2]
    closed_centerline = np.vstack((centerline, centerline[0]))

    occupied_samples = []
    for start, end in zip(
        closed_centerline[:-1], closed_centerline[1:], strict=True
    ):
        for fraction in np.linspace(0.0, 1.0, 21, endpoint=False):
            point = start + fraction * (end - start)
            if _world_pixel(image, metadata, point) != 255:
                occupied_samples.append(point.tolist())

    assert occupied_samples == []
