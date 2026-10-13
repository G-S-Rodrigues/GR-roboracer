"""Tier 1: the map -> track extractor and the canonical track file."""

from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
SPIELBERG = REPO / "config" / "tracks" / "spielberg.yaml"


def _asymmetric_track():
    from raceline.track import Track

    t = np.linspace(0.0, 2 * np.pi, 40, endpoint=False)
    return Track(
        name="oval",
        xy=np.column_stack([8.0 * np.cos(t), 5.0 * np.sin(t)]),
        curvature=0.1 * np.cos(t),
        w_left=1.0 + 0.2 * np.sin(t) ** 2,
        w_right=0.7 + 0.1 * np.cos(t) ** 2,
    )


def test_raceline_1100_track_yaml_matches_the_committed_format(tmp_path):
    """RACELINE-1100: load/write the canonical track file (left before right)."""
    from raceline.track import load_track, write_track

    # the committed file: 5 columns in canonical order, widths 1.1 / 1.1
    spielberg = load_track(SPIELBERG)
    assert spielberg.name == "Spielberg"
    assert spielberg.xy.shape == (864, 2)
    assert np.allclose(spielberg.w_left, 1.1) and np.allclose(
        spielberg.w_right, 1.1
    )
    assert np.allclose(spielberg.xy[0], [0.0, 0.0])
    assert spielberg.curvature[0] == pytest.approx(0.0000471)

    # round trip of an asymmetric track: left and right must not swap
    track = _asymmetric_track()
    out = write_track(track, tmp_path / "t.yaml", source="unit test")
    back = load_track(out)
    assert back.name == "oval"
    assert np.allclose(back.xy, track.xy, atol=1e-8)
    assert np.allclose(back.curvature, track.curvature, atol=1e-8)
    assert np.allclose(back.w_left, track.w_left, atol=1e-6)
    assert np.allclose(back.w_right, track.w_right, atol=1e-6)
    assert not np.allclose(back.w_left, back.w_right)
    assert out.read_text().startswith("# unit test")

    # rejected, as racing_common's track.cpp rejects them
    from dataclasses import replace

    with pytest.raises(ValueError):
        write_track(replace(track, w_left=track.w_left * 0), tmp_path / "a")
    with pytest.raises(ValueError):
        write_track(
            replace(track, xy=np.vstack([track.xy[:1], track.xy])),
            tmp_path / "b",
        )
    # stricter than track.cpp: fewer than 3 points
    with pytest.raises(ValueError):
        write_track(
            replace(
                track,
                xy=track.xy[:2],
                curvature=track.curvature[:2],
                w_left=track.w_left[:2],
                w_right=track.w_right[:2],
            ),
            tmp_path / "c",
        )
    good = tmp_path / "g.yaml"
    write_track(track, good)
    text = good.read_text()
    for name, bad in (
        ("v", text.replace("format_version: 1", "format_version: 2")),
        ("u", text.replace("units: meters", "units: feet")),
        ("o", text.replace("closed: true", "closed: false")),
        ("r", text.replace("  - [", "  - [1.0, ", 1)),
    ):
        p = tmp_path / f"bad_{name}.yaml"
        p.write_text(bad)
        with pytest.raises(ValueError):
            load_track(p)
