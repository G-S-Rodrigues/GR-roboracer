"""Where a track's committed data lives, in one place (D2, ADR 0009).

ROS-free. A track is a folder `<tracks root>/<Map>/` of the data repo. The
survey grid, the MOLA map and the optimized raceline are all derived from it,
so a run can only be localized and driven on its own track.
"""

from pathlib import Path

import yaml

SURVEY = ("slam_toolbox", "res_0050")  # amcl's map; another by code change
MAP_METHODS = ("slam_toolbox", "mola", "far_lio")
SUBMODULE_HINT = (
    "; the track data is the GR-roboracer-data submodule, run "
    "`git submodule update --init third_party/GR-roboracer-data` "
    "on the host"
)


class NoRacelineForVersion(ValueError):
    """A raceline is committed for the optimizer, but not for this version."""


def track_dir(maps_root: Path, map_name: str) -> Path:
    """The folder of `map_name` under the scenario's `map_directory`."""
    return Path(maps_root) / map_name


def survey_grid(track: Path) -> Path:
    """amcl's occupancy grid: the slam_toolbox survey of this track."""
    track = Path(track)
    return track / "maps" / Path(*SURVEY) / f"{track.name}.yaml"


def mola_files(track: Path) -> tuple[str, str]:
    """(.mm, provenance yaml) of the MOLA map built for `track`.

    Raises FileNotFoundError, naming the script that builds it, when absent.
    """
    track = Path(track)
    mola_dir = track / "maps" / "mola"
    mm = mola_dir / f"{track.name}.mm"
    provenance = mola_dir / f"{track.name}.yaml"
    for path in (mm, provenance):
        if not path.is_file():
            raise FileNotFoundError(
                f"no MOLA map {path}; build it with scripts/mola_map_track.sh"
            )
    return str(mm), str(provenance)


def optimized_raceline(
    track: Path,
    optimizer: str,
    vehicle_version: str,
    track_stem: str | None = None,
) -> Path:
    """The committed raceline of `optimizer` for `vehicle_version`.

    The line is `racelines/<optimizer>/<limit set>/raceline.csv` whose
    sidecar records `vehicle_version`; the limit-set folder is not the
    version. Fails closed: no match, two matches, a sidecar that disagrees or
    a verdict that is not ok raises, and the centerline is never a fallback.
    """
    track = Path(track)
    folder = track / "racelines" / optimizer
    matches = []
    seen = []
    for csv in sorted(folder.glob("*/raceline.csv")):
        sidecar_path = csv.with_suffix(".yaml")
        if not sidecar_path.is_file():
            raise FileNotFoundError(f"no raceline sidecar at {sidecar_path}")
        sidecar = yaml.safe_load(sidecar_path.read_text(encoding="utf-8"))
        seen.append(sidecar.get("vehicle_version"))
        if sidecar.get("vehicle_version") == vehicle_version:
            matches.append((csv, sidecar_path, sidecar))
    if not matches:
        message = (
            f"no committed raceline for vehicle_version {vehicle_version!r} "
            f"under {folder}"
        )
        if seen:
            # A line is there but records another version: its sidecar
            # disagrees with the vehicle asked for, as before D2.
            raise NoRacelineForVersion(
                f"{message}; the sets there carry {seen}"
            )
        hint = SUBMODULE_HINT if not track.parent.is_dir() else ""
        raise FileNotFoundError(f"{message}{hint}")
    if len(matches) > 1:
        folders = ", ".join(str(csv.parent) for csv, _, _ in matches)
        raise ValueError(
            f"{len(matches)} limit sets carry vehicle_version "
            f"{vehicle_version!r}: {folders}; one version names one set"
        )
    csv, sidecar_path, sidecar = matches[0]
    expected_values = [("optimizer", optimizer)]
    if track_stem is not None:
        expected_values.insert(0, ("track", track_stem))
    for key, expected in expected_values:
        if sidecar.get(key) != expected:
            raise ValueError(
                f"{sidecar_path}: {key} is {sidecar.get(key)!r}, "
                f"expected {expected!r}"
            )
    if (sidecar.get("verdict") or {}).get("ok") is not True:
        raise ValueError(f"{sidecar_path}: verdict.ok is not true")
    return csv
