"""ROS-free: the per-node parameters a scenario file implies.

The scenario file already names the world (`track`, `map_directory`,
`environment.map`). Every track-aware node must load that same world, so its
parameters are derived here rather than hand-set in the vehicle file, where a
second, disagreeing copy would go unnoticed (repo-gotchas #15).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

# Optional scenario keys that set the identity racing_metrics and
# racing_recording report. Absent, the vehicle file's values stand - which is
# what keeps the analytic_circle golden's record unchanged.
REPORTING_KEYS = ("scenario_id", "track_version")


PLANNERS = ("track_centerline", "optimized_raceline")
RACELINE_OPTIMIZERS = ("tum_mincurv_iqp",)
# The simulators the launch can start (config/reference_stack.yaml's
# `simulator` role: BRINGUP-1600), and which front end drives AutoDRIVE's.
SIMULATORS = ("f1tenth_gym_jax", "autodrive")
AUTODRIVE_BRIDGES = ("ours", "official")
# Vehicle files the gym cannot cast a scan for: the launch refuses them on the
# gym and points at `simulator:=autodrive`. BRINGUP-1650 keeps this equal to
# the contract test's NOT_GYM_CAST.
GYM_INCOMPATIBLE_VEHICLES = frozenset({"autodrive_roboracer"})


def _vehicle_version(vehicle_path: Path) -> str:
    """The vehicle parameter version both reporting nodes agree on.

    Duplicates raceline/raceline/vehicle.py's reader: ros_ws never imports the
    uv project (ADR 0003).
    """
    document = yaml.safe_load(Path(vehicle_path).read_text(encoding="utf-8"))
    versions = {
        name: ((document.get(name) or {}).get("ros__parameters") or {}).get(
            "vehicle_parameter_version"
        )
        for name in ("racing_metrics", "racing_recording")
    }
    if None in versions.values() or len(set(versions.values())) != 1:
        raise ValueError(
            f"{vehicle_path}: vehicle_parameter_version must be set and equal "
            f"under racing_metrics and racing_recording, got {versions}"
        )
    return str(versions["racing_metrics"])


def _optimized_raceline(
    track_file: Path, optimizer: str, vehicle_path: Path | None
) -> str:
    if vehicle_path is None:
        raise ValueError("planner optimized_raceline needs a vehicle_path")
    version = _vehicle_version(vehicle_path)
    csv = (
        track_file.parent.parent
        / "racelines"
        / track_file.stem
        / version
        / f"{optimizer}.csv"
    )
    if not csv.is_file():
        raise FileNotFoundError(f"no committed raceline at {csv}")
    sidecar_path = csv.with_suffix(".yaml")
    if not sidecar_path.is_file():
        raise FileNotFoundError(f"no raceline sidecar at {sidecar_path}")
    sidecar = yaml.safe_load(sidecar_path.read_text(encoding="utf-8"))
    for key, expected in (
        ("track", track_file.stem),
        ("vehicle_version", version),
        ("optimizer", optimizer),
    ):
        if sidecar.get(key) != expected:
            raise ValueError(
                f"{sidecar_path}: {key} is {sidecar.get(key)!r}, "
                f"expected {expected!r}"
            )
    if (sidecar.get("verdict") or {}).get("ok") is not True:
        raise ValueError(f"{sidecar_path}: verdict.ok is not true")
    return str(csv)


def track_parameters(
    scenario_path: Path,
    *,
    planner: str = "track_centerline",
    raceline_optimizer: str = "tum_mincurv_iqp",
    vehicle_path: Path | None = None,
) -> dict[str, dict[str, Any]]:
    """Return {node name: parameter overrides} for one scenario file.

    `planner="optimized_raceline"` swaps the scenario's own
    `<map>_raceline.csv` for the committed
    `config/racelines/<track>/<vehicle version>/<optimizer>.csv`. It fails
    closed: an unknown name, a missing file or a sidecar that disagrees raises,
    and the centerline is never a fallback. Under the default
    `track_centerline`, a map with no `<map>_raceline.csv` gets an empty
    `raceline_path`, which means "drive the centerline".
    """
    if planner not in PLANNERS:
        raise ValueError(f"unknown planner {planner!r}; one of {PLANNERS}")
    if raceline_optimizer not in RACELINE_OPTIMIZERS:
        raise ValueError(
            f"unknown raceline_optimizer {raceline_optimizer!r}; "
            f"one of {RACELINE_OPTIMIZERS}"
        )
    scenario_path = Path(scenario_path).resolve()
    document = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    base = scenario_path.parent
    track_file = (base / document["track"]).resolve()
    track = str(track_file)
    map_name = str(document["environment"]["map"])
    raceline = str(
        (
            base
            / document["map_directory"]
            / map_name
            / f"{map_name}_raceline.csv"
        ).resolve()
    )
    if planner == "optimized_raceline":
        raceline = _optimized_raceline(
            track_file, raceline_optimizer, vehicle_path
        )
    elif not Path(raceline).is_file():
        # A map with no raceline of its own (Porto's is part 11's): the
        # support node then publishes the track's centerline (BRINGUP-1580).
        raceline = ""
    # The track's committed occupancy grid, surveyed by scripts/map_track.sh
    # and served to nav2_amcl by nav2_map_server. Derived from the scenario's
    # own map for the same reason the raceline is: a hand-set grid path is a
    # second copy of which world this is, and localizing against another
    # track's walls is silent, not an error.
    occupancy_grid = str(
        (
            base
            / document["map_directory"]
            / map_name
            / "slam"
            / f"{map_name}.yaml"
        ).resolve()
    )
    reported = document.get("metrics") or {}
    reporting = {
        key: reported[key] for key in REPORTING_KEYS if key in reported
    }
    return {
        "racing_safety_supervisor": {"track_path": track},
        "racing_bringup_support": {
            "track_path": track,
            "raceline_path": raceline,
        },
        "racing_metrics": {"track_path": track, **reporting},
        "racing_evaluation": {"track_path": track},
        "racing_recording": dict(reporting),
        "map_server": {"yaml_filename": occupancy_grid},
    }
