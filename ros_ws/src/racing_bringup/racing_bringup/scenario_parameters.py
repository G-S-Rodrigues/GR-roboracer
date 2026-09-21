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


def track_parameters(scenario_path: Path) -> dict[str, dict[str, Any]]:
    """Return {node name: parameter overrides} for one scenario file."""
    scenario_path = Path(scenario_path).resolve()
    document = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    base = scenario_path.parent
    track = str((base / document["track"]).resolve())
    map_name = str(document["environment"]["map"])
    raceline = str(
        (
            base
            / document["map_directory"]
            / map_name
            / f"{map_name}_raceline.csv"
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
    }
