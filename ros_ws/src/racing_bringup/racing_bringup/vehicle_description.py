"""Which URDF draws which vehicle.

`racing_vehicle_description/urdf/descriptions.yaml` maps a vehicle file's stem
to a URDF. racing_vehicle_description's display.launch.py carries a 3-line copy
of this lookup (it must not depend on this package); BRINGUP-1056 and DESC-1090
pin both to the one map.
"""

from pathlib import Path

import yaml


def description_for(vehicle: str, share: Path) -> Path:
    """The URDF in `share` (racing_vehicle_description's share directory)
    that draws `vehicle`, a vehicle file path; only its stem is used."""
    share = Path(share)
    stem = Path(vehicle).stem
    mapping = yaml.safe_load(
        (share / "urdf" / "descriptions.yaml").read_text(encoding="utf-8")
    )
    if stem not in mapping:
        raise RuntimeError(f"no description for vehicle {stem}")
    return share / "urdf" / mapping[stem]
