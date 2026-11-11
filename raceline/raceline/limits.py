"""The ``limits.md`` beside a committed raceline: what it was optimized under.

The table between ``<!-- limits:begin -->`` and ``<!-- limits:end -->`` is
rendered from the record and the vehicle tables, so it cannot drift from them;
the ``## Provenance`` section below it is written by a human and kept.
"""

import re
from pathlib import Path

from raceline.vehicle import VehicleLimits

BEGIN = "<!-- limits:begin -->"
END = "<!-- limits:end -->"
PROVENANCE = "## Provenance"
STUB = (
    "TODO: say which simulator, where the numbers come from, which caps and "
    "which grip basis, with the vehicle-file lines that justify each."
)


def _ini_number(ini: str, key: str) -> str:
    found = re.search(rf'"{key}": *(-?[0-9.eE+-]+)', ini or "")
    return found.group(1) if found else "n/a"


def render(record: dict, vehicle: VehicleLimits) -> str:
    """The marked table block (markers included) for ``record``."""
    params = record["parameters"]
    ini = params.get("ini", "")
    veh = record["vehicle"]
    ggv_ay = float(vehicle.ggv[0, 2])
    rows = [
        ("limit set", record["limit_set"]),
        ("vehicle version", record["vehicle_version"]),
        ("optimizer", record["optimizer"]),
        ("v_max (m/s)", _ini_number(ini, "v_max")),
        ("ax cap (m/s^2)", f"{vehicle.a_x_max:g}"),
        ("brake cap (m/s^2)", "= ax (GGV)"),
        ("ay, GGV at v=0 (m/s^2)", f"{ggv_ay:.6f}"),
        ("mue used", _ini_number(ini, "mue") if "mue" in ini else "n/a"),
        ("width_opt (m)", _ini_number(ini, "width_opt")),
        ("safety_width (m)", f"{params['safety_width']:.4f}"),
        ("margin (m per side)", f"{params['margin']:g}"),
        ("curvlim (1/m)", _ini_number(ini, "curvlim")),
        ("tyre set", str(params.get("tyre_set", "n/a"))),
        ("vehicle file", f"`{veh['path']}` sha256 `{veh['sha256']}`"),
        ("ggv.csv sha256", f"`{veh['ggv_sha256']}`"),
        ("ax_max_machines.csv sha256", f"`{veh['ax_max_machines_sha256']}`"),
    ]
    lines = [BEGIN, "", "| Limit | Value |", "|---|---|"]
    lines += [f"| {k} | {v} |" for k, v in rows]
    lines += ["", END]
    return "\n".join(lines)


def write(
    path: Path, record: dict, vehicle: VehicleLimits, title: str | None = None
) -> None:
    """Write ``limits.md``: replace the table block, keep ``## Provenance``."""
    path = Path(path)
    block = render(record, vehicle)
    if path.is_file():
        text = path.read_text(encoding="utf-8")
        if BEGIN in text and END in text:
            head, rest = text.split(BEGIN, 1)
            _, tail = rest.split(END, 1)
            path.write_text(head + block + tail, encoding="utf-8")
            return
    head = f"# {title or record['limit_set']}\n\n"
    path.write_text(
        f"{head}{block}\n\n{PROVENANCE}\n\n{STUB}\n", encoding="utf-8"
    )


def has_provenance(text: str) -> bool:
    """A real ``## Provenance`` section: present and not the stub."""
    if PROVENANCE not in text:
        return False
    body = text.split(PROVENANCE, 1)[1].strip()
    return bool(body) and STUB not in body
