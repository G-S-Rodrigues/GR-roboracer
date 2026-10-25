"""Guards on the AutoDRIVE compose services and the pinned-image fragment.

Plain file parses: there is no docker CLI in `dev` and CI never runs compose,
so what is checked here is the text, not what compose resolves it to.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPOSE = REPO_ROOT / "docker" / "docker-compose.yaml"
FRAGMENT = REPO_ROOT / "config" / "autodrive" / "simulator.yaml"
IMAGE = re.compile(
    r"^autodriveecosystem/(?P<name>[a-z0-9_]+):(?P<tag>[A-Za-z0-9._-]+)"
    r"@sha256:(?P<digest>[0-9a-f]{64})$"
)
AUTODRIVE = ("autodrive-sim", "autodrive-devkit")


def _compose() -> dict:
    return yaml.safe_load(COMPOSE.read_text())["services"]


def test_check_1610_autodrive_services_are_profiled() -> None:
    """CHECK-1610: `autodrive-sim` and `autodrive-devkit` exist only under
    profile `autodrive`, extend the config fragment and name no image
    themselves; `dev` and `fast-sim` carry no profile, so a default `up`
    still starts exactly what it did."""
    services = _compose()
    for name in ("dev", "fast-sim"):
        assert "profiles" not in services[name], f"{name} must stay default"
    for name in AUTODRIVE:
        assert name in services, f"{name} missing from docker-compose.yaml"
        service = services[name]
        assert service.get("profiles") == ["autodrive"], name
        assert "image" not in service, f"{name} names an image itself"
        ext = service.get("extends")
        assert ext and ext["service"] in ("sim", "devkit"), name
        target = (COMPOSE.parent / ext["file"]).resolve()
        assert target == FRAGMENT, f"{name} extends {target}"


def test_check_1620_fragment_pins_both_images_to_one_edition() -> None:
    """CHECK-1620: the fragment holds one `x-autodrive.edition`, and each
    image is `autodriveecosystem/<name>:<edition>@sha256:<64 hex>` with the
    tag equal to the edition in both."""
    data = yaml.safe_load(FRAGMENT.read_text())
    edition = data["x-autodrive"]["edition"]
    assert edition, "no edition"
    seen = set()
    for key in ("sim", "devkit"):
        image = data["services"][key]["image"]
        match = IMAGE.match(image)
        assert match, f"{key}: {image!r} is not name:tag@sha256:<64 hex>"
        assert match["tag"] == edition, f"{key}: tag {match['tag']} != edition"
        seen.add(match["name"])
    assert len(seen) == 2, "sim and devkit must be two different images"


def test_check_1630_sim_command_waits_for_bridge_and_escapes_dollars() -> None:
    """CHECK-1630: the sim runs headless on port 4567, takes the bridge host
    from AUTODRIVE_BRIDGE_HOST (default `autodrive-devkit`), waits for the
    bridge port before exec (it never reconnects), and writes every shell `$`
    as `$$` so compose does not interpolate it from the host."""
    service = _compose()["autodrive-sim"]
    command = service["command"]
    assert command[0] == "-c" and len(command) == 2, command
    script = command[1]
    for token in ("-batchmode", "-nographics", "-port 4567", 'exec "./'):
        assert token in script, f"{token!r} missing from the sim script"
    assert "/dev/tcp/$$AUTODRIVE_BRIDGE_HOST/4567" in script
    assert script.index("/dev/tcp/") < script.index('exec "./'), "wait first"
    assert "getent hosts" in script
    bare = re.findall(r"(?<!\$)\$(?!\$)", script)
    assert not bare, "bare $ in the sim script; compose would interpolate it"
    env = service["environment"]
    assert (
        env["AUTODRIVE_BRIDGE_HOST"]
        == "${AUTODRIVE_BRIDGE_HOST:-autodrive-devkit}"
    )
    devkit = _compose()["autodrive-devkit"]
    assert devkit["environment"]["ROS_DOMAIN_ID"] == (
        "${AUTODRIVE_ROS_DOMAIN_ID:-42}"
    )
