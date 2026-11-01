"""Tier 1: the dev image carries the recorder's Socket.IO closure (SIMAD-1150).

Our server runs in `dev`, so `docker/dev-tools-requirements.txt` must hold the
same Linux closure `tools/autodrive_record.py.lock` pinned when part 1 found
the pair the sim can talk to (gotcha 30). A bump of one without the other
would change the protocol silently.
"""

import re
import tomllib
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
LOCK = REPO / "tools" / "autodrive_record.py.lock"
DEV_REQUIREMENTS = REPO / "docker" / "dev-tools-requirements.txt"

# Debian ships six 1.16.0 (satisfies python-socketio's six>=1.9) and
# pyyaml; pinning either in the image would fight apt (gotcha 27).
DEBIAN_OWNED = frozenset({"six", "pyyaml"})
# In the lock only behind markers that are false on Linux/CPython 3.12:
# cffi needs sys_platform == 'win32', pycparser needs it through cffi.
NOT_LINUX_CPYTHON = frozenset({"cffi", "pycparser"})
# Not in the lock: the replay client's WebSocket transport (plan D3).
EXTRA_PINS = frozenset({"websocket-client"})


def _norm(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def lock_versions(text: str) -> dict[str, str]:
    """Every package in a uv.lock-style text, name -> version."""
    data = tomllib.loads(text)
    return {_norm(p["name"]): p["version"] for p in data["package"]}


def requirement_pins(text: str) -> dict[str, str]:
    """`name==version` lines of a requirements text, name -> version."""
    pins: dict[str, str] = {}
    for line in text.splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            name, version = line.split("==")
            pins[_norm(name)] = version
    return pins


def mismatches(lock: dict[str, str], pins: dict[str, str]) -> list[str]:
    """What the requirements text gets wrong against the lock's closure."""
    out = []
    for name, version in sorted(lock.items()):
        if name in DEBIAN_OWNED | NOT_LINUX_CPYTHON:
            continue
        if pins.get(name) != version:
            out.append(f"{name}: lock {version}, requirements {pins.get(name)}")
    for name in sorted(EXTRA_PINS):
        if name not in pins:
            out.append(f"{name}: not pinned")
    return out


def test_simad_1150_dev_requirements_pin_the_recorder_closure() -> None:
    """SIMAD-1150: every Linux package of the recorder lock, bar the
    Debian-owned and the Windows/PyPy-only ones listed above, is pinned in the
    dev requirements at the same version, and websocket-client is pinned."""
    lock = lock_versions(LOCK.read_text())
    assert set(lock) >= DEBIAN_OWNED | NOT_LINUX_CPYTHON, "stale exclusions"
    problems = mismatches(lock, requirement_pins(DEV_REQUIREMENTS.read_text()))
    assert not problems, "; ".join(problems)


def test_simad_1150_negative_a_bumped_pin_is_reported() -> None:
    """SIMAD-1150 negative: the comparison is not vacuous."""
    lock = {"gevent": "26.9.0", "six": "1.17.0", "cffi": "2.1.1"}
    good = {"gevent": "26.9.0", "websocket-client": "1.9.2"}
    assert mismatches(lock, good) == []
    bumped = {"gevent": "26.9.1", "websocket-client": "1.9.2"}
    assert mismatches(lock, bumped) == [
        "gevent: lock 26.9.0, requirements 26.9.1"
    ]
    assert mismatches(lock, {"gevent": "26.9.0"}) == [
        "websocket-client: not pinned"
    ]


def installed_mismatches(pins: dict[str, str], version_of) -> list[str]:
    """Pins the running interpreter does not have, by `version_of(name)`
    (raising PackageNotFoundError when absent)."""
    from importlib.metadata import PackageNotFoundError

    out = []
    for name, wanted in sorted(pins.items()):
        try:
            have = version_of(name)
        except PackageNotFoundError:
            have = None
        if have != wanted:
            out.append(f"{name}: pinned {wanted}, installed {have}")
    return out


def test_simad_1150_the_running_image_carries_the_pinned_closure() -> None:
    """SIMAD-1150: the interpreter running the tests has every Socket.IO
    closure pin installed. A stale `dev` image (rebuilt after the merge, ADR
    0002) then fails here, by name, instead of as "racing_sim not found" or
    "replay stalled" in the tier-2 server tests."""
    from importlib.metadata import version

    wanted = {
        name: pinned
        for name, pinned in requirement_pins(
            DEV_REQUIREMENTS.read_text()
        ).items()
        if name
        in {
            "python-socketio",
            "python-engineio",
            "gevent",
            "gevent-websocket",
            "greenlet",
            "zope-event",
            "zope-interface",
            "websocket-client",
        }
    }
    assert len(wanted) == 8
    problems = installed_mismatches(wanted, version)
    assert not problems, (
        "stale image, rebuild gr-roboracer:dev (ADR 0002) or put the "
        "closure on PYTHONPATH: " + "; ".join(problems)
    )


def test_simad_1150_negative_a_missing_or_old_package_is_reported() -> None:
    """SIMAD-1150 negative: the installed-version check is not vacuous."""
    from importlib.metadata import PackageNotFoundError

    def version_of(name: str) -> str:
        if name == "gevent":
            return "26.9.0"
        if name == "old":
            return "1.0"
        raise PackageNotFoundError(name)

    assert installed_mismatches(
        {"gevent": "26.9.0", "old": "2.0", "absent": "1.0"}, version_of
    ) == [
        "absent: pinned 1.0, installed None",
        "old: pinned 2.0, installed 1.0",
    ]
