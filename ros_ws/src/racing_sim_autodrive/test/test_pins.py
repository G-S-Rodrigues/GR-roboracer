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
