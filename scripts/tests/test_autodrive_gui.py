"""Guards on `scripts/autodrive_gui.sh`, the devkit half of the AutoDRIVE
window route (docs/agents/running-wsl.md, "AutoDRIVE GUI").

The window itself is a manual check. What is tested here is behaviour, with a
stub `docker` first on `PATH` that logs its argv and the domain it saw: the
script cannot touch the wrong ROS domain, refuses a taken port, fails when the
bridge never listens, and prints the Windows steps that worked live.
"""

from __future__ import annotations

import os
import stat
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "autodrive_gui.sh"
SHA256 = "2D3E5CC7073009D8F28F41B2266C129DD64210688C70228A6811B844EE48A751"
ASSET = "autodrive_simulator_practice_windows.zip"

STUB = """#!/bin/sh
echo "domain=$AUTODRIVE_ROS_DOMAIN_ID args=$*" >> "$STUB_LOG"
case "$1" in
  ps) [ -n "$STUB_PS" ] && echo "$STUB_PS"; exit 0 ;;
  exec) exit "${STUB_EXEC_RC:-0}" ;;
esac
exit 0
"""


def _run(tmp_path: Path, name: str, **extra: str):
    stub = tmp_path / "docker"
    stub.write_text(STUB)
    stub.chmod(stub.stat().st_mode | stat.S_IXUSR)
    log = tmp_path / f"{name}.log"
    env = {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "STUB_LOG": str(log),
        "AUTODRIVE_GUI_WAIT": "2",
        **extra,
    }
    done = subprocess.run(
        ["bash", str(SCRIPT)], env=env, capture_output=True, text=True
    )
    lines = log.read_text().splitlines() if log.exists() else []
    return done, lines


def test_check_1670_domain_is_validated_and_reaches_compose(tmp_path) -> None:
    """CHECK-1670: an unset, non-integer, out-of-range (over 101) or 42
    `AUTODRIVE_ROS_DOMAIN_ID` is refused before any docker call; a good one
    reaches `docker compose ... run --rm --no-deps -d -p 127.0.0.1:4567:4567
    --name autodrive-devkit-gui autodrive-devkit` in compose's environment;
    nothing starts the whole stack."""
    assert SCRIPT.is_file(), "scripts/autodrive_gui.sh missing"
    for i, domain in enumerate((None, "", "42", "abc", "300", " 7", "-1")):
        extra = {} if domain is None else {"AUTODRIVE_ROS_DOMAIN_ID": domain}
        done, calls = _run(tmp_path, f"bad{i}", **extra)
        assert done.returncode == 2, f"domain {domain!r}: {done.returncode}"
        assert calls == [], f"docker called for domain {domain!r}"
    done, calls = _run(tmp_path, "good", AUTODRIVE_ROS_DOMAIN_ID="66")
    assert done.returncode == 0, done.stderr
    compose = [c for c in calls if "compose" in c]
    assert len(compose) == 1, calls
    assert compose[0].startswith("domain=66 "), compose[0]
    for part in (
        "--profile autodrive run --rm --no-deps -d",
        "-p 127.0.0.1:4567:4567",
        "--name autodrive-devkit-gui autodrive-devkit",
    ):
        assert part in compose[0], compose[0]
    for banned in (" up ", " down ", "--remove-orphans"):
        assert banned not in compose[0], banned


def test_check_1675_taken_port_and_dead_bridge_fail(tmp_path) -> None:
    """CHECK-1675: a container already publishing 4567 gives exit 3 and no
    compose call; a bridge that never listens inside the container gives exit
    1 (no "bridge ready") and the container is stopped."""
    assert SCRIPT.is_file(), "scripts/autodrive_gui.sh missing"
    done, calls = _run(
        tmp_path, "taken", AUTODRIVE_ROS_DOMAIN_ID="66", STUB_PS="abc123"
    )
    assert done.returncode == 3, done.stderr
    assert not any("compose" in c for c in calls), calls
    done, calls = _run(
        tmp_path, "dead", AUTODRIVE_ROS_DOMAIN_ID="66", STUB_EXEC_RC="1"
    )
    assert done.returncode == 1, done.stdout
    assert "bridge ready" not in done.stdout
    assert any("args=stop autodrive-devkit-gui" in c for c in calls), calls


def test_check_1680_printed_windows_steps(tmp_path) -> None:
    """CHECK-1680: the script passes `bash -n`, and on success prints the
    steps that worked live: save and set the culture, restore it, the asset
    and its hash, a path placeholder, the launch flags, Connect only when not
    connected, and teleop for Autonomous mode."""
    assert SCRIPT.is_file(), "scripts/autodrive_gui.sh missing"
    syntax = subprocess.run(
        ["bash", "-n", str(SCRIPT)], capture_output=True, text=True
    )
    assert syntax.returncode == 0, syntax.stderr
    done, _ = _run(tmp_path, "ok", AUTODRIVE_ROS_DOMAIN_ID="66")
    out = done.stdout
    for needle in (
        "(Get-Culture).Name",
        "Set-Culture en-US",
        "Set-Culture pt-PT",
        ASSET,
        SHA256,
        "Expand-Archive",
        "<folder for the sim>",
        "-ip 127.0.0.1 -port 4567",
        "only if the window shows it is",
        "teleop_keyboard",
        "ROS_DOMAIN_ID=66",
    ):
        assert needle in out, needle
