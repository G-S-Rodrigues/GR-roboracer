"""Guards on `scripts/autodrive_gui.sh`, the devkit half of the AutoDRIVE
window route (docs/agents/running-wsl.md, "AutoDRIVE GUI").

The window itself is a manual check; what is tested here is that the script
cannot touch the wrong ROS domain or the shared compose project, and that the
command it prints for Windows is the one the owner ran.
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
EXE = r"C:\dev\Autodrive\autodrive_simulator\AutoDRIVE Simulator.exe"


def _stub_docker(tmp_path: Path) -> tuple[dict, Path]:
    log = tmp_path / "argv.log"
    stub = tmp_path / "docker"
    stub.write_text(f'#!/bin/sh\necho "$@" >> "{log}"\n')
    stub.chmod(stub.stat().st_mode | stat.S_IXUSR)
    env = {"PATH": f"{tmp_path}:{os.environ['PATH']}"}
    return env, log


def test_check_1670_script_refuses_unset_or_shared_domain(tmp_path) -> None:
    """CHECK-1670: the script needs `AUTODRIVE_ROS_DOMAIN_ID`, refuses it
    unset or 42 (the headless stack's domain) before any docker call, and its
    text starts only `autodrive-devkit`, with `--no-deps` and the loopback
    publish, and never `up`, `down` or `--remove-orphans`."""
    assert SCRIPT.is_file(), "scripts/autodrive_gui.sh missing"
    for domain in (None, "42"):
        env, log = _stub_docker(tmp_path)
        if domain is not None:
            env["AUTODRIVE_ROS_DOMAIN_ID"] = domain
        done = subprocess.run(
            ["bash", str(SCRIPT)], env=env, capture_output=True, text=True
        )
        assert done.returncode != 0, f"domain {domain!r} was accepted"
        assert not log.exists(), f"docker called for domain {domain!r}"
    text = SCRIPT.read_text()
    for needle in (
        "run",
        "--no-deps",
        "-p 127.0.0.1:4567:4567",
        "autodrive-devkit",
    ):
        assert needle in text, needle
    for banned in (" up ", " down ", "--remove-orphans"):
        assert banned not in text, banned


def test_check_1680_printed_windows_command_and_syntax() -> None:
    """CHECK-1680: the script passes `bash -n`, and names the release asset,
    its sha256, the exe path and `-ip 127.0.0.1 -port 4567`."""
    assert SCRIPT.is_file(), "scripts/autodrive_gui.sh missing"
    done = subprocess.run(
        ["bash", "-n", str(SCRIPT)], capture_output=True, text=True
    )
    assert done.returncode == 0, done.stderr
    text = SCRIPT.read_text()
    for needle in (ASSET, SHA256, EXE, "-ip 127.0.0.1 -port 4567"):
        assert needle in text, needle
