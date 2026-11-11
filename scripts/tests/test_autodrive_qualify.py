"""Guards on `scripts/autodrive_qualify.sh`, the host wrapper of the live
Porto qualification run (SIM-3730, #44 part 10). The sim, compose and the run
itself are a live check; here a stub `docker` first on `PATH` logs its argv.
"""

from __future__ import annotations

import os
import stat
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "autodrive_qualify.sh"

DOCKER = """#!/bin/sh
echo "docker $*" >> "$STUB_LOG"
case "$1" in
  ps) [ -n "$STUB_PS" ] && echo "$STUB_PS"; exit 0 ;;
esac
exit "${STUB_EXIT:-0}"
"""


def _run(tmp_path: Path, name: str, *args: str, **extra: str):
    stub = tmp_path / "docker"
    stub.write_text(DOCKER)
    stub.chmod(stub.stat().st_mode | stat.S_IXUSR)
    log = tmp_path / f"{name}.log"
    env = {
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "STUB_LOG": str(log),
        "STUB_PS": "gr-roboracer-dev",
        **extra,
    }
    done = subprocess.run(
        ["bash", str(SCRIPT), *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    lines = log.read_text().splitlines() if log.exists() else []
    return done, lines


def test_check_2140_dry_run_prints_the_three_commands(tmp_path) -> None:
    """CHECK-2140: the script passes `bash -n`; `--dry-run` runs nothing but
    `docker ps`, and prints (1) the compose line that recreates only
    `autodrive-sim` with the bridge host set to `gr-roboracer-dev`, (2) the
    `docker exec` that runs SIM-3730 on the given domain in the given
    workdir, and (3) the stop of the sim container."""
    assert SCRIPT.is_file(), "scripts/autodrive_qualify.sh missing"
    syntax = subprocess.run(
        ["bash", "-n", str(SCRIPT)], capture_output=True, text=True
    )
    assert syntax.returncode == 0, syntax.stderr
    done, calls = _run(
        tmp_path,
        "dry",
        "--domain",
        "64",
        "--workdir",
        "/ws/.scratch/autodrive-p10",
        "--dry-run",
    )
    assert done.returncode == 0, done.stderr
    out = done.stdout
    for part in (
        "AUTODRIVE_BRIDGE_HOST=gr-roboracer-dev",
        "--profile autodrive up -d --no-deps --force-recreate autodrive-sim",
        "docker exec gr-roboracer-dev bash -lc",
        "cd /ws/.scratch/autodrive-p10",
        "ROS_DOMAIN_ID=64",
        "tests/autodrive/test_sim_3730_porto_qualification.py",
        "docker stop gr-roboracer-autodrive-sim",
    ):
        assert part in out, (part, out)
    assert all(c.startswith("docker ps") for c in calls), calls


def test_check_2150_bad_domain_and_missing_dev_are_refused(tmp_path) -> None:
    """CHECK-2150: no domain, 42, or a domain outside 60-69 gives exit 2; a
    dev container that is not running gives exit 4; neither starts compose."""
    assert SCRIPT.is_file(), "scripts/autodrive_qualify.sh missing"
    for args in ((), ("--domain", "42"), ("--domain", "59"), ("--domain", "x")):
        done, calls = _run(tmp_path, "bad", *args, "--dry-run")
        assert done.returncode == 2, (args, done.stderr)
        assert not any("compose" in c for c in calls), calls
    done, calls = _run(tmp_path, "novalue", "--domain")
    assert done.returncode == 2, done.stderr
    assert not any("compose" in c for c in calls), calls
    done, calls = _run(
        tmp_path, "nodev", "--domain", "64", "--dry-run", STUB_PS=""
    )
    assert done.returncode == 4, done.stderr
    assert not any("compose" in c for c in calls), calls


def test_check_2160_case_selects_the_fast_lap_test(tmp_path) -> None:
    """CHECK-2160: `--dry-run --case fast_lap` runs SIM-3750 into
    `autodrive_fast_lap`; the default stays SIM-3730; an unknown `--case`
    exits 2 naming the accepted values and runs nothing."""
    done, calls = _run(
        tmp_path, "fast", "--domain", "64", "--case", "fast_lap", "--dry-run"
    )
    assert done.returncode == 0, done.stderr
    assert "test_sim_3750_porto_fast_mintime_lap.py" in done.stdout
    assert "autodrive_fast_lap" in done.stdout
    assert "test_sim_3730" not in done.stdout
    done, _ = _run(tmp_path, "dflt", "--domain", "64", "--dry-run")
    assert done.returncode == 0, done.stderr
    assert "test_sim_3730_porto_qualification.py" in done.stdout
    assert "autodrive_qualify" in done.stdout
    assert "autodrive_fast_lap" not in done.stdout
    done, calls = _run(tmp_path, "bad", "--domain", "64", "--case", "nope")
    assert done.returncode == 2
    assert "qualification" in done.stderr and "fast_lap" in done.stderr
    assert not calls


def test_check_2695_corner_sweep_takes_a_sweep_dir(tmp_path) -> None:
    """CHECK-2695: `--case corner_sweep --sweep-dir D --dry-run` prints the
    SIM-3760 test with `AUTODRIVE_SWEEP_DIR=D` into `autodrive_corner_sweep`;
    `corner_sweep` without `--sweep-dir` exits 2, and so does `--sweep-dir`
    with any other case; the two existing cases print without the variable;
    the unknown-case message names all three cases."""
    done, calls = _run(
        tmp_path,
        "sweep",
        "--domain",
        "64",
        "--case",
        "corner_sweep",
        "--sweep-dir",
        "/ws/sweep/a9",
        "--dry-run",
    )
    assert done.returncode == 0, done.stderr
    assert "test_sim_3760_porto_corner_sweep.py" in done.stdout
    assert "autodrive_corner_sweep" in done.stdout
    assert "AUTODRIVE_SWEEP_DIR=/ws/sweep/a9" in done.stdout
    assert "test_sim_3730" not in done.stdout
    done, calls = _run(
        tmp_path, "nodir", "--domain", "64", "--case", "corner_sweep"
    )
    assert done.returncode == 2
    assert "--sweep-dir" in done.stderr
    assert not calls
    for case in ("qualification", "fast_lap"):
        done, calls = _run(
            tmp_path,
            "other",
            "--domain",
            "64",
            "--case",
            case,
            "--sweep-dir",
            "/x",
        )
        assert done.returncode == 2, case
        assert not calls
    for case in ("qualification", "fast_lap"):
        done, _ = _run(
            tmp_path, "keep", "--domain", "64", "--case", case, "--dry-run"
        )
        assert done.returncode == 0, done.stderr
        assert "AUTODRIVE_SWEEP_DIR" not in done.stdout
    done, _ = _run(tmp_path, "bad", "--domain", "64", "--case", "nope")
    assert done.returncode == 2
    for name in ("qualification", "fast_lap", "corner_sweep"):
        assert name in done.stderr


# --- CHECK-2480..2482: `--sim windows` and the estimator cases (1V-fix-b) ---

ROUTE = """#!/bin/sh
echo "route $*" >> "$STUB_LOG"
exit 0
"""

DOCKER_WINDOWS = """#!/bin/sh
echo "docker $*" >> "$STUB_LOG"
case "$1" in
  ps)
    case "$*" in
      *autodrive-sim*) [ -n "$STUB_PS_SIM" ] && echo "$STUB_PS_SIM" ;;
      *) [ -n "$STUB_PS" ] && echo "$STUB_PS" ;;
    esac
    exit 0 ;;
  exec)
    case "$*" in
      *"/proc/net/tcp"*) [ -n "$STUB_LISTEN" ] && exit 0
                   exit 1 ;;
    esac
    exit "${STUB_EXIT:-0}" ;;
esac
exit 0
"""

POWERSHELL = """#!/bin/sh
echo "powershell.exe $*" >> "$STUB_LOG"
n=$(cat "$STUB_COUNT" 2>/dev/null || echo 0)
n=$((n + 1))
echo "$n" > "$STUB_COUNT"
case "${STUB_PS_MODE:-ok}" in
  vsock_once)
    if [ "$n" -eq 1 ]; then
      echo "UtilAcceptVsock:273: accept4 failed 110"
      exit 0
    fi ;;
  vsock_always)
    echo "UtilAcceptVsock:273: accept4 failed 110"
    exit 0 ;;
  fail) echo "boom" >&2; exit 1 ;;
esac
exit 0
"""


def _tree(tmp_path: Path) -> Path:
    """A copy of the script in a tmp tree beside a fake route script and an
    empty helper: the copy's `root` is the tree, so the real route script
    (docker, sockets) is never started."""
    root = tmp_path / "tree"
    (root / "scripts" / "windows").mkdir(parents=True, exist_ok=True)
    script = root / "scripts" / "autodrive_qualify.sh"
    script.write_text(SCRIPT.read_text())
    route = root / "scripts" / "autodrive_route.sh"
    route.write_text(ROUTE)
    route.chmod(0o755)
    (root / "scripts" / "windows" / "autodrive.ps1").write_text("")
    return script


def _run_windows(
    tmp_path: Path, name: str, *args: str, powershell: bool = True, **extra
):
    """Run the tmp copy with fake docker (and, by default, a fake
    powershell.exe). Without `powershell` the PATH carries no powershell.exe
    at all (WSL puts the real one on it)."""
    script = _tree(tmp_path)
    bin_dir = tmp_path / f"bin-{name}"
    bin_dir.mkdir()
    docker = bin_dir / "docker"
    docker.write_text(DOCKER_WINDOWS)
    docker.chmod(0o755)
    if powershell:
        ps = bin_dir / "powershell.exe"
        ps.write_text(POWERSHELL)
        ps.chmod(0o755)
    path = [
        d
        for d in os.environ["PATH"].split(":")
        if not (Path(d) / "powershell.exe").exists()
    ]
    log = tmp_path / f"{name}.log"
    env = {
        "PATH": ":".join([str(bin_dir), *path]),
        "STUB_LOG": str(log),
        "STUB_COUNT": str(tmp_path / f"{name}.count"),
        "STUB_PS": "gr-roboracer-dev",
        "STUB_LISTEN": "1",
        "AUTODRIVE_SIM_EXE": r"C:\dev\Autodrive\sim\AutoDRIVE Simulator.exe",
        "AUTODRIVE_WSL_IP": "172.20.0.9",
        **extra,
    }
    done = subprocess.run(
        ["bash", str(script), *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    lines = log.read_text().splitlines() if log.exists() else []
    return done, lines


def test_check_2480_sim_windows_dry_run_prints_the_route_not_the_container(
    tmp_path,
) -> None:
    """CHECK-2480: `--sim container` is the default (CHECK-2140 stays as it
    was); `--sim windows --dry-run` prints, in order, the route start, the
    test in the background in `dev`, the poll for a listener on 4567 in
    `dev`, the helper through `powershell.exe -NoProfile -ExecutionPolicy
    Bypass` with the sim exe and the WSL IP, and the route stop, and NONE of
    the compose recreate or the stop of the sim container; an unknown `--sim`
    exits 2. (The unknown-`--sim` half is green before the change, because
    the script already exits 2 on any unknown argument.)"""
    done, calls = _run_windows(
        tmp_path,
        "dry",
        "--domain",
        "64",
        "--case",
        "fast_lap",
        "--sim",
        "windows",
        "--dry-run",
    )
    assert done.returncode == 0, done.stderr
    out = done.stdout
    for part in (
        "autodrive_route.sh",
        "docker exec gr-roboracer-dev bash -lc",
        "tests/autodrive/test_sim_3750_porto_fast_mintime_lap.py",
        "/proc/net/tcp",
        ":11D7",
        "powershell.exe -NoProfile -ExecutionPolicy Bypass -Command",
        "autodrive.ps1",
        "autodrive -SimPath",
        "AutoDRIVE Simulator.exe",
        "-Ip 172.20.0.9",
    ):
        assert part in out, (part, out)
    for part in ("compose", "--force-recreate", "docker stop"):
        assert part not in out, (part, out)
    order = [
        out.index(p)
        for p in (
            "autodrive_route.sh",
            "test_sim_3750",
            "/proc/net/tcp",
            "autodrive -SimPath",
        )
    ]
    assert order == sorted(order), out
    assert not any("compose" in c or "docker stop" in c for c in calls), calls
    done, calls = _run_windows(
        tmp_path, "bad", "--domain", "64", "--sim", "nope", "--dry-run"
    )
    assert done.returncode == 2, done.stderr
    assert "--sim" in done.stderr or "unknown argument" in done.stderr
    done, _ = _run_windows(
        tmp_path,
        "dflt",
        "--domain",
        "64",
        "--dry-run",
        AUTODRIVE_SIM_EXE="",
    )
    assert done.returncode == 0, done.stderr  # container: no exe needed
    assert "--force-recreate autodrive-sim" in done.stdout
    assert "powershell.exe" not in done.stdout


def test_check_2481_sim_windows_refusals_and_the_vsock_retry(tmp_path) -> None:
    """CHECK-2481: `--sim windows` refuses a running sim container (exit 5)
    and a missing `powershell.exe` (exit 6), and a missing
    AUTODRIVE_SIM_EXE (exit 2), each before it starts the route; a run starts
    the route, runs the helper once and stops the route; a helper reply
    carrying `UtilAcceptVsock ... accept4 failed 110` is retried once and
    then succeeds; the same reply twice (or any helper failure) fails loudly
    (exit 7) and still stops the route."""
    args = ("--domain", "64", "--case", "fast_lap", "--sim", "windows")
    done, calls = _run_windows(
        tmp_path, "run", *args, STUB_PS_SIM="gr-roboracer-autodrive-sim"
    )
    assert done.returncode == 5, (done.returncode, done.stderr)
    assert not any(c.startswith("route") for c in calls), calls
    done, calls = _run_windows(tmp_path, "nops", *args, powershell=False)
    assert done.returncode == 6, (done.returncode, done.stderr)
    assert not any(c.startswith("route") for c in calls), calls
    done, calls = _run_windows(tmp_path, "noexe", *args, AUTODRIVE_SIM_EXE="")
    assert done.returncode == 2, (done.returncode, done.stderr)
    assert "AUTODRIVE_SIM_EXE" in done.stderr
    assert not any(c.startswith("route") for c in calls), calls

    done, calls = _run_windows(tmp_path, "ok", *args)
    assert done.returncode == 0, (done.stdout, done.stderr)
    routes = [c for c in calls if c.startswith("route")]
    assert routes[0] == "route " and routes[-1] == "route stop", routes
    assert sum(c.startswith("powershell.exe") for c in calls) == 1, calls

    done, calls = _run_windows(
        tmp_path, "once", *args, STUB_PS_MODE="vsock_once"
    )
    assert done.returncode == 0, (done.stdout, done.stderr)
    assert sum(c.startswith("powershell.exe") for c in calls) == 2, calls

    done, calls = _run_windows(
        tmp_path, "twice", *args, STUB_PS_MODE="vsock_always"
    )
    assert done.returncode == 7, (done.returncode, done.stderr)
    assert sum(c.startswith("powershell.exe") for c in calls) == 2, calls
    assert [c for c in calls if c.startswith("route")][-1] == "route stop"
    done, calls = _run_windows(tmp_path, "fail", *args, STUB_PS_MODE="fail")
    assert done.returncode == 7, (done.returncode, done.stderr)
    assert [c for c in calls if c.startswith("route")][-1] == "route stop"

    done, calls = _run_windows(tmp_path, "red", *args, STUB_EXIT="1")
    assert done.returncode == 1, (done.returncode, done.stderr)  # test failed


def test_check_2482_new_cases_select_their_tests(tmp_path) -> None:
    """CHECK-2482: `--case survey|mola_map|fast_lap_amcl|fast_lap_mola`
    select SIM-3770 / 3771 / 3780 / 3781 into `autodrive_<case>`, under both
    `--sim container` and `--sim windows`; the unknown-case message names all
    seven cases. (`--case survey` exits 2 before the change.)"""
    for case, test, out_name in (
        ("survey", "test_sim_3770_porto_survey_map.py", "autodrive_survey"),
        ("mola_map", "test_sim_3771_porto_mola_map.py", "autodrive_mola_map"),
        (
            "fast_lap_amcl",
            "test_sim_3780_porto_fast_lap_amcl.py",
            "autodrive_fast_lap_amcl",
        ),
        (
            "fast_lap_mola",
            "test_sim_3781_porto_fast_lap_mola.py",
            "autodrive_fast_lap_mola",
        ),
    ):
        for sim in ("container", "windows"):
            done, _ = _run_windows(
                tmp_path,
                f"{case}-{sim}",
                "--domain",
                "64",
                "--case",
                case,
                "--sim",
                sim,
                "--dry-run",
            )
            assert done.returncode == 0, (case, sim, done.stderr)
            assert f"tests/autodrive/{test}" in done.stdout, done.stdout
            assert f".scratch-out/{out_name} python3" in done.stdout
            assert "test_sim_3730" not in done.stdout
    done, _ = _run_windows(
        tmp_path, "unk", "--domain", "64", "--case", "nope", "--dry-run"
    )
    assert done.returncode == 2
    for case in (
        "qualification",
        "fast_lap",
        "corner_sweep",
        "survey",
        "mola_map",
        "fast_lap_amcl",
        "fast_lap_mola",
    ):
        assert case in done.stderr, done.stderr
