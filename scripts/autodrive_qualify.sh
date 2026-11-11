#!/usr/bin/env bash
# Run a live Porto case (SIM-3730 qualification, SIM-3750 fast lap; #44) from the host:
# recreate the single-use AutoDRIVE sim container pointed at `dev`, run the
# test in `dev`, stop the sim. Needs `dev` up (docker compose up -d dev).
#   scripts/autodrive_qualify.sh --domain 64 [--workdir /ws/.scratch/x]
#       [--case qualification|fast_lap|corner_sweep|survey|mola_map|
#               fast_lap_amcl|fast_lap_mola] [--sweep-dir D]
#       [--sim container|windows] [--dry-run]
# --case picks the test: qualification (default) is SIM-3730, fast_lap is
# SIM-3750, corner_sweep is SIM-3760 (needs --sweep-dir D, a sweep dir inside
# dev, passed as AUTODRIVE_SWEEP_DIR; any other case refuses it), survey is
# SIM-3770 (slam_toolbox map), mola_map SIM-3771 (MOLA map), fast_lap_amcl
# SIM-3780 and fast_lap_mola SIM-3781 (the fast lap on an estimator); anything
# else exits 2.
# --sim container (default) recreates the headless sim container pointed at
# dev. --sim windows drives the GUI build on the Windows host instead: it
# starts scripts/autodrive_route.sh, runs the test in dev in the background,
# waits for the server's listener on 4567 in dev, then runs
# scripts/windows/autodrive.ps1 through powershell.exe (AUTODRIVE_SIM_EXE =
# the Windows path of AutoDRIVE Simulator.exe, required; AUTODRIVE_WSL_IP
# overrides the address) and waits for the test; the route stops on exit.
# The Windows culture must be en-US (gotcha 38). The Windows route does not
# stop the sim window: close it yourself.
# --domain is the stack's ROS_DOMAIN_ID, 60-69 (never 42, the devkit default).
# --workdir is the checkout inside dev (default /ws). Artifacts land in
# <workdir>/.scratch-out/autodrive_qualify (qualification.json, porto_run.npz),
# or .scratch-out/autodrive_fast_lap (fast_lap.json, porto_run.npz), or
# .scratch-out/autodrive_corner_sweep (corner_sweep.json, channels.jsonl.gz).
# Exit: 0 pass, 1 test failed, 2 bad arguments, 4 dev is not running,
# 5 (windows) the headless sim container is running, 6 (windows) no
# powershell.exe, 7 (windows) the helper failed.
set -euo pipefail

domain=""
workdir=/ws
dry=0
case_name=qualification
sweep_dir=
sim_mode=container
while [ $# -gt 0 ]; do
    case "$1" in
    --domain | --workdir)
        if [ $# -lt 2 ]; then
            echo "$1 needs a value" >&2
            exit 2
        fi
        if [ "$1" = "--domain" ]; then domain="$2"; else workdir="$2"; fi
        shift 2
        ;;
    --case)
        if [ $# -lt 2 ]; then
            echo "--case needs a value" >&2
            exit 2
        fi
        case_name="$2"
        shift 2
        ;;
    --sim)
        if [ $# -lt 2 ]; then
            echo "--sim needs a value" >&2
            exit 2
        fi
        sim_mode="$2"
        shift 2
        ;;
    --sweep-dir)
        if [ $# -lt 2 ]; then
            echo "--sweep-dir needs a value" >&2
            exit 2
        fi
        sweep_dir="$2"
        shift 2
        ;;
    --dry-run) dry=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
if ! [[ "$domain" =~ ^[0-9]+$ ]] || [ "$((10#$domain))" -lt 60 ] ||
    [ "$((10#$domain))" -gt 69 ]; then
    echo "--domain must be an integer 60-69 (not 42)" >&2
    exit 2
fi

case "$case_name" in
qualification)
    test_file=test_sim_3730_porto_qualification.py
    out_name=autodrive_qualify
    ;;
fast_lap)
    test_file=test_sim_3750_porto_fast_mintime_lap.py
    out_name=autodrive_fast_lap
    ;;
corner_sweep)
    test_file=test_sim_3760_porto_corner_sweep.py
    out_name=autodrive_corner_sweep
    ;;
survey)
    test_file=test_sim_3770_porto_survey_map.py
    out_name=autodrive_survey
    ;;
mola_map)
    test_file=test_sim_3771_porto_mola_map.py
    out_name=autodrive_mola_map
    ;;
fast_lap_amcl)
    test_file=test_sim_3780_porto_fast_lap_amcl.py
    out_name=autodrive_fast_lap_amcl
    ;;
fast_lap_mola)
    test_file=test_sim_3781_porto_fast_lap_mola.py
    out_name=autodrive_fast_lap_mola
    ;;
*)
    echo "--case must be qualification, fast_lap, corner_sweep, survey, mola_map, fast_lap_amcl or fast_lap_mola, not $case_name" >&2
    exit 2
    ;;
esac

case "$sim_mode" in
container | windows) ;;
*)
    echo "--sim must be container or windows, not $sim_mode" >&2
    exit 2
    ;;
esac
sim_exe="${AUTODRIVE_SIM_EXE:-}"
if [ "$sim_mode" = windows ] && [ -z "$sim_exe" ]; then
    echo "--sim windows needs AUTODRIVE_SIM_EXE (the Windows path of AutoDRIVE Simulator.exe)" >&2
    exit 2
fi

if [ "$case_name" = corner_sweep ] && [ -z "$sweep_dir" ]; then
    echo "--case corner_sweep needs --sweep-dir D" >&2
    exit 2
fi
if [ "$case_name" != corner_sweep ] && [ -n "$sweep_dir" ]; then
    echo "--sweep-dir is only for --case corner_sweep" >&2
    exit 2
fi
sweep_env=""
if [ -n "$sweep_dir" ]; then sweep_env="AUTODRIVE_SWEEP_DIR=$sweep_dir "; fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dev=gr-roboracer-dev
sim=gr-roboracer-autodrive-sim
if [ -z "$(docker ps -q --filter "name=^${dev}\$" 2>/dev/null)" ] &&
    [ -z "$(docker ps --filter "name=^${dev}\$" --format '{{.Names}}' 2>/dev/null)" ]; then
    echo "container $dev is not running: docker compose -f docker/docker-compose.yaml up -d dev" >&2
    exit 4
fi

compose="AUTODRIVE_BRIDGE_HOST=$dev docker compose -f $root/docker/docker-compose.yaml --profile autodrive up -d --no-deps --force-recreate autodrive-sim"
inner="cd $workdir && source setup.sh && source install/setup.bash && flock /tmp/gr-roboracer-heavy-lane.lock env ROS_DOMAIN_ID=$domain ${sweep_env}AUTODRIVE_OUT=$workdir/.scratch-out/$out_name python3 -m pytest tests/autodrive/$test_file -s"
run="docker exec $dev bash -lc '$inner'"
stop="docker stop $sim"

if [ "$sim_mode" = windows ]; then
    # The Windows sim window replaces the container: refuse a running one
    # (two sims on 4567), and a host with no powershell.exe.
    if [ -n "$(docker ps -q --filter "name=^${sim}\$" 2>/dev/null)" ] ||
        [ -n "$(docker ps --filter "name=^${sim}\$" --format '{{.Names}}' 2>/dev/null)" ]; then
        echo "container $sim is running: docker stop $sim first (one sim on 4567)" >&2
        exit 5
    fi
    if [ "$dry" -eq 0 ] && ! command -v powershell.exe >/dev/null 2>&1; then
        echo "powershell.exe not found: --sim windows needs WSL interop" >&2
        exit 6
    fi
    wsl_ip="${AUTODRIVE_WSL_IP:-$(hostname -I 2>/dev/null | awk '{print $1}')}"
    route="$root/scripts/autodrive_route.sh"
    ps_file="$(wslpath -w "$root/scripts/windows/autodrive.ps1" 2>/dev/null ||
        echo "$root/scripts/windows/autodrive.ps1")"
    helper="powershell.exe -NoProfile -ExecutionPolicy Bypass -Command \". '$ps_file'; autodrive -SimPath '$sim_exe' -Ip $wsl_ip\""
    # dev has no ss: read the listening sockets (state 0A) of port 4567
    # (hex 11D7) from /proc/net/tcp and tcp6.
    poll="docker exec $dev grep -qiE ':11D7 [0-9A-F:]+ 0A ' /proc/net/tcp /proc/net/tcp6"
    if [ "$dry" -eq 1 ]; then
        printf '%s\n%s &\n%s\n%s\n%s\n' "$route" "$run" "$poll" "$helper" "$route stop"
        exit 0
    fi

    pytest_pid=""
    cleanup() {
        if [ -n "$pytest_pid" ] && kill -0 "$pytest_pid" 2>/dev/null; then
            kill "$pytest_pid" 2>/dev/null || true
            docker exec "$dev" pkill -f "tests/autodrive/$test_file" \
                >/dev/null 2>&1 || true
        fi
        "$route" stop >/dev/null 2>&1 || true
    }
    trap cleanup EXIT INT TERM
    "$route"
    eval "$run" &
    pytest_pid=$!
    # The sim connects once (gotcha 30): start the helper only when the
    # server in dev listens.
    listening=0
    for _ in $(seq 1 180); do
        if docker exec "$dev" grep -qiE ':11D7 [0-9A-F:]+ 0A ' \
            /proc/net/tcp /proc/net/tcp6 2>/dev/null; then
            listening=1
            break
        fi
        kill -0 "$pytest_pid" 2>/dev/null || break
        sleep 1
    done
    if [ "$listening" -eq 0 ]; then
        echo "no listener on 4567 in $dev within 180 s (or the test ended)" >&2
        exit 7
    fi
    # Interop works from a Windows directory; an empty reply carrying
    # `UtilAcceptVsock ... accept4 failed 110` is a failed call, never
    # success: retry once, then fail loudly.
    helper_ok=0
    for attempt in 1 2; do
        rc=0
        reply="$( (cd /mnt/c 2>/dev/null || true; eval "$helper") 2>&1)" || rc=$?
        if [ "$rc" -eq 0 ] && ! grep -q 'UtilAcceptVsock' <<<"$reply"; then
            helper_ok=1
            break
        fi
        echo "helper attempt $attempt failed (exit $rc): $reply" >&2
    done
    if [ "$helper_ok" -eq 0 ]; then
        echo "the Windows helper failed twice; see above" >&2
        exit 7
    fi
    status=0
    wait "$pytest_pid" || status=1
    pytest_pid=""
    exit "$status"
fi

if [ "$dry" -eq 1 ]; then
    printf '%s\n%s\n%s\n' "$compose" "$run" "$stop"
    exit 0
fi

trap 'docker stop "$sim" >/dev/null 2>&1' EXIT INT TERM
eval "$compose"
status=0
eval "$run" || status=1
exit "$status"
