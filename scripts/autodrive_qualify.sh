#!/usr/bin/env bash
# Run a live Porto case (SIM-3730 qualification, SIM-3750 fast lap; #44) from the host:
# recreate the single-use AutoDRIVE sim container pointed at `dev`, run the
# test in `dev`, stop the sim. Needs `dev` up (docker compose up -d dev).
#   scripts/autodrive_qualify.sh --domain 64 [--workdir /ws/.scratch/x]
#       [--case qualification|fast_lap|corner_sweep] [--sweep-dir D] [--dry-run]
# --case picks the test: qualification (default) is SIM-3730, fast_lap is
# SIM-3750, corner_sweep is SIM-3760 (needs --sweep-dir D, a sweep dir inside
# dev, passed as AUTODRIVE_SWEEP_DIR; any other case refuses it); anything
# else exits 2.
# --domain is the stack's ROS_DOMAIN_ID, 60-69 (never 42, the devkit default).
# --workdir is the checkout inside dev (default /ws). Artifacts land in
# <workdir>/.scratch-out/autodrive_qualify (qualification.json, porto_run.npz),
# or .scratch-out/autodrive_fast_lap (fast_lap.json, porto_run.npz), or
# .scratch-out/autodrive_corner_sweep (corner_sweep.json, channels.jsonl.gz).
# Exit: 0 pass, 1 test failed, 2 bad arguments, 4 dev is not running.
set -euo pipefail

domain=""
workdir=/ws
dry=0
case_name=qualification
sweep_dir=
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
*)
    echo "--case must be qualification, fast_lap or corner_sweep, not $case_name" >&2
    exit 2
    ;;
esac

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

if [ "$dry" -eq 1 ]; then
    printf '%s\n%s\n%s\n' "$compose" "$run" "$stop"
    exit 0
fi

trap 'docker stop "$sim" >/dev/null 2>&1' EXIT INT TERM
eval "$compose"
status=0
eval "$run" || status=1
exit "$status"
