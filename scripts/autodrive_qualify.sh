#!/usr/bin/env bash
# Run a live Porto case from the host: recreate the single-use AutoDRIVE sim
# container pointed at `dev`, run the test in `dev`, stop the sim. Needs `dev`
# up (docker compose up -d dev).
#   scripts/autodrive_qualify.sh --domain 64 [--workdir /ws/.scratch/x]
#       [--case qualification|mapping] [--reference-map P] [--dry-run]
# --domain is the stack's ROS_DOMAIN_ID, 60-69 (never 42, the devkit default).
# --workdir is the checkout inside dev (default /ws).
# --case qualification (default) runs SIM-3730 (#44 part 10); artifacts land in
# <workdir>/.scratch-out/autodrive_qualify (qualification.json, porto_run.npz).
# --case mapping runs SIM-3740 (part 11a: follow_the_gap + the survey mapper);
# artifacts land in <workdir>/.scratch-out/autodrive_mapping (Porto.png/yaml,
# mapping.json, porto_map_run.npz).
# --reference-map P (a path inside dev; mapping only) is a second run's check
# against the first run's Porto.yaml: it is passed into the docker exec line
# as AUTODRIVE_REFERENCE_MAP, because the host environment does not reach it.
# Exit: 0 pass, 1 test failed, 2 bad arguments, 4 dev is not running.
set -euo pipefail

domain=""
workdir=/ws
dry=0
case_name=qualification
reference_map=""
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
    --case | --reference-map)
        if [ $# -lt 2 ]; then
            echo "$1 needs a value" >&2
            exit 2
        fi
        if [ "$1" = "--case" ]; then case_name="$2"; else reference_map="$2"; fi
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
    test_file=tests/autodrive/test_sim_3730_porto_qualification.py
    out_name=autodrive_qualify
    ;;
mapping)
    test_file=tests/autodrive/test_sim_3740_porto_mapping.py
    out_name=autodrive_mapping
    ;;
*) echo "--case must be qualification or mapping, not '$case_name'" >&2; exit 2 ;;
esac
if [ -n "$reference_map" ] && [ "$case_name" != mapping ]; then
    echo "--reference-map is valid only with --case mapping" >&2
    exit 2
fi
reference_env=""
if [ -n "$reference_map" ]; then
    reference_env="AUTODRIVE_REFERENCE_MAP=$reference_map "
fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dev=gr-roboracer-dev
sim=gr-roboracer-autodrive-sim
if [ -z "$(docker ps -q --filter "name=^${dev}\$" 2>/dev/null)" ] &&
    [ -z "$(docker ps --filter "name=^${dev}\$" --format '{{.Names}}' 2>/dev/null)" ]; then
    echo "container $dev is not running: docker compose -f docker/docker-compose.yaml up -d dev" >&2
    exit 4
fi

compose="AUTODRIVE_BRIDGE_HOST=$dev docker compose -f $root/docker/docker-compose.yaml --profile autodrive up -d --no-deps --force-recreate autodrive-sim"
inner="cd $workdir && source setup.sh && source install/setup.bash && flock /tmp/gr-roboracer-heavy-lane.lock env ROS_DOMAIN_ID=$domain AUTODRIVE_OUT=$workdir/.scratch-out/$out_name ${reference_env}python3 -m pytest $test_file -s"
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
