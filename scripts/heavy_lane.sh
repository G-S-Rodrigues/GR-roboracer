#!/usr/bin/env bash
# Run one heavy command (a ready gate, a live qualification, a recording)
# alone on this machine. Every session, on the host or in any container, takes
# the same lock: the host directory /tmp/gr-roboracer-lane, which the compose
# services bind-mount at /lane. Two files at the same path on the host and in
# a container are two files; this is one.
#   scripts/heavy_lane.sh [--max-load N] [--wait SECONDS] -- <command> [args]
# Takes an exclusive flock on <lane>/heavy.lock, then waits for the 1-minute
# load average to drop below N (default 8) before running the command. The
# lock is an open fd 9 that the wrapper and a bash child inherit, so it goes
# when every holder of fd 9 is gone: a shell orphan of the command keeps it,
# a Python child (colcon, pytest, ros2 launch) does not.
# --wait gives up after SECONDS (default: wait forever).
# Exit: the command's own, 2 bad arguments, 3 gave up waiting, 4 no lane.
set -euo pipefail

max_load=8
wait_s=""
while [ $# -gt 0 ]; do
    case "$1" in
    --max-load | --wait)
        if [ $# -lt 2 ] || ! [[ "$2" =~ ^[0-9]+$ ]]; then
            echo "$1 needs a whole number" >&2
            exit 2
        fi
        if [ "$1" = "--max-load" ]; then max_load="$2"; else wait_s="$2"; fi
        shift 2
        ;;
    --) shift; break ;;
    *) echo "unknown argument: $1 (the command goes after --)" >&2; exit 2 ;;
    esac
done
if [ $# -eq 0 ]; then
    echo "no command given after --" >&2
    exit 2
fi

# The command may itself call this script (check.sh inside a wrapped run).
# A second flock on the same file would wait on its own parent forever.
if [ -n "${HEAVY_LANE_HELD:-}" ]; then
    exec "$@"
fi

# GR_LANE_DIR wins; then the /lane mount; a container without the mount stops
# (its own /tmp would give a private lock that excludes nobody); only the host
# uses the directory behind the mount. GR_LANE_MOUNT and GR_CONTAINER_MARKER
# exist for the tests.
lane_mount=${GR_LANE_MOUNT:-/lane}
if [ -n "${GR_LANE_DIR:-}" ]; then
    lane=$GR_LANE_DIR
elif [ -d "$lane_mount" ]; then
    lane=$lane_mount
    # Compose creates a missing source root:root 0755; open it so the host user
    # can take the lock too (effective only as root, which dev is).
    chmod 1777 "$lane" 2>/dev/null || true
elif [ -e "${GR_CONTAINER_MARKER:-/.dockerenv}" ]; then
    echo "no $lane_mount in this container: it was made before compose mounted the lane; recreate it (a private lock would exclude nobody)" >&2
    exit 4
else
    lane=/tmp/gr-roboracer-lane
    # Created here with world-write so a root container and the host user
    # can both open it; compose would otherwise create it root-owned.
    (umask 0 && mkdir -p "$lane") && chmod 1777 "$lane" 2>/dev/null || true
fi
if [ ! -d "$lane" ] || [ ! -w "$lane" ]; then
    echo "no writable lane at $lane (owner $(stat -c %U "$lane" 2>/dev/null || echo none), mode $(stat -c %a "$lane" 2>/dev/null || echo none)): if root owns it, run: docker exec gr-roboracer-dev chmod 1777 /lane (or sudo chmod 1777 $lane)" >&2
    exit 4
fi

lock="$lane/heavy.lock"
(umask 0 && : >>"$lock") 2>/dev/null || true
exec 9<"$lock"
flock_args=(-x)
[ -n "$wait_s" ] && flock_args+=(-w "$wait_s")
if ! flock "${flock_args[@]}" 9; then
    echo "gave up after ${wait_s}s: $(cat "$lane/owner" 2>/dev/null || echo 'owner unknown') holds the lane" >&2
    exit 3
fi

# Only the holder writes this, so it names the run that is on the machine.
echo "$(date -u +%FT%TZ) pid $$ $* " >"$lane/owner" 2>/dev/null || true

# Holding the lock does not make the machine quiet: wait out other load too.
while [ "$(cut -d. -f1 /proc/loadavg)" -ge "$max_load" ]; do
    sleep 5
done

export HEAVY_LANE_HELD=1
# Not exec: bash must stay alive so fd 9, and with it the lock, stays open.
# A bash child inherits fd 9 and keeps the lock past the wrapper's death; a
# Python-spawned child does not, so leftover ROS nodes hold nothing.
"$@"
