#!/usr/bin/env bash
# Run one heavy command (a ready gate, a live qualification, a recording)
# alone on this machine. Every session, on the host or in any container, takes
# the same lock: the host directory /tmp/gr-roboracer-lane, which the compose
# services bind-mount at /lane. Two files at the same path on the host and in
# a container are two files; this is one.
#   scripts/heavy_lane.sh [--max-load N] [--wait SECONDS] -- <command> [args]
# Takes an exclusive flock on <lane>/heavy.lock, then waits for the 1-minute
# load average to drop below N (default 8) before running the command. The
# lock goes when the command's process does, so a crash cannot strand it.
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

# In a container the lane is /lane; on the host it is the directory behind it.
if [ -d /lane ]; then
    lane=/lane
else
    lane=${GR_LANE_DIR:-/tmp/gr-roboracer-lane}
    # Created here with world-write so a root container and the host user
    # can both open it; compose would otherwise create it root-owned.
    (umask 0 && mkdir -p "$lane") && chmod 1777 "$lane" 2>/dev/null || true
fi
if [ ! -d "$lane" ] || [ ! -w "$lane" ]; then
    echo "no writable lane at $lane: recreate the container so compose mounts it" >&2
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
# The child gets fd 9 too, which keeps the lock until the whole tree is gone.
"$@"
