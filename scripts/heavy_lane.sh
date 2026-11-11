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
# Exit: the command's own, 2 bad arguments, 3 gave up waiting, 4 no lane, a
# deleted /lane, or a lane whose heavy.lock cannot be opened.
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
    # A host directory removed under a running container (systemd-tmpfiles'
    # 30-day /tmp clean) leaves /lane openable with link count 0; the host
    # would then make a new lane and the two sides would hold two locks.
    if [ "$(stat -c %h "$lane" 2>/dev/null || echo 1)" = 0 ]; then
        echo "$lane_mount is a deleted directory: the host lane /tmp/gr-roboracer-lane was removed (for example by systemd-tmpfiles' 30-day /tmp clean) while this container ran; a lock here would exclude nobody. Re-make the host dir and recreate dev, or exclude it with a tmpfiles.d 'x /tmp/gr-roboracer-lane' line" >&2
        exit 4
    fi
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
lock="$lane/heavy.lock"
# An unwritable lane (compose made it root 0755) still works when heavy.lock
# is readable: flock needs a read-only fd, not a writable directory.
[ -e "$lock" ] || (umask 0 && : >>"$lock") 2>/dev/null || true
# Whoever owns the lock may open it to every user (effective for its owner or
# root); the host and a root container then share one file.
chmod 0666 "$lock" 2>/dev/null || true
if [ ! -d "$lane" ] || [ ! -r "$lock" ]; then
    echo "no usable lane at $lane (owner $(stat -c %U "$lane" 2>/dev/null || echo none), mode $(stat -c %a "$lane" 2>/dev/null || echo none)): heavy.lock cannot be opened; if root owns the lane, run: docker exec gr-roboracer-dev chmod 1777 /lane (or sudo chmod 1777 $lane)" >&2
    exit 4
fi
exec 9<"$lock"
flock_args=(-x)
[ -n "$wait_s" ] && flock_args+=(-w "$wait_s")
if ! flock "${flock_args[@]}" 9; then
    echo "gave up after ${wait_s}s: $(cat "$lane/owner" 2>/dev/null || echo 'owner unknown') holds the lane" >&2
    exit 3
fi

# Only the holder writes this, so it names the run that is on the machine.
stamped=yes
stamp="$(date -u +%FT%TZ) pid $$ $* "
# An existing owner is written in place with no O_CREAT: in a sticky lane,
# fs.protected_regular refuses an O_CREAT open (root included) of a file that
# another user owns, which would leave a stale stamp naming the wrong holder.
if [ -e "$lane/owner" ]; then
    printf '%s\n' "$stamp" | dd of="$lane/owner" conv=nocreat status=none \
        2>/dev/null || stamped=no
else
    (umask 0 && printf '%s\n' "$stamp" >"$lane/owner") 2>/dev/null || stamped=no
fi
chmod 0666 "$lane/owner" 2>/dev/null || true
if [ "${stamped:-}" = no ]; then
    echo "lane taken; cannot write $lane/owner ($(stat -c '%U %a' "$lane/owner" 2>/dev/null || echo none)): waiters will see \"owner unknown\"; one wrapper run in dev opens the lane" >&2
fi

# Holding the lock does not make the machine quiet: wait out other load too.
while [ "$(cut -d. -f1 /proc/loadavg)" -ge "$max_load" ]; do
    sleep 5
done

export HEAVY_LANE_HELD=1
# Not exec: bash must stay alive so fd 9, and with it the lock, stays open.
# A bash child inherits fd 9 and keeps the lock past the wrapper's death; a
# Python-spawned child does not, so leftover ROS nodes hold nothing.
"$@"
