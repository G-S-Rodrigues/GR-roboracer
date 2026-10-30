#!/usr/bin/env bash
# Start the route the Windows AutoDRIVE sim uses to reach our server in `dev`:
#   sim -> socat (WSL IP:4567, nodelay) -> relay container (127.0.0.1:4567)
#       -> the server in dev (AUTODRIVE_SERVER_PORT, default 4567).
# Start the server first, then this, then the sim with `-ip <WSL IP>`.
# Stop it with: scripts/autodrive_route.sh stop
# Why nodelay: without it the socat hop holds small Engine.IO writes for
# Nagle and delayed ACK (measured 48 ms p50 per reply, 4.5 ms with it);
# docs/agents/autodrive.md has the numbers. AUTODRIVE_WSL_IP overrides the
# address (it changes on a WSL restart).
set -euo pipefail

relay=autodrive-route-relay
pidfile="${AUTODRIVE_ROUTE_PID:-/tmp/autodrive-route.pid}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "${1:-}" = "stop" ]; then
    if [ -f "$pidfile" ]; then
        kill "$(cat "$pidfile")" 2>/dev/null || true
        rm -f "$pidfile"
    fi
    docker stop "$relay" >/dev/null 2>&1 || true
    echo "route stopped"
    exit 0
fi

wsl_ip="${AUTODRIVE_WSL_IP:-$(hostname -I | awk '{print $1}')}"
if [ -z "$wsl_ip" ]; then
    echo "no WSL IP found; set AUTODRIVE_WSL_IP" >&2
    exit 2
fi
server_port="${AUTODRIVE_SERVER_PORT:-4567}"

# One listener on 4567 at a time: refuse rather than stop someone else's.
if [ -n "$(docker ps -q --filter publish=4567)" ] ||
    [ -n "$(ss -H -ltn 'sport = :4567' 2>/dev/null)" ]; then
    echo "port 4567 is already taken; not touching it" >&2
    exit 3
fi

dev="${AUTODRIVE_DEV_CONTAINER:-gr-roboracer-dev}"
fields="$(docker inspect -f \
    '{{.Config.Image}} {{range $n, $v := .NetworkSettings.Networks}}{{$n}} {{$v.IPAddress}}{{end}}' \
    "$dev" 2>/dev/null || true)"
read -r image network up_host <<<"$fields"
if [ -z "${up_host:-}" ]; then
    echo "container $dev has no network address; is it running?" >&2
    exit 4
fi

docker run -d --rm --name "$relay" --network "$network" \
    -p 127.0.0.1:4567:4567 -e "UP_HOST=$up_host" \
    -e "UP_PORT=$server_port" -e NODELAY=1 \
    -v "$root/scripts/autodrive_relay.py:/relay.py:ro" \
    "$image" python3 /relay.py >/dev/null

socat "TCP-LISTEN:4567,bind=$wsl_ip,fork,reuseaddr,nodelay" \
    "TCP:127.0.0.1:4567,nodelay" &
echo "$!" >"$pidfile"
sleep 1
if ! kill -0 "$(cat "$pidfile")" 2>/dev/null; then
    echo "socat did not stay up on $wsl_ip:4567" >&2
    docker stop "$relay" >/dev/null 2>&1 || true
    exit 1
fi

cat <<EOT
route ready: sim -> $wsl_ip:4567 -> relay -> $up_host:$server_port
On Windows: -ip $wsl_ip -port 4567 (scripts/windows/autodrive.ps1 -Ip $wsl_ip)
Stop: scripts/autodrive_route.sh stop
EOT
