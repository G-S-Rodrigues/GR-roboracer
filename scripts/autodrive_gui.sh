#!/usr/bin/env bash
# Start the AutoDRIVE devkit bridge for the simulator's Windows window, and
# print the steps to run on Windows. Runs on the WSL host (Docker Desktop).
#   AUTODRIVE_ROS_DOMAIN_ID=66 scripts/autodrive_gui.sh
# Stop it with: docker stop autodrive-devkit-gui
# docs/agents/running-wsl.md, "AutoDRIVE GUI", has the measured route.
# AUTODRIVE_GUI_WAIT (seconds, default 60) bounds the readiness wait.
set -euo pipefail

domain="${AUTODRIVE_ROS_DOMAIN_ID:-}"
if ! [[ "$domain" =~ ^[0-9]+$ ]] || [ "$((10#$domain))" -gt 101 ] ||
    [ "$((10#$domain))" -eq 42 ]; then
    echo "set AUTODRIVE_ROS_DOMAIN_ID to an integer 0-101 that is not 42" \
        "(the headless stack and dev use 42)" >&2
    exit 2
fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
name=autodrive-devkit-gui
wait_s="${AUTODRIVE_GUI_WAIT:-60}"

# One listener on 4567 at a time: refuse rather than stop someone else's.
if [ -n "$(docker ps -q --filter publish=4567)" ]; then
    echo "port 4567 is already published by a container; not touching it" >&2
    exit 3
fi

docker compose -f "$root/docker/docker-compose.yaml" --profile autodrive \
    run --rm --no-deps -d -p 127.0.0.1:4567:4567 --name "$name" \
    autodrive-devkit

# Docker Desktop accepts TCP on the published port whether or not the bridge
# listens, so probe from inside the container. The sim connects only once.
ready=0
for _ in $(seq 1 "$wait_s"); do
    if docker exec "$name" bash -c 'exec 3<>/dev/tcp/127.0.0.1/4567' \
        2>/dev/null; then
        ready=1
        break
    fi
    sleep 1
done
if [ "$ready" -ne 1 ]; then
    echo "the devkit did not listen on 4567 within ${wait_s}s" >&2
    docker stop "$name" >/dev/null 2>&1 || true
    exit 1
fi

cat <<EOT
bridge ready on 127.0.0.1:4567 (ROS_DOMAIN_ID=$domain)

On Windows, in PowerShell. The sim writes numbers in the Windows culture and a
',' stalls the bridge (gotcha 38), so change it for the sitting only:
  (Get-Culture).Name     # write this down, it is what you restore later
  Set-Culture en-US
Open a NEW PowerShell window (the culture applies to new processes), then:
  \$dir = "<folder for the sim>"
  curl.exe -L -o sim.zip https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-iros/autodrive_simulator_practice_windows.zip
  (Get-FileHash sim.zip -Algorithm SHA256).Hash   # must be 2D3E5CC7073009D8F28F41B2266C129DD64210688C70228A6811B844EE48A751
  Expand-Archive sim.zip \$dir
  & "\$dir\\autodrive_simulator\\AutoDRIVE Simulator.exe" -ip 127.0.0.1 -port 4567
The flags only fill the address; the window never connects by itself. Open
Menu, click Connection once (only if not already Connected; a second click
disconnects), and trust Connected! in "docker logs
autodrive-devkit-gui", not the window label.
Menu > Camera Switch five times from Driver's Eye reaches Trackcam; no flag
selects a camera. scripts/windows/autodrive.ps1 does these clicks (verified at
one 1920x991 window); docs/agents/autodrive.md has the facts.
In Autonomous mode the car does not move until something sends commands;
Manual mode drives from the window.
Keyboard teleop for Autonomous mode:
  docker exec -it $name bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=$domain && ros2 run autodrive_roboracer teleop_keyboard'
When done, restore the culture you wrote down, e.g. Set-Culture pt-PT
Stop the bridge: docker stop $name
EOT
