#!/usr/bin/env bash
# Start the AutoDRIVE devkit bridge for the simulator's Windows window, and
# print the command to run on Windows. Runs on the WSL host (Docker Desktop).
#   AUTODRIVE_ROS_DOMAIN_ID=66 scripts/autodrive_gui.sh
# Stop it with: docker stop autodrive-devkit-gui
# docs/agents/running-wsl.md, "AutoDRIVE GUI", has the measured route.
set -euo pipefail

domain="${AUTODRIVE_ROS_DOMAIN_ID:-}"
if [ -z "$domain" ] || [ "$domain" = "42" ]; then
    echo "set AUTODRIVE_ROS_DOMAIN_ID to a domain that is not 42" \
        "(the headless stack and dev use it)" >&2
    exit 2
fi

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
name=autodrive-devkit-gui

# One listener on 4567 at a time: refuse rather than stop someone else's.
if [ -n "$(docker ps -q --filter publish=4567)" ]; then
    echo "port 4567 is already published by a container; not touching it" >&2
    exit 3
fi

docker compose -f "$root/docker/docker-compose.yaml" --profile autodrive \
    run --rm --no-deps -d -p 127.0.0.1:4567:4567 --name "$name" \
    autodrive-devkit

for _ in $(seq 1 30); do
    if (exec 3<>/dev/tcp/127.0.0.1/4567) 2>/dev/null; then
        break
    fi
    sleep 1
done

cat <<EOT
bridge ready on 127.0.0.1:4567 (ROS_DOMAIN_ID=$domain)

On Windows, in PowerShell with the culture set to en-US (Set-Culture en-US;
the sim writes numbers in the Windows culture and a ',' stalls the bridge):
  curl.exe -L -o sim.zip https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-iros/autodrive_simulator_practice_windows.zip
  (Get-FileHash sim.zip -Algorithm SHA256).Hash   # 2D3E5CC7073009D8F28F41B2266C129DD64210688C70228A6811B844EE48A751
  & "C:\dev\Autodrive\autodrive_simulator\AutoDRIVE Simulator.exe" -ip 127.0.0.1 -port 4567
(asset autodrive_simulator_practice_windows.zip, unzipped to C:\\dev\\Autodrive)

To drive from the keyboard (Autonomous mode needs a command source):
  docker exec -it $name bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=$domain && ros2 run autodrive_roboracer teleop_keyboard'
Stop: docker stop $name
EOT
