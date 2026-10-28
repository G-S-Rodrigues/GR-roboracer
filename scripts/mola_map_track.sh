#!/bin/bash
# Regenerate a track's committed MOLA map (part 1 of the MOLA plan).
#
# One lap of the reference stack on ground truth at 1x, seed 7 by default,
# while MOLA-LO's lidar2d pipeline (config/localization/
# mola_lidar2d_pipeline.yaml) runs in mapping mode beside it, reading /scan
# and /odom and nothing from /ground_truth. The start is held; this script
# waits for MOLA to subscribe /scan and publish, then calls the sim's
# ~/reset, so MOLA's origin is the car's pose at t=0 (repo-gotchas #19). MOLA
# writes a simplemap; sm2mm turns it into the native .mm (a 5 cm CVoxelMap
# layer named `localmap`, config/localization/mola_sm2mm.yaml).
# Run in the dev container, from /ws, after a colcon build:
#
#   ./scripts/mola_map_track.sh config/scenarios/spielberg_1080.yaml \
#       config/scenarios/maps/Spielberg/mola/Spielberg 7
#
# The output is <stem>.mm + <stem>.yaml (the provenance, with the alignment T
# from the map frame to the world frame: the /ground_truth/odom pose at the
# stamp of MOLA's first processed scan), owned by the container's user: chown
# it back before committing it as a reviewed diff. MOLA is multi-threaded, so
# two runs are close but not identical.
set -euo pipefail

scenario=${1:?usage: mola_map_track.sh <scenario.yaml> <output-stem> [seed]}
stem=${2:?usage: mola_map_track.sh <scenario.yaml> <output-stem> [seed]}
seed=${3:-7}
lap_wait=${LAP_WAIT:-300}
time_scale=1.0

trap 'echo "mola_map_track.sh: failed at line $LINENO: $BASH_COMMAND" >&2' ERR
cd "$(dirname "$0")/.."
# colcon's setup scripts read unset variables.
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
mkdir -p log "$(dirname "$stem")"
case "$stem" in
/*) output=$stem ;;
*) output=$(pwd)/$stem ;;
esac
simplemap=$(pwd)/log/mola_mapping.simplemap
rm -f "$simplemap"

mola_args=$(python3 - <<'PY'
from pathlib import Path

import yaml

config = Path("config/localization/mola_lidar2d_mapping.yaml").resolve()
for key, value in yaml.safe_load(config.read_text()).items():
    if key == "mola_lo_pipeline":
        value = config.parent / value
    print(f"{key}:={value}")
PY
)

ros2 launch racing_bringup sim_pure_pursuit.launch.py \
    scenario:="$scenario" use_rviz:=false seed:="$seed" start_held:=true \
    time_scale:="$time_scale" recording_path:=log/mola_mapping.jsonl \
    >log/mola_mapping_launch.log 2>&1 &
launch=$!
# shellcheck disable=SC2086
MOLA_SIMPLEMAP_OUTPUT=$simplemap ros2 launch mola_lidar_odometry \
    ros2-lidar-odometry.launch.py $mola_args \
    >log/mola_mapping_mola.log 2>&1 &
mola=$!
# Match the node executables' path, never a bare "mola": pkill -f would also
# kill any shell whose command line merely mentions it.
trap 'kill -TERM "$mola" "$launch" 2>/dev/null || true; sleep 5; pkill -f lib/mola_launcher/ || true; pkill -f install/racing_ || true; pkill -f racing_mola_map_alignment || true; pkill -f robot_state_publisher || true' EXIT

# Deterministic t=0: the sim is held at clock 0 and republishes its snapshot.
# Wait for MOLA to subscribe /scan and process its first scan - which it logs
# as the initial re-localization, at the held stamp 0, so that stamp is the
# one T is taken at - then reset.
first_stamp=0.0
for _ in $(seq 1 120); do
    ros2 service list 2>/dev/null | grep -q racing_sim/reset &&
        grep -q "Initial re-localization done" log/mola_mapping_mola.log &&
        break
    sleep 2
done
grep -q "Initial re-localization done" log/mola_mapping_mola.log
# Subscribe to the metrics while the sim is still held, so that the CLI's
# start-up and DDS discovery do not land in the first seconds of the lap, where
# a stall would latch STALE_INPUT (gotcha #19).
timeout "$lap_wait" ros2 topic echo --once \
    --qos-durability transient_local --qos-reliability reliable \
    /scenario/metrics racing_interfaces/msg/ScenarioMetrics \
    >log/mola_mapping_metrics.txt &
echo_pid=$!
sleep 5
ros2 service call /racing_sim/reset racing_interfaces/srv/Reset \
    "{seed: $seed, scenario_id: \"\"}" >log/mola_mapping_reset.log
wait "$echo_pid"
grep -E "lap_completed|lap_time|collision_count" log/mola_mapping_metrics.txt
grep -q "lap_completed: true" log/mola_mapping_metrics.txt

# A clean MOLA shutdown writes the simplemap before the launch exits; wait on
# the launch's own pid (a watchdog kills it if the save hangs).
# SIGINT goes to mola-cli itself: sent only to `ros2 launch` it was measured to
# leave mola-cli running for 15 minutes, never saving.
kill -INT "$mola" 2>/dev/null || true
pkill -INT -f lib/mola_launcher/mola-cli || true
( sleep 900; kill -KILL "$mola" 2>/dev/null ) &
watchdog=$!
wait "$mola" || true
kill "$watchdog" 2>/dev/null || true
test -s "$simplemap" || { echo "no simplemap written" >&2; exit 1; }
# Stop the stack, so the recording the provenance reads is complete.
# SIGTERM: a background job of a non-interactive shell starts with SIGINT
# ignored, so a SIGINT sent to `ros2 launch` does nothing.
kill -TERM "$launch" 2>/dev/null || true
( sleep 120; kill -KILL "$launch" 2>/dev/null ) &
watchdog=$!
wait "$launch" || true
kill "$watchdog" 2>/dev/null || true
sm2mm -i "$simplemap" -o "$output.mm" -p config/localization/mola_sm2mm.yaml

python3 - "$output" "$scenario" "$seed" "$time_scale" "$first_stamp" "$stem" <<'PY'
import hashlib, json, math, subprocess, sys
import yaml

output, scenario, seed, time_scale, first_stamp, stem = sys.argv[1:]
first_stamp = float(first_stamp)
truth = None
for line in open("log/mola_mapping.jsonl"):
    try:
        record = json.loads(line)
    except json.JSONDecodeError:
        continue  # the stack was stopped mid-write: a cut last line
    if record.get("topic") != "/ground_truth/odom":
        continue
    stamp = record["stamp"]["sec"] + record["stamp"]["nanosec"] * 1e-9
    if truth is None or abs(stamp - first_stamp) < abs(truth[0] - first_stamp):
        truth = (stamp, record["payload"])
_, pose = truth
yaw = 2 * math.atan2(pose["qz"], pose["qw"])


def version(package):
    return subprocess.check_output(
        ["dpkg-query", "-W", "-f=${Version}", package], text=True
    ).split("-")[0]


mm = open(f"{output}.mm", "rb").read()
name = stem.rsplit("/", 1)[-1]
provenance = {
    "command": (
        "./scripts/mola_map_track.sh "
        f"{scenario} {stem} {seed}"
    ),
    "scenario": scenario,
    "seed": int(seed),
    "time_scale": float(time_scale),
    "mola_version": version("ros-jazzy-mola-lidar-odometry"),
    "mp2p_icp_version": version("ros-jazzy-mp2p-icp"),
    "T_map_mola": {
        "x": round(pose["x"], 4),
        "y": round(pose["y"], 4),
        "z": 0.074,
        "yaw": round(yaw, 5),
    },
    "T_source": "first_processed_scan",
    "mm_sha256": hashlib.sha256(mm).hexdigest(),
    "mm_bytes": len(mm),
}
with open(f"{output}.yaml", "w") as handle:
    handle.write(
        f"# MOLA-LO's map of {name}, built by one mapping lap; see the plan.\n"
        f"# Regenerate (dev container, from /ws, after a colcon build):\n"
        f"#   {provenance['command']}\n"
        f"# T_map_mola is the /ground_truth/odom pose at the stamp "
        f"({first_stamp}) of MOLA's\n# first processed scan, used only to "
        f"score the run against ground truth.\n"
    )
    yaml.safe_dump(provenance, handle, sort_keys=False)
PY
