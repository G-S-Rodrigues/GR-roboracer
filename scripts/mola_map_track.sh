#!/bin/bash
# Regenerate a track's committed MOLA map (MOLA plan, parts 1 and 2).
#
# One lap of the reference stack on ground truth at 1x while MOLA-LO's
# lidar2d pipeline (config/localization/mola_lidar2d_pipeline.yaml) maps
# beside it, through racing_bringup's mola_mapping.launch.py: it reads /scan
# and /odom and nothing from /ground_truth. The start is held; this script
# waits for MOLA to subscribe /scan and publish, then calls the sim's
# ~/reset, so MOLA's origin is the car's pose at t=0 (repo-gotchas #19). MOLA
# writes a simplemap; sm2mm turns it into the native .mm (a 5 cm CVoxelMap
# layer named `localmap`, config/localization/mola_sm2mm.yaml), and the grid
# is exported beside it: the simplemap moved into the world frame by T
# (`sm-cli tf`), a COccupancyGridMap2D layer (config/localization/
# mola_sm2grid.yaml) and mm2grid, so the extractor, amcl and map_accuracy read
# <stem>_grid.{png,yaml} unchanged.
# Run in the dev container, from /ws, after a colcon build:
#
#   ./scripts/mola_map_track.sh config/scenarios/spielberg_1080.yaml \
#       config/scenarios/maps/Spielberg/mola/Spielberg 7
#
# The output is <stem>.mm, <stem>_grid.{png,yaml} and <stem>.yaml (the
# provenance, with the alignment T from the map frame to the world frame: the
# /ground_truth/odom pose at the stamp of MOLA's first processed scan), owned
# by the container's user: chown it back before committing it as a reviewed
# diff. MOLA is multi-threaded, so two runs are close but not identical.
set -euo pipefail

usage="usage: mola_map_track.sh <scenario.yaml> <output-stem> <seed>"
scenario=${1:?$usage}
stem=${2:?$usage}
seed=${3:?$usage}
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

# A kill watchdog. Its stdio is closed and stop_watchdog kills its `sleep`
# too: a leftover `sleep` holding the caller's stdout pipe made
# subprocess.run(capture_output=True) wait out its whole timeout after this
# script had exited 0 (nightly 40663334967, SIM-5520; BRINGUP-1770).
start_watchdog() {
    ( exec >/dev/null 2>&1 </dev/null; sleep "$1"; kill -KILL "$2" ) &
    watchdog=$!
}
stop_watchdog() {
    # Subshell first, so it cannot go on to the kill when its sleep dies.
    local children
    children=$(pgrep -P "$watchdog" 2>/dev/null || true)
    kill "$watchdog" 2>/dev/null || true
    # shellcheck disable=SC2086
    [ -z "$children" ] || kill $children 2>/dev/null || true
}

ros2 launch racing_bringup sim_pure_pursuit.launch.py \
    scenario:="$scenario" use_rviz:=false seed:="$seed" start_held:=true \
    time_scale:="$time_scale" recording_path:=log/mola_mapping.jsonl \
    >log/mola_mapping_launch.log 2>&1 &
launch=$!
ros2 launch racing_bringup mola_mapping.launch.py \
    simplemap_output:="$simplemap" >log/mola_mapping_mola.log 2>&1 &
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
# Release t=0 only once the supervisor has heard /clock (its latched
# ~/clock_received, F1): a best-effort /clock writer matched after t=0 leaves
# its clock at 0 and latches STALE (gotcha #19). A timeout fails the script.
timeout 60 ros2 topic echo --once \
    --qos-durability transient_local --qos-reliability reliable \
    /racing_safety_supervisor/clock_received rosgraph_msgs/msg/Clock \
    >log/mola_mapping_clock_received.txt
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
start_watchdog 900 "$mola"
wait "$mola" || true
stop_watchdog
test -s "$simplemap" || { echo "no simplemap written" >&2; exit 1; }
# Stop the stack, so the recording the provenance reads is complete.
# SIGTERM: a background job of a non-interactive shell starts with SIGINT
# ignored, so a SIGINT sent to `ros2 launch` does nothing.
kill -TERM "$launch" 2>/dev/null || true
start_watchdog 120 "$launch"
wait "$launch" || true
stop_watchdog
sm2mm -i "$simplemap" -o "$output.mm" -p config/localization/mola_sm2mm.yaml

# T_map_mola: the pose of the world frame's car at MOLA's first processed
# scan (/ground_truth/odom is read here and nowhere in MOLA), rounded as the
# provenance records it so the grid and the yaml agree. Written to a file for
# the grid step and the provenance step.
t_file=$(pwd)/log/mola_mapping_T.txt
python3 - "$first_stamp" "$t_file" <<'PY'
import json, math, sys

first_stamp, out = float(sys.argv[1]), sys.argv[2]
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
with open(out, "w") as handle:
    handle.write(
        f"{round(pose['x'], 4)} {round(pose['y'], 4)} {round(yaw, 5)}\n"
    )
PY

# The grid: the simplemap moved into the world frame by T (sm-cli takes the
# yaw in degrees), then a grid-layer pipeline (a second sm2mm, because mm2grid
# reads only a COccupancyGridMap2D layer) and mm2grid, with the survey's
# thresholds. Nothing is resampled.
read -r tx ty tyaw <"$t_file"
yaw_deg=$(python3 -c "import math; print(math.degrees($tyaw))")
world_simplemap=$(pwd)/log/mola_mapping_world.simplemap
grid_mm=$(pwd)/log/mola_mapping_grid.mm
sm-cli tf "$simplemap" "$world_simplemap" "[$tx $ty 0 $yaw_deg 0 0]"
sm2mm -i "$world_simplemap" -o "$grid_mm" -p config/localization/mola_sm2grid.yaml
(cd "$(dirname "$output")" && mm2grid "$grid_mm" --occupied-thresh 0.65 \
    --free-thresh 0.196 -o "$(basename "$output")_grid")

python3 - "$output" "$scenario" "$seed" "$time_scale" "$first_stamp" "$stem" "$t_file" <<'PY'
import hashlib, subprocess, sys
import yaml

output, scenario, seed, time_scale, first_stamp, stem, t_file = sys.argv[1:]
first_stamp = float(first_stamp)
x, y, yaw = (float(v) for v in open(t_file).read().split())


def version(package):
    return subprocess.check_output(
        ["dpkg-query", "-W", "-f=${Version}", package], text=True
    ).split("-")[0]


mm = open(f"{output}.mm", "rb").read()
grid = open(f"{output}_grid.png", "rb").read()
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
        "x": x,
        "y": y,
        "z": 0.074,
        "yaw": yaw,
    },
    "T_source": "first_processed_scan",
    "mm_sha256": hashlib.sha256(mm).hexdigest(),
    "mm_bytes": len(mm),
    "loop_closure": False,
    "grid_image": f"{name}_grid.png",
    "grid_sha256": hashlib.sha256(grid).hexdigest(),
    "grid_bytes": len(grid),
    "grid_pipeline": "config/localization/mola_sm2grid.yaml",
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
