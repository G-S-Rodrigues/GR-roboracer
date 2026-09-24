#!/bin/bash
# Regenerate a track's committed occupancy grid (plan D7, Step 3's map_server).
#
# A survey, not a SLAM run: one lap of the reference stack on ground truth
# at 1x, seed 42, on the track's exact-sensor scenario, while slam_toolbox's
# mapper (config/localization/slam_toolbox_survey.yaml, scan matching off)
# lays /scan at the exact dead-reckoned poses; then save /map.
# Run in the dev container, from /ws, after a colcon build:
#
#   ./scripts/map_track.sh config/scenarios/spielberg_survey.yaml \
#       config/scenarios/maps/Spielberg/slam/Spielberg
#
# The output is <stem>.png + <stem>.yaml, owned by the container's user:
# chown it back before committing it as a reviewed diff. Which scans the
# async mapper takes is not deterministic, so two runs are not identical:
# check the new grid against the track PNG (plan D7 has the method and the
# committed grid's numbers).
set -euo pipefail

scenario=${1:?usage: map_track.sh <scenario.yaml> <output-stem>}
stem=${2:?usage: map_track.sh <scenario.yaml> <output-stem>}
lap_wait=${LAP_WAIT:-240}

cd "$(dirname "$0")/.."
# colcon's setup scripts read unset variables.
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
mkdir -p log "$(dirname "$stem")"
# The stem is relative to the repository root, which this script has
# already cd'd to - but SIM-5010 hands it a tmp_path. Prefixing an
# absolute stem with $(pwd) makes map_saver_cli write to a path under
# /ws that does not exist, and it fails with nothing on stderr.
case "$stem" in
/*) output=$stem ;;
*) output=$(pwd)/$stem ;;
esac

ros2 launch racing_bringup slam_toolbox_mapping.launch.py \
    scenario:="$scenario" use_rviz:=false seed:=42 \
    recording_path:=log/mapping.jsonl >log/mapping_launch.log 2>&1 &
launch=$!
# Match the node executables' path, never a bare "slam_toolbox": pkill -f
# would also kill any shell whose command line merely mentions it.
trap 'kill -INT "$launch" 2>/dev/null; sleep 5; pkill -f install/racing_ || true; pkill -f lib/slam_toolbox/ || true; pkill -f robot_state_publisher || true' EXIT

# The lap's metrics are latched once it completes.
timeout "$lap_wait" ros2 topic echo --once \
    --qos-durability transient_local --qos-reliability reliable \
    /scenario/metrics racing_interfaces/msg/ScenarioMetrics \
    >log/mapping_metrics.txt
grep -E "lap_completed|lap_time|collision_count" log/mapping_metrics.txt
grep -q "lap_completed: true" log/mapping_metrics.txt

# The mapper publishes /map every map_update_interval; the last one covers
# the whole lap.
# map_saver_cli's own arguments come first and --ros-args last; its usage
# text says so, and the other order is silent - it saved a map_<epoch>.pgm
# in the working directory with default thresholds, reporting success.
# PNG, not the default PGM: the same grid is ~2.7 MB raw and ~40 kB
# compressed, and this one is committed to git.
ros2 run nav2_map_server map_saver_cli \
    -f "$output" --fmt png --occ 0.65 --free 0.196 \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=60.0 \
    -p map_subscribe_transient_local:=true
