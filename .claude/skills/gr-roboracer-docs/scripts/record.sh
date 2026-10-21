#!/bin/bash
# Record one documented launch with RViz on screen, inside the docs container.
# usage: record.sh <name> <out_dir> -- <documented launch command...>
# Copy this folder's scripts into <out_dir> first (the container sees /ws).
# Writes <name>-raw.mp4, -clock.csv, -frames.csv, -metrics.txt, -launch.log.
cd /ws && source /ws/setup.sh
set +e
NAME=$1; D=$2; shift 2; [ "$1" = -- ] && shift
[ "$#" -gt 0 ] || { echo "no launch command"; exit 2; }
PAT='install/racing_|ros2 launch|rviz2|ffmpeg|grab_rviz|lib/slam_toolbox/|lib/nav2_|robot_state_publisher'
left() { ps -eo pid,stat,args | grep -E "$PAT" | grep -v grep | grep -v defunct; }
if [ -n "$(left)" ]; then echo "LEFTOVERS (stale graph):"; left; exit 3; fi
mkdir -p "$D"; rm -f "$D/$NAME"-*
echo "CMD: $*" > "$D/$NAME-cmd.txt"
echo "DOMAIN: $ROS_DOMAIN_ID" >> "$D/$NAME-cmd.txt"
python3 "$D/clock_log.py" "$D/$NAME-clock.csv" > "$D/$NAME-clock.err" 2>&1 &
CLK=$!
mkfifo "$D/$NAME.fifo"; exec 3<>"$D/$NAME.fifo"
python3 -u "$D/grab_rviz.py" --fps 15 --seconds 900 \
  --frames-csv "$D/$NAME-frames.csv" --geom-out "$D/$NAME-geom.txt" \
  --crop-out "$D/$NAME-crop.txt" >&3 2>"$D/$NAME-grab.err" &
GRAB=$!
setsid "$@" </dev/null >"$D/$NAME-launch.log" 2>&1 3>&- &
LPID=$!
for _ in $(seq 1 120); do [ -s "$D/$NAME-geom.txt" ] && break; sleep 0.5; done
GEOM=$(cat "$D/$NAME-geom.txt"); echo "geom $GEOM"
# setsid + 3>&- : ffmpeg must own neither the fifo's write end nor the
# launch's signals, or it never writes its index (the 2026-10-21 failure).
setsid nice -n 5 ffmpeg -hide_banner -loglevel warning -f rawvideo \
  -pix_fmt bgr0 -video_size "$GEOM" -framerate 15 -i "$D/$NAME.fifo" \
  -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p \
  -movflags +frag_keyframe+empty_moov "$D/$NAME-raw.mp4" \
  </dev/null >"$D/$NAME-ffmpeg.log" 2>&1 3>&- &
FF=$!
timeout 900 ros2 topic echo --once --qos-durability transient_local \
  --qos-reliability reliable /scenario/metrics \
  racing_interfaces/msg/ScenarioMetrics > "$D/$NAME-metrics.txt" 2>&1
sleep 3
kill -TERM $GRAB; sleep 1; exec 3>&-
wait $FF
kill -INT $CLK
kill -INT -- -$LPID 2>/dev/null; sleep 10
pkill -f install/racing_; pkill -f lib/slam_toolbox/; pkill -f lib/nav2_
pkill -f robot_state_publisher; pkill -f rviz2; sleep 3
if [ -n "$(left)" ]; then echo "SURVIVORS:"; left; pkill -9 -f "$PAT"; sleep 1; fi
rm -f "$D/$NAME.fifo"
echo "FINAL LEFT:"; left; echo "(end)"
