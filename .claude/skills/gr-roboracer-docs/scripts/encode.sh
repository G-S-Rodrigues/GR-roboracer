#!/bin/bash
# Trim a raw recording, speed it up, burn the playback multiplier into the
# top-right corner, and write the Drive MP4 plus the README/Results GIF.
# usage: encode.sh <raw.mp4> <start_s> <end_s> <rtf> <out_stem> [mp4_speed] [gif_speed]
#   start_s/end_s: video seconds (2 s before sim 0, 1 s past the lap end)
#   rtf: measured real-time factor (sim seconds per wall second)
# Writes <out_stem>.mp4, <out_stem>.gif and prints both multipliers.
set -euo pipefail
RAW=$1; T0=$2; T1=$3; RTF=$4; OUT=$5; K=${6:-4}; G=${7:-10}
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
[ -f "$FONT" ] || { echo "missing font $FONT"; exit 2; }
M=$(python3 -c "print(f'{$K*$RTF:.1f}')")
MG=$(python3 -c "print(f'{$G*$RTF:.1f}')")
label() {  # drawtext for a multiplier, top-right, boxed for any background
  echo "drawtext=fontfile=$FONT:text='$1x speed':fontsize=$2:fontcolor=white:\
box=1:boxcolor=black@0.6:boxborderw=8:x=w-tw-16:y=16"
}
ffmpeg -hide_banner -loglevel warning -y -ss "$T0" -to "$T1" -i "$RAW" \
  -vf "setpts=PTS/$K,fps=30,$(label "$M" 32)" -an \
  -c:v libx264 -preset slow -crf 23 -pix_fmt yuv420p -movflags +faststart \
  "$OUT.mp4"
ffmpeg -hide_banner -loglevel warning -y -ss "$T0" -to "$T1" -i "$RAW" \
  -vf "setpts=PTS/$G,fps=10,scale=640:-2:flags=lanczos,$(label "$MG" 18),\
split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer" \
  "$OUT.gif"
SIZE=$(stat -c %s "$OUT.gif")
echo "mp4 $OUT.mp4 multiplier ${M}x"
echo "gif $OUT.gif multiplier ${MG}x bytes $SIZE"
[ "$SIZE" -lt 5242880 ] || { echo "GIF over 5 MiB: lower fps or width"; exit 3; }
