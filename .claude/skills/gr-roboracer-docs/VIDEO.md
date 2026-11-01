# Recording a demonstration video

A video is evidence: it shows the **documented command**, on `main`'s code, run unchanged. The
scripts are in `scripts/`; each was written after a capture that failed silently.

## 0. Claim the heavy lane

Recording runs a full lap at `time_scale` 1.0 with RViz on screen, so it needs a quiet host. Tell
every other session using this repo before you start and when you end, and wait for theirs to
finish a `--ready` or a long CPU run. Record in your own container — its own compose project,
image tag, build/install volumes and `ROS_DOMAIN_ID` — never in `gr-roboracer-dev`. RViz display
setup on the WSL2 host is in `docs/agents/running-wsl.md`. `ffmpeg` and the label font are in the
`dev` image (ADR 0002 amendment 2026-10-28): build your recording image from `main`'s `dev` target
under its own tag. If `ffmpeg -version` fails in it, the image predates that change: rebuild it
under your own tag, never `gr-roboracer:dev`; do not install by hand.

**Done when** `ffmpeg -version` succeeds in your container, the other sessions have acknowledged
and `ps` in your container shows no `install/racing_`, `ros2 launch`, `rviz2` or `ffmpeg` process.

## 1. Record

Check the container's checkout out at `main`'s merge commit of the change being shown, and
build it. Take the launch command **verbatim** from its workflow page (grep it; do not retype).
Copy `scripts/` into the container under `/ws/log/<name>/`, then:

```bash
bash /ws/log/<name>/record.sh <name> /ws/log/<name> -- <documented launch command>
```

It records RViz at 15 fps with `grab_rviz.py` (ffmpeg's own `x11grab` is refused under WSLg),
logs `/clock` best-effort (the sim publishes it BEST_EFFORT; a reliable subscriber logs nothing),
waits for `/scenario/metrics`, and kills the whole graph.

A failed or colliding lap is **reported, never tuned**: no argument, parameter, timeout or
`time_scale` changes to make it pass — it may be a defect in the docs or the code. One retake of
the same command is allowed only when the capture tool failed. If ffmpeg died without writing
its index, `salvage_mp4.py` rebuilds the stream losslessly from a reference clip.

**Done when** `<name>-metrics.txt` shows `lap_completed: true` with `collision_count: 0`, and
`<name>-raw.mp4` plays.

## 2. Encode with the speed label

Find sim 0 (the first `<name>-clock.csv` sample, matched to `<name>-frames.csv`) and the
real-time factor (least squares of sim against wall over the lap). Trim from 2 s before sim 0 to
1 s past the lap end, then:

```bash
bash encode.sh <name>-raw.mp4 <start_s> <end_s> <rtf> <out_stem>
```

It writes `<out_stem>.mp4` (H.264, 4x, faststart) and `<out_stem>.gif` (640 px, 10x, under
5 MiB), each with its multiplier — edit speed times the measured real-time factor — burned
into the top-right corner, and prints both multipliers for the captions.

**Done when** you have looked at frames at 5%, 50% and 95% of the MP4 and each shows the car,
the track and the speed label.

## 3. Upload to Drive

`rclone` on the host has a remote `gr-docs-videos:` scoped to the owner's shared demonstrations
folder. Copy the MP4 out of the container, then:

```bash
~/.local/bin/rclone copy <out_stem>.mp4 gr-docs-videos:
~/.local/bin/rclone link gr-docs-videos:<out_stem>.mp4
```

Write the URL as `https://drive.google.com/file/d/<id>/view`. Check it signed out: `curl -sL`
returns 200 on `drive.google.com` with the file name in the title and no "Request access". If
`rclone listremotes` does not list `gr-docs-videos:`, hand the MP4 to the owner and wait for
the link.

**Done when** the signed-out check passes.

## 4. Retire the previous video, then place the new one

GitHub cannot play a Drive video or a Drive GIF inline, so the current video keeps its GIF in
`docs/assets/` and every earlier one is a Drive thumbnail on `docs/demonstrations.md`. Create
that page on first use, with an intro saying each picture opens its full recording on Google
Drive.

Move the video being replaced to the top of `docs/demonstrations.md` (newest first):

```markdown
### <What it shows> (<YYYY-MM-DD>)

[![<What it shows>](https://drive.google.com/thumbnail?id=<id>&sz=w640)](https://drive.google.com/file/d/<id>/view)

▶ [Watch the full video (Google Drive)](https://drive.google.com/file/d/<id>/view), <M>x speed.
Recorded with `<command>` on `<scenario>`, seed <n>, at commit `<sha>`. <What it was replaced by.>
```

Delete its GIF from `docs/assets/`; git history keeps it. Then put the new GIF in its place,
linked to its Drive URL, with the same `▶ Watch the full video` line and caption beneath. The
README carries no lap time; numbers belong to `docs/results.md`.

**Done when** `docs_check.py` passes, `docs/assets/` holds only current GIFs, and every video
ever shown is either on its owner page or in `docs/demonstrations.md`.
