# Recording a demonstration video

A video is evidence. There are two kinds.

- A **demonstration** shows a documented command, on `main`'s code, run unchanged, and drives on an
  estimator's localization (`pose_source` other than `ground_truth`; R-d), on the gym and on
  AutoDRIVE alike: that is what proves the whole stack works. One exemption: a **mapping video**
  that drives on ground truth while the estimator builds the map is allowed, and is labelled a
  survey in its caption.
- A **development evidence** video (R-c) comes from any command: a test, a probe, a script, a
  branch. It carries its command and commit in a label and caption, is uploaded to the Drive folder
  `dev evidences` (section 3) with a local copy in `.scratch/<feature>/run/video/`, and never fills
  the README or `docs/results.md` slots unless re-recorded under the demonstration rules.

The scripts are in `scripts/`; each was written after a capture that failed silently.

**Every RViz video shows the map the run uses.** The Map display must be live and aligned with the
`/visualization/track` walls (gym and AutoDRIVE). amcl serves its survey grid and slam_toolbox
publishes its own map, so start nothing. When the run publishes no `/map` (ground truth; MOLA, if
it publishes none), start `python3 scripts/observer_map.py --scenario <the command's scenario>
--pose-source <its pose source>` in the capture container, **after `record.sh` has written
`<name>-geom.txt`** (`record.sh`'s leftover check matches `lib/nav2_`, so an observer started
first makes it exit 3; its `pkill -f lib/nav2_` cleanup stops the observer's nodes and the helper
exits with them). The helper refuses `--pose-source amcl` and `slam_toolbox` (exit 3), which publish `/map` themselves; that is what guards against two `/map` publishers, since in the headless AutoDRIVE order the observer starts first and no publisher exists yet to see. Wait for
its `observer map: ... ok` line before recording starts.

**Only the simulator's window is ever saved** (R-i). Never the desktop, never another window;
desktop capture (`ddagrab`), cropped or not, is not allowed, and this binds probe and test captures
too. A capture that holds anything but the sim window is deleted after the check that found it,
and reported. The RViz modes comply: `grab_rviz.py` reads the RViz window itself.

## Modes

The steps below are the **gym** mode. A command on the AutoDRIVE simulator differs where a step
says so; pick the mode the command's workflow page documents.

| Mode | When | Status |
|---|---|---|
| **gym** | a `ros2 launch` on `f1tenth_gym_jax` (the default simulator) | as written below |
| **AutoDRIVE (the sim's own view)** | the competition sim's Windows window on Trackcam (`docs/agents/autodrive.md`); a video of AutoDRIVE records this window, never RViz | **unverified**: never recorded |
| **AutoDRIVE headless** | development evidence only, where no GUI can run (CI, Linux without Vulkan): a command that runs the `autodrive-sim` container, such as `scripts/autodrive_qualify.sh`; RViz shows the map | run once, 2026-11-10 (the fast min-time Porto lap, commit `e604c81`) |

**AutoDRIVE (the sim's own view), unverified.** User docs may document the Windows steps, so a
workflow page can give the command; `docs/running.md` will hold it once a run proves it.
`autodrive_qualify.sh` recreates the headless sim, so it cannot drive the window. The route to our
server, the Trackcam clicks and "Recording the GUI window" (R-i, Windows ffmpeg `gfxcapture` by the
sim's window, run from WSL) are in `docs/agents/autodrive.md`. Encoding, the label and placement
below apply. When a run has recorded it, write this mode from what that run did.

## 0. Claim the heavy lane

Recording runs a full lap at `time_scale` 1.0 with RViz on screen, so it needs a quiet host. Tell
every other session using this repo before you start and when you end, and wait for theirs to
finish a `--ready` or a long CPU run. Record in your own container — its own compose project,
image tag, build/install volumes and `ROS_DOMAIN_ID` — never in `gr-roboracer-dev`. RViz display
setup on the WSL2 host is in `docs/agents/running-wsl.md`. `ffmpeg` and the label font are in the
`dev` image (ADR 0002 amendment 2026-10-28): build your recording image from `main`'s `dev` target
under its own tag. If `ffmpeg -version` fails in it, the image predates that change: rebuild it
under your own tag, never `gr-roboracer:dev`; do not install by hand.

**AutoDRIVE headless.** The run is free-running at the sim's own pace (no `time_scale`, no
`/clock`). `autodrive_qualify.sh` runs the stack in `gr-roboracer-dev` by design, so only the
capture is yours: start your container with `docker run` (not the shared compose project, gotcha
34) on the `gr-roboracer_default` network, with the `dev` service's X11/WSLg mounts and
environment, `ROS_DOMAIN_ID` equal to the script's `--domain`, and its own build/install/log
volumes. Cross-container DDS on that network works (probed with a `ros2 topic pub` in `dev` and
`ros2 topic echo` in yours).

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

**AutoDRIVE headless.** The documented command is a host script that starts no RViz
(`use_rviz:=false`), so RViz runs in your container as an observer and `record.sh` is given it in
place of the launch:

1. Make a `.scratch/` worktree at `main`'s merge commit and build it in `dev` too (gotchas 20, 33);
   the script's documented `--workdir /ws/.scratch/<worktree>` runs the stack on it. `/ws` itself is
   whatever branch the main checkout is on.
2. Copy `racing_vehicle_description/rviz/sim_pure_pursuit.rviz` (the launch's own config) into
   `/ws/log/<name>/`. A path containing `install/racing_` makes `record.sh`'s cleanup `pkill -f`
   kill `record.sh` itself: seen 2026-11-10, after ffmpeg had closed (the MP4 was whole), but
   before the leftover check ran.
3. In your container, log the sim's lap clock, which stands in for `/clock`:
   `ros2 topic echo --csv /racing_sim/autodrive_counters racing_interfaces/msg/SimCounters >
   /ws/log/<name>/counters.csv` (columns: stamp sec, nanosec, frame, `lap_count`, `lap_time`,
   `last_lap_time`, `best_lap_time`, `collisions`; the stamp is wall time).
4. Start `record.sh <name> /ws/log/<name> -- rviz2 -d /ws/log/<name>/sim_pure_pursuit.rviz`, wait
   for `<name>-geom.txt`, start the observer map (`observer_map.py`, above) and wait for its `ok`
   line, then run the documented command on the host. `/scenario/metrics` is
   published once, after the last of `laps:` laps, so `record.sh` stops after the whole run. The
   script recreates and stops `autodrive-sim` itself, which handles the connect order (gotcha 30);
   the headless sim has no culture issue (gotcha 38 is the Windows build) and uses no devkit (40).
5. Stop the counters logger. The command's own verdict (`1 passed`, its `FAST_LAP` or
   qualification line) is the lap result; `<name>-clock.csv` stays empty.

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

**AutoDRIVE headless.** Sim 0 is the counters row where `lap_time` restarts after the reset, and the
lap end is the row where `lap_count` reaches the last lap; map both wall stamps to video seconds
through `<name>-frames.csv` (`n / 15`). The real-time factor is the least squares of cumulative sim
time (`lap_time` plus the earlier laps' `last_lap_time`) against the stamp between them (0.998 on
2026-11-10). Two Porto laps are about 15 s, so pass edit speeds 1 and 2
(`encode.sh ... <out_stem> 1 2`): the default 4x and 10x make a 4 s video and a 2 s GIF.

**Done when** you have looked at frames at 5%, 50% and 95% of the MP4 and each shows the car,
the track, the speed label **and the map under the track, aligned with the `/visualization/track`
walls**. The RViz Displays panel shows no warning on Map: check it on the first frame or in a
screenshot of the panel before recording. For the AutoDRIVE sim's view the frames show Trackcam,
not Driver's Eye.

## 3. Upload to Drive

`rclone` on the host has a remote `gr-docs-videos:` scoped to the owner's shared demonstrations
folder. Copy the MP4 out of the container, then, for a demonstration (the folder root):

```bash
~/.local/bin/rclone copy <out_stem>.mp4 gr-docs-videos:
~/.local/bin/rclone link gr-docs-videos:<out_stem>.mp4
```

A **development evidence** video goes to the subfolder `dev evidences`, never the root. Keep the
local copy in `.scratch/<feature>/run/video/`, and put the link in the PR body (`gh` cannot attach
files):

```bash
~/.local/bin/rclone mkdir "gr-docs-videos:dev evidences"        # first use only
~/.local/bin/rclone copy <file>.mp4 "gr-docs-videos:dev evidences"
~/.local/bin/rclone link "gr-docs-videos:dev evidences/<file>.mp4"
```

Write the URL as `https://drive.google.com/file/d/<id>/view`. Check it signed out: `curl -sL`
returns 200 on `drive.google.com` with the file name in the title and no "Request access". If
`rclone listremotes` does not list `gr-docs-videos:`, hand the MP4 to the owner and wait for
the link.

**Done when** the signed-out check passes.

## 4. Retire the previous video, then place the new one

(Demonstrations only. A development evidence video is placed nowhere in the docs: its link goes in
the PR body.)

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

A demonstration on an estimator replaces a ground-truth one in the same slot, and the old one
moves to `docs/demonstrations.md` as written above.

A video that is not the README headline (the mapping video, the AutoDRIVE fast lap) sits with its
numbers in `docs/results.md`, GIF in `docs/assets/`; when it is the first of its kind there is
nothing to retire. Whether it replaces the README headline is the owner's call (SKILL.md step 3).

**Done when** `docs_check.py` passes, `docs/assets/` holds only current GIFs, and every video
ever shown is either on its owner page or in `docs/demonstrations.md`.
