# Launching and connecting the AutoDRIVE simulator (agents)

The competition simulator's window is a **Windows** build; an agent in WSL starts it through
`powershell.exe`. The measured route, rates and traps are in `docs/agents/running-wsl.md`
("AutoDRIVE GUI"); this page is what to do and what not to assume.

## Launch and connect

1. Start the bridge first (the sim connects once, gotcha 30):
   `AUTODRIVE_ROS_DOMAIN_ID=<not 42> scripts/autodrive_gui.sh` (it publishes `127.0.0.1:4567`).
2. Run the helper `scripts/windows/autodrive.ps1` (optional; it is a click automation, not CI):
   `powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ". '<windows path of autodrive.ps1>'; autodrive -SimPath '<...\AutoDRIVE Simulator.exe>'"`.
   `-ExecutionPolicy Bypass` applies to that process only; it changes no machine setting.
   Parameters: `-SimPath` (or `$env:AUTODRIVE_SIM_EXE`), `-Ip` (default `127.0.0.1`; a recorder on
   the WSL host needs the WSL IP), `-Port`, `-NoConnect`, `-NoTrackcam`, `-LoadSeconds`.
3. Confirm the connection in a log, not the window label: `Connected!` in
   `docker logs autodrive-devkit-gui` on the devkit route, or a session id (`sid`) in our own
   server's or a probe recorder's log.

## The route to our own server (headless server in `dev`, Windows sim)

`dev` publishes no port (ADR 0002), so the Windows sim reaches the server in it through two hops:
`scripts/autodrive_route.sh` starts a relay container (`scripts/autodrive_relay.py`, publishing
`127.0.0.1:4567`) and a `socat` on the WSL IP. Start the server first, then the script, then the
sim with `-ip <WSL IP>`; `scripts/autodrive_route.sh stop` ends both. It refuses a taken 4567.

**Socat's `nodelay` is the one that matters; the relay sets TCP_NODELAY too, with no measurable
effect of its own.** Offline lock-step replay of the recorded W1a stream (3000 frames,
reply latency, p50 / p99 / max in ms), part 8:

| path | p50 | p99 | max |
|---|---|---|---|
| direct to the server | 3.7 | 9.4 | 59.6 |
| WSL IP -> socat -> relay -> server (part 7's route) | 48.2 | 59.6 | 144.9 |
| the same, relay `NODELAY`, socat without | 48.1 | 53.8 | 96.4 |
| the same, socat `nodelay` | 4.6 | 7.3 | 9.0 |
| socat `nodelay` and relay `NODELAY` (the script) | 4.3 | 7.0 | 8.3 |
| relay alone, `NODELAY` off, no socat (client on the host) | 4.4 | 6.8 | 9.3 |

asyncio enables TCP_NODELAY on every TCP connection by default, so the rows with relay `NODELAY` off did not actually turn it off, and "no measurable effect" means nothing was varied there, not that the option is inert.

Without `nodelay` on socat, Nagle and delayed ACK hold each small Engine.IO write about 45 ms. That
is consistent with the 11.5-14.7 Hz part 7 measured live only if the sim is lock-step (24.5 + 45 ms is
about 70 ms, 14 Hz); that is not established, and part 7's 1.1 s gaps are unexplained (the offline max
is 145 ms). The direct row's 59.6 ms max misses the 50 ms bar for the server alone; it is one outlier
and not the handler's (its own max is 4-13 ms), and it is unexplained. The direct row's client ran in
`dev`, the route rows' on the host, so they are not like for like. The server's own handler is not the bound: per frame it
takes p50 1.1 ms, p99 2.0-2.5 ms, max 4-13 ms (decode 0.3, session 0.6, all publishes 0.35, emit
0.05), with or without subscribers on `/scan` and `/odom`. Verifying the live rate through this
route needs the Windows sim running against it, not an offline replay.

## Facts (from the AutoDRIVE source @75b7df5 and live runs)

- `-ip` / `-port` only fill the address fields. The build connects alone only under `-batchmode`
  (no window). In the GUI, **one click on Menu > Connection** connects; a second click disconnects.
- No flag or setting selects a camera. Driver's Eye is the start view; **5 presses of Menu > Camera
  Switch** reach Trackcam (Bird's, God's, Dash, Rear, Track).
- The helper clicks at window-relative fractions (Menu 0.04,0.04; Connection 0.04,0.379; Camera
  Switch 0.04,0.563). **Verified at one 1920x991 window only**; another size or layout may miss.
- Timing: wait for the window (up to 60 s), then `-LoadSeconds` (default 12; raise it on a slow
  load), 1000 ms after the Menu click, 600 ms after every other. A press lost at 400 ms once
  landed on Rearcam instead of Trackcam, so do not shorten these.
- A test recorder standing in for the server must speak **Engine.IO v3**: python-socketio 4.2.0 /
  python-engineio 3.13.0. 5.x accepts the TCP connection but never yields a session.
- The Windows culture must be en-US while the sim runs (gotcha 38): a comma decimal stalls the
  bridge. The owner keeps it en-US.

## Do not

- Route around a permission block: if launching the sim or sending clicks is refused, stop and ask
  the owner. Only an explicit owner approval covers it.
- Leave a second listener on 4567 or a sim running when you are done.

## The launch switch (part 8b)

`simulator:=autodrive` on `sim_pure_pursuit.launch.py` starts our server (`autodrive_bridge:=ours`);
the argument table and refusals are in `docs/configuration.md`. `config/autodrive/porto_frame.yaml`
is `measured: derived` (two offline fits, tx +-0.15 m); part 10's lap measures it. SIM-3610 drives the
whole launch from the recorded stream on an ephemeral port and judges STALE_INPUT against the data,
not the replay process: any latch stamped at or before the last `/ground_truth/odom` fails, and the
end-of-stream latch (commands stop, the supervisor latches 100 ms later) must arrive. A mid-stream
latch after an odom gap of 100 ms or more is reported as a stand-in stall. The
sim races the track clockwise while `porto.yaml`'s centerline is counter-clockwise (open topic
porto-race-direction); Pure Pursuit on that centerline is part 10's to settle.

## Front end B, the official devkit bridge (part 9)

`autodrive_bridge:=official` runs `racing_sim_autodrive_devkit` (node `racing_sim`) in place of the server.
The devkit container bridges the simulator's Socket.IO to 13 ROS topics on its own domain `D`; B holds two
rclpy contexts, the stack's and `D`, and feeds the same `BridgeSession` as A through `on_sim_frame`. Frames are
assembled by DDS `publication_sequence_number` (a gap is a lost frame, dropped and counted, never paired
across). A frame whose stamps spread beyond `SKEW_S` = 75 ms (a publisher stalled mid-burst; measured 2026-10-31 over ~3300 stand-in frames per condition, source_timestamp spread max 13.6 ms and p99 2.9 ms unloaded, max 36.7 ms and p99 18.8 ms under `taskset -c 0,1` plus 4 busy loops, bar one 276 ms outlier; 75 ms is 2x the loaded max) is dropped alone and
counted as `stamp_dropped`, with no resync; ips != odom, a sequence regression or a backlog still resyncs. The stamp and `dt` come from the devkit's `lidar` header stamp. Commands are latched on the devkit:
steering and throttle are published once per assembled frame (not as `/drive` arrives), and reset is held True until the session sees the reset take
effect, then False.

Three places hold its tests, by what each needs:
- CI-able: tier 1 `SIMAD-14xx`, tier 2 `SIMAD-22xx`, the conformance row `autodrive_official`, and tier 3
  `SIM-3620` (the whole launch against `racing_sim_autodrive.devkit_standin`).
- Local, with the devkit container and no simulator: `tests/autodrive/` holds `SIM-3710` (B equals A on the
  contract, command lag at most 3 frames) and `SIM-3720` (the stand-in equals the real devkit). They fail,
  never skip, while the container is down. Never move them under `tests/system/`. Run, from `dev`:
  `AUTODRIVE_ROS_DOMAIN_ID=64 docker compose -f docker/docker-compose.yaml --profile autodrive up -d --no-deps autodrive-devkit`,
  then `ROS_DOMAIN_ID=62 AUTODRIVE_ROS_DOMAIN_ID=64 python3 -m pytest tests/autodrive -s`. Run them on every
  edition bump.
- Live, with the simulator: a manual slot after the PR.
