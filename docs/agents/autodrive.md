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

**Both hops must set TCP_NODELAY.** Offline lock-step replay of the recorded W1a stream (3000 frames,
reply latency, p50 / p99 / max in ms), part 8:

| path | p50 | p99 | max |
|---|---|---|---|
| direct to the server | 3.7 | 9.4 | 59.6 |
| WSL IP -> socat -> relay -> server (part 7's route) | 48.2 | 59.6 | 144.9 |
| the same, relay `NODELAY`, socat without | 48.1 | 53.8 | 96.4 |
| the same, socat `nodelay` | 4.6 | 7.3 | 9.0 |
| socat `nodelay` and relay `NODELAY` (the script) | 4.3 | 7.0 | 8.3 |

Without `nodelay` on socat, Nagle and delayed ACK hold each small Engine.IO write about 45 ms, which
is the 11.5-14.7 Hz part 7 measured live. The server's own handler is not the bound: per frame it
takes p50 1.1 ms, p99 2.0-2.5 ms, max 4-13 ms (decode 0.3, session 0.6, all publishes 0.35, emit
0.05), with or without subscribers on `/scan` and `/odom`. Verifying the live rate through this
route is the live slot's job (a L-H/L-W run), not an offline one.

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
