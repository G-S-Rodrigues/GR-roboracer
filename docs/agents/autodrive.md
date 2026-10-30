# Launching and connecting the AutoDRIVE simulator (agents)

The competition simulator's window is a **Windows** build; an agent in WSL starts it through
`powershell.exe`. The measured route, rates and traps are in `docs/agents/running-wsl.md`
("AutoDRIVE GUI"); this page is what to do and what not to assume.

## Launch and connect

1. Start the bridge first (the sim connects once, gotcha 30):
   `AUTODRIVE_ROS_DOMAIN_ID=<not 42> scripts/autodrive_gui.sh` (it publishes `127.0.0.1:4567`).
2. Run the helper `scripts/windows/autodrive.ps1` (optional; it is a click automation, not CI):
   `powershell.exe -NoProfile -Command ". '<windows path of autodrive.ps1>'; autodrive -SimPath '<...\AutoDRIVE Simulator.exe>'"`.
   Parameters: `-SimPath` (or `$env:AUTODRIVE_SIM_EXE`), `-Ip` (default `127.0.0.1`; a recorder on
   the WSL host needs the WSL IP), `-Port`, `-NoConnect`, `-NoTrackcam`, `-LoadSeconds`.
3. Confirm the connection in the **server log**: a session id (`sid`) and messages arriving. Do not
   trust the window's "Connected" label.

## Facts (from the AutoDRIVE source @75b7df5 and live runs)

- `-ip` / `-port` only fill the address fields. The build connects alone only under `-batchmode`
  (no window). In the GUI, **one click on Menu > Connection** connects; a second click disconnects.
  The earlier "the flags auto-connect" wording was wrong.
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
