# Competition

The RoboRacer competitions we race in: the AutoDRIVE RoboRacer Sim Racing League (our target, ICRA
2027) and the physical RoboRacer races. This file holds every edition's dates and results, the
registration estimates, the league rules as read, what the rules leave unstated, and where our stack
stands against them. Read it before a grill, spec or plan that touches AutoDRIVE, the submission
image, race-time inputs or the unseen-track final. How the simulator is launched and bridged is in
`simulation.md`; localization after a collision reset is in `localization-slam.md`; the raceline for an
unseen track is in `raceline.md`. Which external sources we trust, and on whose word, is in
[`../research/reliable-sources.md`](../research/reliable-sources.md).

## Re-check cadence (keep this current)

**Owner, 2026-10-25: re-check the league rules at least monthly, and at the start of every grill, spec
or plan that touches AutoDRIVE.** Add a row below with the date and any change. The rules below are the
2026 text; a 2027 edition may change them.

Read each time:

- https://autodrive-ecosystem.github.io/competitions/ (one page per edition below it).
- The docs repo `AutoDRIVE-Ecosystem/autodrive-ecosystem.github.io`, folder `docs/competitions/`. Watch
  for a `roboracer-sim-racing-rules-2027.md` or `-guide-2027.md`.
- Docker Hub tags of `autodriveecosystem/autodrive_roboracer_sim` (a new edition ships new tags).
- The devkit repo `AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing`.
- The race sites (`<conference><year>-race.roboracer.ai`) and the CDC competitions page. Until about
  2026-11-25, check CDC 2026 weekly: CDC 2025's edition was announced only about 5 weeks before its
  conference.

| Checked | What was read | Result |
|---|---|---|
| 2026-10-24 | rules and guide 2026 in full; docs repo @80693ddb (2026-10-20), devkit @fde15908 (2026-10-05) | six editions; no 2027 edition |
| 2026-10-28 | rules and guide 2026 (both dated 2026-09-18) against devkit `autodrive_bridge.py` @fde15908 | stamps, restricted topics and lidar frame confirmed (see Dead ends) |
| 2026-11-10, 2026-11-11 | competitions page; ICRA 2026, IROS 2026, VTC2026-Fall race sites; CDC 2026 call for competitions | no 2027 edition; no CDC 2026 sim league |
| 2026-11-14 | competitions page; docs repo @379401d4 (2026-11-09, a Slack invite link change only); devkit; Docker tags; CDC 2026 | unchanged. `docs/competitions/` holds rules and guide for 2024-2026 only. Tags are `{2025-icra, 2025-cdc-tf, 2026-icra, 2026-iros}-{explore, practice, compete}`. Devkit head still fde15908. CDC 2026's call lists no competitions and `/events/competitions` returns 404 |

## Current state

- **Target: the sim league at ICRA 2027** (owner, 2026-10-11). Entry has been a goal since
  2026-09-30. Order: the optimized raceline (issue
  [#12](https://github.com/G-S-Rodrigues/GR-roboracer/issues/12)) first, then the AutoDRIVE bridge.
  Tripwire: if the raceline is not at its part 6 by 2026-12-01, run a 1-2 day AutoDRIVE spike then and
  interleave. AutoDRIVE's unknowns must surface by about January 2027.
- **The AutoDRIVE backend is on main** under issue
  [#44](https://github.com/G-S-Rodrigues/GR-roboracer/issues/44): `simulator:=autodrive`, our Socket.IO
  server (front end A) and the official devkit bridge (`autodrive_bridge:=official`, front end B,
  [PR #71](https://github.com/G-S-Rodrigues/GR-roboracer/pull/71)). The league car drives as shipped;
  [`docs/agents/autodrive.md`](../agents/autodrive.md) and
  [`docs/configuration.md`](../configuration.md) say how.
- **The pinned edition is `2026-iros-practice`**, by digest, in
  [`config/autodrive/simulator.yaml`](../../config/autodrive/simulator.yaml). Its header lists what an
  edition bump must re-record.
- **Porto, the qualification track, is in the tree and races clockwise:**
  [`config/tracks/porto.yaml`](../../config/tracks/porto.yaml)
  ([PR #74](https://github.com/G-S-Rodrigues/GR-roboracer/pull/74)).
- **A qualification rehearsal runs the league's format** (12 laps, 10 scored, 10 s per collision):
  `tests/autodrive/test_sim_3730_porto_qualification.py`, run by `scripts/autodrive_qualify.sh`. It is in
  no gate.
- **The sim counters are debug-only in the stack**, because the league restricts them at race time
  (`/racing_sim/autodrive_counters`, [`docs/configuration.md`](../configuration.md)).
- **Not built:** a submission image, a race path that reads only allowed topics, localization that
  survives the collision reset, the unseen-track pipeline, and a Humble build. The #44 grill
  (2026-10-25) left each to a later feature.

## The two competitions

Two different things carry the RoboRacer name.

| | RoboRacer Sim Racing League (our target) | RoboRacer Autonomous Racing Competition / Grand Prix |
|---|---|---|
| What | Virtual: the AutoDRIVE simulator, a digital twin of the RoboRacer car | In person, a real 1/10 car |
| Where | Remote; teams submit a Docker image | At the host conference |
| Cost | Free, and separate from conference registration | Travel, a car, usually conference registration |
| Numbering | 1st to 6th so far | 22nd to 31st so far |

Verified 2026-11-11. The race sites for ICRA 2026, IROS 2026 and VTC2026-Fall each say they host the
physical race "as well as a virtual benchmark competition in the form of the Roboracer Sim Racing
League". ICRA 2026's and IROS 2026's link the AutoDRIVE pages (the 5th and 6th leagues). VTC2026-Fall
(Boston, Sep 6-9 2026) links no sim page and gives no sim dates, and AutoDRIVE lists no VTC edition. It
is almost certainly the same 6th league as IROS 2026, whose registration ran Aug 5 - Sep 6.

- No race site says the sim league qualifies for, or is part of, the physical race.
- No page names a prize. CDC 2024 had a winners' session ("Words from the Winners", 5 min per team).
  ICRA 2025 and IROS 2024 offered teams about a 10 min talk in a special session.
- IFAC 2026 (Busan) was physical only. CDC 2025's physical race was cancelled for low registrations and
  pointed everyone to the virtual race.

How the hosts differ (judgment, not a published ranking):

- **ICRA:** the largest robotics conference, the biggest field and the most visibility (Autoware wrote up
  the ICRA 2025 winners). Our target.
- **IROS:** the other flagship robotics conference. The 2026 field was the fastest yet.
- **CDC:** the top control-theory conference. Smaller fields (about 20 qualifiers), but VAUL won both, so
  not easy. Physical turnout is weak. If CDC 2026 announces a league, it is a dry run of the unseen-track
  final.
- **VTC, IFAC:** host physical races and have no sim league of their own.

The rules, the simulator and Porto as the qualification track have been the same at every edition.

## League rules as read (2026 text)

Sources: rules https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-rules-2026/ and
guide https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/ (both dated
2026-09-18), docs repo @80693ddb, devkit @fde15908.

- **Qualification:** 10 consecutive autonomous laps on the practice track, provided ahead of time. It
  has been Porto in every edition. Speed does not matter. Exceeding the collision tolerance
  disqualifies.
- **Final:** time attack, one car at a time, on a previously unseen track (it may be replicated in the
  physical race). 12 laps: a warm-up lap (not scored, a startup buffer), 10 scored laps, a cool-down
  lap (not scored). Score is the total time of the 10 scored laps. Best lap breaks ties.
- **Collisions:** any collider contact with the track bounds adds 10 s (wheels on the ground excepted).
  More than 10 collisions disqualifies. The car is auto-reset to the last checkpoint, which is hidden,
  and the lap timer keeps running. The rules say localization must survive the reset. Wrong-way driving
  does not stop the timer.
- **Allowed at race time:** inputs `/autodrive/roboracer_1/{lidar, front_camera, imu, left_encoder,
  right_encoder, steering, throttle}`; outputs `steering_command` and `throttle_command`.
- **Restricted (debug and training only):** `ips`, `odom`, `/tf`, `reset_command`, `collision_count`,
  `lap_count`, `lap_time`, `best_lap_time`, `last_lap_time`. Ground truth is forbidden.
- **Forbidden:** modifying the simulator, the car, the track, the protocol (WebSocket only) or the
  devkit (ROS 2 only). The devkit package `autodrive_roboracer` stays unmodified, and the organizers
  launch the official bridge.
- **Submission:** a Docker Hub image "based off of the official AutoDRIVE Devkit container", which is
  `FROM ros:humble` (Ubuntu 22.04, Python 3.10). The entrypoint `/home/autodrive_devkit.sh` holds the
  full startup; putting it in `~/.bashrc` is forbidden. A private repository adds `autodriveecosystem`
  as a collaborator. The Hub overview must give the run steps.
- **Evaluation host:** i9-14900K, RTX 4090, 64 GB. The sim runs at Ultra quality with the camera on.
  The organizers run the team image with `--network=host --ipc=host --privileged --gpus all` plus X11,
  so the GPU reaches the team container. They record a bag of all topics and the screen.
- **Car (guide 2026, unchanged since 2024):** 0.50 x 0.27 m, 3.906 kg, steering +-0.5236 rad, lidar
  1080 beams / 40 Hz / 270 deg / 0.06-10 m. The full table and our measurements of it are in
  `simulation.md` and [`docs/results.md`](../results.md).
- **Final track timing:** the edition's `compete` image is public on track-release day (2026-iros-compete
  was pushed 09-14), about 3 days before the final submission. A team could map the final track offline
  in that window (inferred, not stated).
- **Final tracks seen so far:** not real F1 circuits. Ducts as walls, with gaps and open stubs, width at
  least 0.90 m, about a 30 x 10 m footprint. The guide says a track "may" have chicanes, bifurcations
  and obstacles.

**Unstated, ask the organizers:** whether a pre-built map may be loaded in the final; the per-run time
limit; the real-time factor; the camera rate and resolution.

## Sim league editions

| # | Host | Registration | Qualification | Final track out | Final | Conference |
|---|---|---|---|---|---|---|
| 1 | IROS 2024 | Jul 22 - Aug 31 2024 | Sep 28-29 | Oct 03 | Oct 05-06 | Oct 14-18, Abu Dhabi |
| 2 | CDC 2024 | Aug 01 - Oct 31 2024 | Nov 30 - Dec 01 | Dec 04 | Dec 07-08 | Dec 16-19, Milan |
| 3 | ICRA 2025 | Feb 20 - Apr 15 2025 | May 03-05 | May 08 | May 11-13 | May 19-23, Atlanta |
| 4 | CDC + Techfest 2025 | Nov 01 - Nov 28 2025 | Nov 29-30 | Dec 02 | Dec 06-07 | Dec 2025, Rio de Janeiro |
| 5 | ICRA 2026 | Feb 23 - Apr 30 2026 | May 09 | May 13 | May 16-17 | Jun 1-5, Vienna |
| 6 | IROS 2026 (+VTC) | Aug 05 - Sep 06 2026 | not read (about Sep 13) | Sep 16 | Sep 19-20 | Sep 27-30, Pittsburgh |

Registration opens 2-3 months before qualification. CDC 2025 was the exception: 4 weeks, announced late.
Qualification is 2-3 weeks before the conference. The final is 3-4 days after the final track is
released.

### Results

Qualification times are 10 laps on Porto. Final times are the 10 scored laps on the unseen track, with
10 s added per collision.

| Edition | Qual. entries | Qual. best lap | Final winner | Final 10 laps | Final best lap |
|---|---|---|---|---|---|
| IROS 2024 | 9 (58 registered) | 11.67 s | TURTLEBOT | 141.59 s | 13.90 s |
| CDC 2024 | 20 (51 registered) | 6.96 s (VAUL) | VAUL | 103.84 s | 10.35 s |
| ICRA 2025 | 21 (58 registered) | 6.77 s | VAUL | 111.46 s | 11.28 s |
| CDC + Techfest 2025 | 20 | 6.49 s | VAUL - Old but Gold | 78.19 s | 7.74 s |
| ICRA 2026 | 28 (about 78 registered, per an NTU post) | 6.46 s (VAUL) | MonacoF1 | 110.27 s | 11.00 s |
| IROS 2026 | 32 | 4.94 s (NTU DeepSpeed) | Siga Siga Racing Team | 71.82 s | 7.16 s |

- **VAUL (Université Laval) is the team to beat:** 3 of 6 finals, top speed "above 10.6 m/s" (ICRA
  2025).
- **The ICRA 2026 final circuit is public** in NTU DeepSpeed's hackathon repo
  (https://github.com/Myaheehee/Recruitment_Hackathon_2627): 6.3 x 18.2 m, 54 m a lap.
- **IROS 2026's qualification laps are about 1.5 s faster than every earlier edition.** A new sim
  release (the `2026-iros` tags), a changed car or a changed practice track is suspected, not confirmed.
- **The final reshuffles the field.** NTU DeepSpeed went from 1st in qualification at ICRA 2026 to 11th
  in the final. Mapping and racing an unseen track decides it.

### When to register (estimates, 2026-11-11)

**Rule: register the day registration opens.** It is free; the only risk is missing it (owner advised
2026-11-11).

| Event | Conference | Registration (est.) | Qualification (est.) | Final (est.) | Status |
|---|---|---|---|---|---|
| CDC 2026 | Dec 15-18 2026, Honolulu | about mid to late Nov 2026 | about late Nov | about early Dec | Not announced (re-checked 2026-11-14) |
| **ICRA 2027** (target) | May 24-28 2027, Seoul | opens about Feb 20-25, closes about mid-late Apr 2027 | about May 1-8 | about May 9-16 | Not announced |
| IROS 2027 | Sep 26 - Oct 1 2027, Florence (third-party listing) | about late Jul to early Sep 2027 | about mid Sep | about late Sep | Not announced |
| CDC 2027 | about Dec 14-17 2027 (third-party listing), city unknown | about Aug to Nov 2027, or late notice | about late Nov | about early Dec | Not announced |

ICRA 2027 is a week earlier in the year than ICRA 2026 (Jun 1-5). An October estimate put its
registration at early March; the 2026-11-11 estimate above follows ICRA 2025's and 2026's actual
openings (Feb 20, Feb 23) instead.

Sources: https://2027.ieee-icra.org/ ; IROS 2027 https://www.beri.net/events/iros-2027 (third party);
CDC 2027 https://www.conferencelists.org/event/66th-ieee-conference-on-decision-and-control-cdc/
(third party); CDC https://cdc2026.ieeecss.org/events/call-for-competitions and
https://cdc2025.ieeecss.org/events/competitions ; race sites https://icra2026-race.roboracer.ai/ ,
https://iros2026-race.roboracer.ai/ , https://vtc2026-race.roboracer.ai/ , https://2026ifac-roboracer.com/ ;
VTC announcement
https://vtsociety.org/post/announcement/30th-roboracer-autonomous-racing-competition-vtc2026-fall ;
Autoware's ICRA 2025 write-up https://autoware.org/how-sim-racing-is-expanding-access-to-autonomy/ .
LinkedIn posts (VAUL, NTU DeepSpeed) refuse automated reading; open them by hand.

## What we measured

Our standing against the league, all on Porto in the AutoDRIVE simulator.

| Date | Run | Result | Race-legal? | Source |
|---|---|---|---|---|
| 2026-10-31 | Qualification rehearsal, base car, Pure Pursuit, three runs | lap 13.73 s, qualification score 137.3 s, 0 collisions | No: ground-truth pose | [PR #74](https://github.com/G-S-Rodrigues/GR-roboracer/pull/74), [`docs/agents/autodrive.md`](../agents/autodrive.md) |
| 2026-11-09 | Fast car (`autodrive_roboracer_fast`, cap 4.5 m/s), min-time line, 2 laps, headless | sim lap 7.38 s (stack 7.40 s), peak 4.44 m/s, 0 collisions | No: `pose_source:=ground_truth` | [PR #88](https://github.com/G-S-Rodrigues/GR-roboracer/pull/88), [`docs/results.md`](../results.md) |
| 2026-11-11 | The same fast lap on amcl, Windows GUI sim, three runs | failed 3 of 3: sim laps 1 / 2 / 0, sim collisions 11 / 38 / 1 | Pose source yes; never completed | [PR #99](https://github.com/G-S-Rodrigues/GR-roboracer/pull/99) |

- **The 7.38 s lap is a ceiling, never a race-legal time.** It ran on ground truth, which the rules
  forbid. Against the table's qualification best laps it sits 0.9 s off ICRA 2026's best (6.46 s) and
  2.4 s off IROS 2026's (4.94 s). Read on 2026-11-11, it would rank somewhere from 2nd to 8th in the
  qualification fields.
- **The collision reset is the measured threat to a race-legal run.** In PR #99's amcl runs the sim's
  respawn jumps the car 0.36-1.43 m back and up to 67 deg in yaw. `/odom` sees no jump, so each respawn
  adds 1-2 m of error and the car re-crashes about every 0.5 s. The first collision happened with good
  localization (0.06-0.21 m error). Details are in `localization-slam.md`.
- **Not yet shown:** a lap on our own estimate; 10 consecutive laps on it; a lap on a map the stack built
  itself; a speed cap near the winners' 9-10+ m/s.

## Dead ends

- **Our Socket.IO server (front end A) cannot be the race path.** The rules require the official devkit
  bridge, unmodified and launched by the organizers, so front end B is the race path. A is development
  infrastructure. No source says whether the organizers object to a custom server used only in
  development. (Checked 2026-10-28 against devkit @fde15908.)
- **Time from `V1 Lap Time` is not a race-time input.** The devkit computes no dt and republishes the sim
  lap time only as `/autodrive/roboracer_1/lap_time`, which the rules restrict at race time. The
  official bridge stamps every message with its own `get_clock().now()` at receipt, with no
  `use_sim_time`. Gotcha #35 has the numbers.
- **Ground-truth pose (`ips`, `odom`, `/tf`) cannot drive a race.** It may drive the car in development,
  as the rules allow in practice, and #44 uses it that way.
- **Porto counter-clockwise was wrong.** The first Porto track assumed the race ran counter-clockwise
  from the overview PNG, which has no arrow. The sim races clockwise: its 21 checkpoints, the spawn
  heading and the recorded drive all agree, and its lap counter incremented 12 times on the regenerated
  `porto.yaml` (resolved in PR #74).
- **Our own car in AutoDRIVE is out.** The league car is fixed and modifying it disqualifies. Racing a
  different car would need our own Unity build, which is research only.
- **The lidar frame is `lidar`**, as ours has been since 2026-10-28 (gotcha 26). The devkit publishes the LaserScan on
  `/autodrive/roboracer_1/lidar` with frame `lidar`, mounted under `roboracer_1` at (0.2733, 0, 0.096).
  The rules fix no frame. Ranges are relative to the lidar mount in the competition.

## Ideas not yet specified

- **The submission image.** Owner, 2026-10-25 (Q2): "for now lets keep on jazzy. i will make the effort
  to change for humble in the future if necessary". The image must be based on the Humble devkit, and
  ROS 2 Humble reaches end of life in May 2027, the month of the ICRA 2027 final. The risk is a Humble
  port landing near the deadline.
- **A race path on allowed topics only**, and localization that survives the teleporting reset. The #44
  grill (Q1, 2026-10-25) scoped race-time-only inputs, localization that survives the reset, the
  submission image and speed tuning out of #44, each to become a later feature.
- **Final-track strategy.** Owner, 2026-10-25 (Q3): "Offline, online fallback (Recommended)". Map the
  released track ourselves, compute the raceline offline (`raceline/` never enters the image), and bake
  map and raceline into the image. Online mapping on the warm-up lap stays selectable as the fallback.
  The race-time estimator localizes on the known map and relocalizes after a reset. A raceline solve
  inside the container is ruled out as the primary path. This depends on pre-built maps being allowed.
- **Rehearse the 3-day final on past final tracks.** Each edition's `compete` image carries that
  edition's final track, so four past finals (2025-icra, 2025-cdc-tf, 2026-icra, 2026-iros) run with no
  Unity work. The ICRA 2026 final circuit is also public (Results above).
- **Enter CDC 2026 as a dry run** if a league is announced.
- **Register for ICRA 2027 the day registration opens.**

## Open questions

- **May a pre-built map be loaded in the final?** The final-track strategy depends on it. Owner to ask
  the organizers, with the per-run time limit, the real-time factor and the camera rate.
- **Can a Jazzy stack run inside an image based on the Humble devkit,** or must the stack be ported to
  Humble? The rules say only "based off of" the devkit container. Humble-to-Jazzy DDS between two
  containers is measured to work (`simulation.md`).
- **Will the 2027 rules keep the 2026 text?** Nothing is published yet (2026-11-14).
- **Why was IROS 2026's qualification about 1.5 s faster?** If the car or the physics changed in
  `2026-iros`, results from earlier tags do not compare.
- **How loaded is the evaluation host, and at what priority do our nodes run?** Our supervisor latches on
  frame gaps over 200 ms, and the organizers' host and load at the race are a risk
  ([`docs/agents/autodrive.md`](../agents/autodrive.md)).
