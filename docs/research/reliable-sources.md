# Reliable sources

The sources this stack trusts, and what it trusts each one *for*. An entry earns a place by being
**competition-proven or peer-reviewed, and inspectable** (open code, or a paper with enough detail to
reimplement). A source reliable for planning is not automatically reliable for localization. The
[2026 survey](autonomous-racing-open-source-2026.md) covers the wider ecosystem.

`Checked` columns were read from the GitHub API or the page on the date shown; re-check before
depending on one. "Not found" means searched and absent, not proven absent.

## Tier A: primary references (proven on track + open code)

| Source | Institution | Trust it for | Licence | Checked 2026-09-30 |
|---|---|---|---|---|
| [TUMFTM/global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) | TUM, Institute of Automotive Technology | Offline raceline: shortest path, min-curvature (+iterative), min-time (CasADi/IPOPT), powertrain-aware. Output columns = our `TrajectoryPoint` | LGPL-3.0 | last push 2023-07-06; Python 3.7 / Ubuntu 20.04 era pins |
| [TUMFTM/trajectory_planning_helpers](https://github.com/TUMFTM/trajectory_planning_helpers) | TUM | The library the above is built on: `opt_min_curv`, `iqp_handler`, spline tools, GGV velocity profile. On PyPI | LGPL-3.0 | PyPI 0.79; last push 2024-03-25 |
| [TUMFTM/laptime-simulation](https://github.com/TUMFTM/laptime-simulation) | TUM | Lap-time simulation, for checking a speed profile is feasible | LGPL-3.0 | last push 2026-04-13 |
| [TUMFTM/GraphBasedLocalTrajectoryPlanner](https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner) | TUM | Local planning / overtaking action sets (Stage 4–5) | see repo | not re-checked |
| [TUMFTM/Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics) | TUM | Vehicle dynamics modelling (full scale, validated on AV-21) | see repo | not re-checked |
| TUM Autonomous Motorsport, [A2RL head-to-head paper](https://arxiv.org/abs/2602.08571) | TUM | How the inaugural A2RL winner built multi-agent racing at the handling limit. **No code link on the page** | — | arXiv 2602.08571 |
| [ForzaETH/race_stack](https://github.com/ForzaETH/race_stack) | ETH Zürich, PBL | Full F1TENTH competition stack: state machine, overtaking, sysid, and **`global_planner`: occupancy grid → centerline + widths → TUM optimizer** (the unseen-track pipeline) | `global_planner` MIT; stack-level: check per package | `main` pushed 2026-09-29; `ros2-jazzy` branch 2025-08-08; also `ros2-humble` |
| [f1tenth/f1tenth_gym_jax](https://github.com/f1tenth/f1tenth_gym_jax) | F1TENTH / RoboRacer (official) | Our primary simulator; later the vectorized RL backend | see repo | in use |
| [ForzaETH/global_racetrajectory_optimization@pip_package](https://github.com/ForzaETH/global_racetrajectory_optimization/tree/pip_package) | ETH (fork of TUM) | The **pip-installable** TUM optimizer ForzaETH's `global_planner` depends on (git submodule). Same optimizer core as TUM | LGPL-3.0 | last push 2024-04-11 |
| [f1tenth/f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | F1TENTH (official) | Track geometry + a TUM-generated reference raceline per track (a cross-check, not our vehicle's line) | GPL-3.0 | pinned @866fe65 |

## Tier B: strong, narrower, or less active

| Source | Institution | Trust it for | Licence | Checked 2026-09-30 |
|---|---|---|---|---|
| [HMCL-UNIST/unicorn-racing-stack](https://github.com/HMCL-UNIST/unicorn-racing-stack) | UNIST (Korea) | Clean ROS 2 Jazzy package split, arm64/Jetson deployment | none declared | last push 2026-07-06 |
| [CL2-UWaterloo/Raceline-Optimization](https://github.com/CL2-UWaterloo/Raceline-Optimization) | U. Waterloo | TUM fork for F1TENTH, with a `slam_toolbox` map → track notebook (manual cleanup step) | LGPL-3.0 | last push 2023-12-08; pins a `trajectory_planning_helpers` fork because of `quadprog` |
| [alexliniger/MPCC](https://github.com/alexliniger/MPCC) | ETH (origin) | The standard open MPCC formulation (Stage 3 controller) | see repo | not re-checked |
| [jainachin/bayesrace](https://github.com/jainachin/bayesrace) | ETH / UPenn (Jain & Morari) | Racing line by Bayesian optimization, plus learned-dynamics MPC; useful for learning, not a production path | MIT | last push 2024-06-17 |
| [BDEvan5/f1tenth_benchmarks](https://github.com/BDEvan5/f1tenth_benchmarks) | — | Benchmark method for classical vs RL on F1TENTH | see repo | not re-checked |
| [TC-Driver paper](https://f1tenth.org/publications/TC_Driver.pdf) | ETH PBL | Trajectory-conditioned RL: the bridge from a classical raceline to RL | — | — |
| [RoboRacer Arena](https://arxiv.org/abs/2608.23040) | Clement, Poks, Bartocci | Isaac Sim tracks generated from occupancy maps; 256 parallel vehicles. Submitted to ICRA 2027; no code link found | CC BY 4.0 paper | 2026-08 |
| [AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing) | AutoDRIVE Ecosystem | League simulator + ROS 2 bridge | BSD-2-Clause | last push 2026-09-15; devkit `FROM ros:humble` |

## Abu Dhabi: A2RL ecosystem (TII: reliable, per owner 2026-09-30)

Full-scale (Dallara SF23) racing. It transfers architecture and discipline, not vehicle parameters
([survey](autonomous-racing-open-source-2026.md), Stage 8). **TII (Technology Innovation Institute) is a reliable source**: it builds the
identical base stack every A2RL team receives, and ran TII EuroRacing in the IAC. Its car-racing
*code* is not public, so trust it for **papers and datasets**, not for code to depend on.

| Source | Institution | What is open | Checked 2026-09-30 |
|---|---|---|---|
| [er.autopilot 1.0](https://arxiv.org/abs/2310.18112) | TII EuroRacing (IAC) | Paper: full oval-racing stack: architecture, localization, planning, control | paper |
| [A2RL Vmax dataset](https://arxiv.org/abs/2607.17813) | A2RL / TII | High-speed LiDAR perception dataset + baseline code | dataset + baselines |
| [github.com/tii-racing](https://github.com/tii-racing) | TII Racing | 3 public repos, all **drone** racing: `drone-racing-dataset` (MIT, 2026-01), `drone-racing-msgs` (MIT), `awesome-robotic-tooling` (CC0, a curated tooling list) | no car-racing code |
| [A2RL](https://a2rl.io/autonomous-car-race) | ASPIRE (ATRC, Abu Dhabi) | League rules, news | — |
| [TUM A2RL head-to-head paper](https://arxiv.org/abs/2602.08571) | TUM (A2RL season 1 winner) | Paper; no code link | paper |
| [Minimalistic stack for high-speed time-trial racing](https://arxiv.org/abs/2509.19636) | Ali, Jardali, Yu, Pushp, Liu (affiliation not on abstract page) | Paper + data on Zenodo; no code found | paper + data |
| Khalifa University ("Fly Eagle", with BIT) | Khalifa University | **Not found**: no open stack or paper located | — |

## Not reliable enough to cite (and why)

- Blog notes and single-author forks with no track results: fine for pointers, never as evidence.
- `vaul-ulaval/f1tenth-raceline-optim`: **archived** 2023-12.
