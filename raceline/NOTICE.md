# Third-party material in `raceline/`

## `tests/fixtures/tum_inputs/`

`racecar_f110.ini`, `veh_dyn_info/ggv.csv` and `veh_dyn_info/ax_max_machines.csv`
are copied unmodified from ForzaETH/race_stack at commit
`202450f` (`stack_master/config/global_planner/`), MIT licence
(`stack_master/LICENSE` at that commit). They are the vehicle inputs TUM's
`trajectory_optimizer` reads from its `input_path`; they are test fixtures,
not this stack's vehicle (raceline part 3 owns that).

## Installed, not vendored

`ForzaETH/global_racetrajectory_optimization` @ `b1b38ec` (LGPL-3.0) and
`trajectory_planning_helpers` (LGPL-3.0) are installed into `tum_env/`'s
virtual environment, not copied into this repository.
