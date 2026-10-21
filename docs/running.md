# Running the stack

## Host setup

First-run setup, with the clone, image build and launch, is in [Quick Start](quick-start.md). Host
requirements, GPU and display notes are in [Setup](setup.md#host-requirements).

## Running a scenario

The full graph, with RViz. Run it from `/ws` inside the container, after the build in
[Quick Start](quick-start.md):

```bash
cd /ws
source /ws/setup.sh
ros2 launch racing_bringup sim_pure_pursuit.launch.py seed:=42
```

In RViz, `base_link` is the car's centre of gravity at height h (0.074 m); `rear_axle` is its TF child,
0.17145 m behind, and the drawing hangs off it.

Headless, printing the run's metrics — this is what the nightly sweep drives:

```bash
./scripts/run_scenario.py --seed 42 --output log/scenario-seed-42.json
```

Add `--vehicle config/vehicles/f1tenth_gym_limits.yaml` to run another vehicle file (the launch's
`vehicle:=`; default `config/vehicles/f1tenth_default.yaml`).

Compare any two metrics records field by field, with the tolerances the tests use:

```bash
python3 scripts/compare_metrics.py tests/golden/baseline.json log/scenario-seed-42.json
```

Regenerating the golden baseline is a **reviewed diff**, run in `fast-sim`, and never a response to a
failing run:

```bash
# tests/golden/baseline.json (analytic_circle, SIM-3040)
docker compose -f docker/docker-compose.yaml run --rm -T fast-sim python -m sim.rollout \
  --track config/tracks/analytic_circle.yaml --maps-root config/scenarios/maps \
  --scenario baseline --seed 42 --max-steps 9000 \
  --image-digest "$(docker image inspect --format '{{.Id}}' gr-roboracer:fast-sim)" \
  --vehicle-parameter-version f1tenth-default-v1 --lookahead-distance 1.0 \
  --minimum-speed 0.5 --maximum-speed 3.0 --curvature-speed-gain 1.0 \
  --output tests/golden/baseline.json

# tests/golden/spielberg_baseline.json (SIM-3090): the same flags with
#   --track config/tracks/spielberg.yaml --scenario spielberg --max-steps 30000
```

The controller flags are `config/vehicles/f1tenth_default.yaml`'s values; `python sim/rollout.py`
fails with `ModuleNotFoundError: sim`, so run it as a module. It regenerates the gym map assets
under `config/scenarios/maps` from the canonical track, byte-identically. The live scenario must
not set physics parameters the rollout does not (SIMJAX-1060).

## When a run misbehaves

Symptoms and fixes (latched stop, silent topics, stale launches) are in
[Setup](setup.md#troubleshooting).
