"""Run the fork's minimum-time trajectory_optimizer in the py3.8 environment.

argv: track_csv inputs_dir safety_width seed_csv export_dir options_json out_csv
Python 3.8 syntax only. ``options_json`` is a JSON object of the fork's
``MintimeOptions`` fields; ``seed_csv`` (``s;x;y;psi;kappa;v;a_x``) becomes its
``seed_line``. Writes the 7-column trajectory to out_csv and the fork's own
``<export_dir>/mintime/solver_status.json``. On ``MintimeNotConverged`` prints
one ``MINTIME_NOT_CONVERGED {json}`` line and exits 1.
"""

import json
import os
import sys

import numpy as np


def main(argv):
    track_csv, inputs_dir, safety_width, seed_csv, export_dir = argv[:5]
    options_json, out_csv = argv[5:]
    # Headless: TUM imports pyplot at module load.
    os.environ.setdefault("MPLBACKEND", "Agg")
    from global_racetrajectory_optimization.mintime_options import (
        MintimeNotConverged,
        MintimeOptions,
    )
    from global_racetrajectory_optimization.trajectory_optimizer import (
        trajectory_optimizer,
    )

    fields = json.loads(options_json)
    fields["seed_line"] = np.loadtxt(seed_csv, delimiter=";")
    track_name = os.path.splitext(os.path.abspath(track_csv))[0]
    try:
        traj, _, _, _ = trajectory_optimizer(
            os.path.abspath(inputs_dir),
            track_name,
            "mintime",
            safety_width=float(safety_width),
            plot=False,
            export_dir=os.path.abspath(export_dir),
            mintime=MintimeOptions(**fields),
        )
    except MintimeNotConverged as err:
        print(
            "MINTIME_NOT_CONVERGED "
            + json.dumps(
                {
                    "status": err.status,
                    "iterations": err.iterations,
                    "state_dir": err.state_dir,
                }
            )
        )
        sys.exit(1)
    np.savetxt(out_csv, traj, delimiter=";")


if __name__ == "__main__":
    main(sys.argv[1:])
