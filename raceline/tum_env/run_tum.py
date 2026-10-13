"""Run ForzaETH's trajectory_optimizer in the isolated py3.8 environment.

argv: track_csv inputs_dir curv_opt_type safety_width out_csv
Python 3.8 syntax only. Writes the 7-column trajectory to out_csv.
"""

import os
import sys

import numpy as np


def main(argv):
    track_csv, inputs_dir, curv_opt_type, safety_width, out_csv = argv
    # Headless: TUM imports pyplot at module load.
    os.environ.setdefault("MPLBACKEND", "Agg")
    from global_racetrajectory_optimization.trajectory_optimizer import (
        trajectory_optimizer,
    )

    track_name = os.path.splitext(os.path.abspath(track_csv))[0]
    traj, _, _, _ = trajectory_optimizer(
        os.path.abspath(inputs_dir),
        track_name,
        curv_opt_type,
        safety_width=float(safety_width),
        plot=False,
    )
    np.savetxt(out_csv, traj, delimiter=";")


if __name__ == "__main__":
    main(sys.argv[1:])
