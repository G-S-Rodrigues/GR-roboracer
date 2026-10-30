"""The server's vehicle parameters for tests: one source (not a test).

Read by `_server_launch.py` (the tier-2 server tests) and by
`racing_sim_adapter/test/conformance.py` (the second registration), so the
two cannot disagree. lr, wheel radius and the throttle gain come from part
4's files, not copies (plan Step 0). h and the steering limit are not single
keys there (h is the rest height plus com_z; the file's 0.5236 is the guide's
rounding of the prefab's 0.523633), so they stay the anchored part-4
numbers; kp/ki are UNTUNED (tuning is part 10).
"""

from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[4]
VEHICLE_DIR = REPO / "config" / "vehicles" / "autodrive_roboracer"

H = 0.07354
MAX_STEERING_ANGLE = 0.523633
KP = 0.02  # untuned
KI = 0.05  # untuned


def vehicle() -> dict:
    source = yaml.safe_load((VEHICLE_DIR / "source.yaml").read_text())
    identified = yaml.safe_load((VEHICLE_DIR / "identified.yaml").read_text())
    values = source["values"]
    return {
        "lr": float(values["com_x_lr_m"]["value"]),
        "h": H,
        "wheel_radius": float(values["wheel_radius_m"]["value"]),
        "throttle_gain": float(
            identified["measurements"]["throttle_gain"]["value"]
        ),
        "max_steering_angle": MAX_STEERING_ANGLE,
        "kp": KP,
        "ki": KI,
    }
