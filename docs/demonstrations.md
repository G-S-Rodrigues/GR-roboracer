# Earlier demonstrations

Every video the README has shown before its current one, newest first. Each picture opens its
full recording on Google Drive.

### AMCL driving a closed-loop Spielberg lap on the generated survey map (2026-10-21)

[![AMCL driving a closed-loop Spielberg lap on the generated survey map](https://drive.google.com/thumbnail?id=1PiRinnzqWcYsrCz1xnA7K2JKh43zbPqP&sz=w640)](https://drive.google.com/file/d/1PiRinnzqWcYsrCz1xnA7K2JKh43zbPqP/view)

▶ [Watch the full video (Google Drive)](https://drive.google.com/file/d/1PiRinnzqWcYsrCz1xnA7K2JKh43zbPqP/view), 3.9x speed.
Recorded with `ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml pose_source:=amcl drive_on_estimate:=true seed:=42` on `config/scenarios/spielberg.yaml`, seed 42, at commit `cadfdd5`. AMCL on the saved survey map, pure pursuit and the safety supervisor; the vehicle drives on its own estimate. Replaced by the minimum-curvature raceline lap on ground truth.
