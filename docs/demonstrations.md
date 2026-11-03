# Earlier demonstrations

Every video the README has shown before its current one, and the recordings of capabilities the
README's headline does not show, newest first. Each picture opens its full recording on Google
Drive.

### The minimum-time raceline on ground-truth pose (2026-11-02)

[![The minimum-time raceline on ground-truth pose](https://drive.google.com/thumbnail?id=1yQtf6ssjfyGWIe4fDTwzev8XOPWl0VPe&sz=w640)](https://drive.google.com/file/d/1yQtf6ssjfyGWIe4fDTwzev8XOPWl0VPe/view)

▶ [Watch the full video (Google Drive)](https://drive.google.com/file/d/1yQtf6ssjfyGWIe4fDTwzev8XOPWl0VPe/view), 4.0x speed.
Recorded with `ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml planner:=optimized_raceline raceline_optimizer:=tum_mintime speed_source:=trajectory seed:=42` on `config/scenarios/spielberg.yaml`, seed 42, at commit `c86821a`. The `tum_mintime` raceline with pure pursuit and the safety supervisor; the vehicle drives on ground-truth pose, so no occupancy map is loaded. The lap is shown for the minimum-time line; the README's headline video stays the minimum-curvature line driven on amcl's estimate.

### The minimum-curvature raceline on ground-truth pose (2026-10-28)

[![The minimum-curvature raceline on ground-truth pose](https://drive.google.com/thumbnail?id=1jyuSPOSun9hCTo3B2k7AFhTbSG5BWP-h&sz=w640)](https://drive.google.com/file/d/1jyuSPOSun9hCTo3B2k7AFhTbSG5BWP-h/view)

▶ [Watch the full video (Google Drive)](https://drive.google.com/file/d/1jyuSPOSun9hCTo3B2k7AFhTbSG5BWP-h/view), 4.0x speed.
Recorded with `ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml planner:=optimized_raceline raceline_optimizer:=tum_mincurv_iqp speed_source:=trajectory seed:=42` on `config/scenarios/spielberg.yaml`, seed 42, at commit `abb22f2`. The `tum_mincurv_iqp` raceline with pure pursuit and the safety supervisor; the vehicle drives on ground-truth pose, so no occupancy map is loaded. Replaced by the same raceline driven on amcl's estimate over the survey map.

### AMCL driving a closed-loop Spielberg lap on the generated survey map (2026-10-21)

[![AMCL driving a closed-loop Spielberg lap on the generated survey map](https://drive.google.com/thumbnail?id=1PiRinnzqWcYsrCz1xnA7K2JKh43zbPqP&sz=w640)](https://drive.google.com/file/d/1PiRinnzqWcYsrCz1xnA7K2JKh43zbPqP/view)

▶ [Watch the full video (Google Drive)](https://drive.google.com/file/d/1PiRinnzqWcYsrCz1xnA7K2JKh43zbPqP/view), 3.9x speed.
Recorded with `ros2 launch racing_bringup sim_pure_pursuit.launch.py scenario:=config/scenarios/spielberg.yaml pose_source:=amcl drive_on_estimate:=true seed:=42` on `config/scenarios/spielberg.yaml`, seed 42, at commit `cadfdd5`. AMCL on the saved survey map, pure pursuit and the safety supervisor; the vehicle drives on its own estimate. Replaced by the minimum-curvature raceline lap on ground truth.
