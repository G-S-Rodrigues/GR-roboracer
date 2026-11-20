
launch dev container:
```
docker compose -f docker/docker-compose.yaml up -d dev
```

inside build:
```
source /ws/setup.sh && colcon build --base-paths ros_ws/src --symlink-install
````

launch command:

```
ros2 launch racing_bringup sim_pure_pursuit.launch.py simulator:=autodrive vehicle:=config/vehicles/autodrive_roboracer.yaml scenario:=config/scenarios/porto.yaml pose_source:=slam_toolbox drive_on_estimate:=false slam_toolbox_config:=config/localization/slam_toolbox_online_1080.yaml laps:=3
```

launch simualtor in windows gui:
```
& "C:\dev\Autodrive\autodrive_simulator\AutoDRIVE Simulator.exe" -ip 172.24.67.156 -port 4567
```