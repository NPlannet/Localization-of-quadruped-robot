# XGO Mini2 Robot Deployment

The physical robot uses a privileged, host-networked ROS 2 Jazzy container.
Host UART, camera, and timing prerequisites are listed separately in
[`raspberry-pi-setup.md`](raspberry-pi-setup.md).

## Sync Files from the laptop

From `robot_sim_sose/`:

```bash
bash scripts/sync_robot_runtime.sh pi@robodoge1.local
```

## Build and enter the container

On the robot:

```bash
cd ~/robot_sim_sose
docker compose -f docker-compose.robot.yml build
docker compose -f docker-compose.robot.yml up -d
docker compose -f docker-compose.robot.yml exec xgo-robot bash
bash scripts/robot_build_workspace.sh
```


## Unified sensor bringup

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py
```

Defaults start the XGO bridge, LD19, camera, dynamic filter, and Foxglove
Bridge. No SLAM method is selected by default. `enable_motion` permits the
bridge to apply `/cmd_vel`; the launch itself does not command movement.


## Select a mapper

The same bringup can start any supported mapper:

```bash
slam_method:= slam_toolbox , cartographer , rtabmap
slam_scan_topic:=/scan , /scan_filtered
```

```bash
# Cartographer with raw LiDAR, external odometry, and IMU
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true slam_method:=cartographer slam_scan_topic:=/scan_filtered

# RTAB-Map with filtered LiDAR and RGB loop recognition
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true slam_method:=rtabmap \
  slam_scan_topic:=/scan_filtered rtabmap_use_camera:=true \
  rtabmap_delete_database_on_start:=true

# For a compute-oriented sensor run without camera or visualization:

ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true start_camera:=false start_foxglove:=false \
  start_dynamic_filter:=false slam_method:=none
```

Use `/scan` for a raw trial. Use `/scan_filtered` only when
`start_dynamic_filter:=true`.


## Record runs, CPU use, and battery

When bringup is already running, attach the standalone recorder:

```bash
bash scripts/record_robot_run.sh rtab_filtered_run_wp_1
```

Press `Ctrl+C` once to finalize the MCAP, CPU/RAM summary, and battery summary.



## Copy a run to the laptop from the laptop shell:

```bash
scp -r pi@robodoge1.local:~/robot_sim_sose/evaluation/runs/rtab_filtered_run_wp_3 \
  robot_sim_sose/evaluation/runs/
```


##  Prepare Waypoints.JSON
before you can use the recorded bag for testing , you have to label the timestamps that were recorded under /ground_truth/waypoint
For that you have to log into the robot and run 

Terminal 1
```bash
ros2 topic echo \
  /ground_truth/waypoint \
  std_msgs/msg/String \
  --field data \
  | tee /workspaces/robot_sim_sose/evaluation/runs/rtab_filtered_run_wp_1/waypoint_timestamps.txt
```
creates a txt file containing the timestamps of the waypoints
  Terminal 2
```bash
  ros2 bag play \
  /workspaces/robot_sim_sose/evaluation/runs/rtab_filtered_run_wp_1/bag \
  --topics /ground_truth/waypoint \
  --rate 100
```


Then you can paste the txt file content together with the context below 
into e.g Chat GPT and create a waypoints.json file in the folder of the run ,
e.g evaluation/run/rtab_filtered/

Context(change order of visited waypoints):
Visited waypoints in order: (e.g 1,2,3,4,5,5,4,3,2,1)
JSON FILE FORMAT
{
  "topic": "/ground_truth/waypoint",
  "frame": "map",
  "marks": [
    {
      "stamp_ns": 1785415036091702448,
      "index": 0,
      "label": "1",
      "x": 0.0,
      "y": 0.0
    },
    {
      "stamp_ns": 1785415051234567890,
      "index": 1,
      "label": "2",
      "x": 1.0,
      "y": 0.0
    }
  ]
}
Coordinates for all Waypoints

{"stamp_ns": t,       "index": n,"label": "1","x": 0.0,"y": 0.0}
{"stamp_ns": t,       "index": n,"label": "2","x": 0.0,"y": 1.0}      
{"stamp_ns": t,       "index": n,"label": "3","x": 0.0,"y": 2.0}
{"stamp_ns": t,       "index": n,"label": "4","x": -1.0,"y": 0.0}
{"stamp_ns": t,       "index": n,"label": "5","x": -1.0,"y": 1.0}
{"stamp_ns": t,       "index": n,"label": "6","x": -1.0,"y": 2.0}
{"stamp_ns": t,       "index": n,"label": "7","x": -2.0,"y": 0.0}
{"stamp_ns": t,       "index": n,"label": "8","x": -2.0,"y": 1.0}
{"stamp_ns": t,       "index": n,"label": "9","x": -2.0,"y": 2.0}    
{"stamp_ns": t,       "index": n,"label": "10","x": -3.0,"y": 0.0}
{"stamp_ns": t,       "index": n,"label": "11","x": -3.0,"y": 1.0}
{"stamp_ns": t,       "index": n,"label": "12","x": -3.0,"y": 2.0}
{"stamp_ns": t,       "index": n,"label": "13","x": -4.0,"y": 0.0}
{"stamp_ns": t,       "index": n,"label": "14","x": -4.0,"y": 1.0}
{"stamp_ns": t,       "index": n,"label": "15","x": -4.0,"y": 2.0}



## Headless bag benchmarks on Raspberry PI

`scripts/robot_bag_benchmark.sh`

Use `scripts/robot_bag_benchmark.sh` to replay a recorded run through a fresh
SLAM pipeline on the Raspberry Pi. The script starts
resource monitoring, excludes recorded derived topics, and exits automatically
after playback. See [`evaluation.md`](evaluation.md#benchmark-recorded-bags-on-the-raspberry-pi)
for commands and the Foxglove trade-off.



## Lichtblick/Foxglove connection

The bridge listens on port `8766`. Add a Foxglove WebSocket connection in
Lichtblick using:

```text
ws://<robot-ip>:8766
```

## Save a map

While `/map` is actively being published:

```bash
bash scripts/robot_stack.sh save-map
```

The default output is `evaluation/results/live/maps/xgo_map.{yaml,pgm}`. A map
saver timeout usually means `/map` is absent, uses incompatible QoS, or the
mapper/occupancy-grid publisher has already stopped.





## Common failures

- **No complete XGO auto-feedback frame:** ensure the bridge uses
  `xgo_imu_read_mode:=orientation_registers`, verify UART host setup, and stop
  every other serial consumer.
- **Random or jumping orientation:** verify the configured robot model and
  controller firmware, then inspect the raw pitch/roll/yaw diagnostic before
  blaming SLAM.
- **RViz queue full or no map:** verify one consistent clock, the complete
  `map -> odom -> base_link -> laser` TF chain, scan timestamps, and the mapper's
  configured scan topic.
- **Camera discovered but cannot be opened:** pass the complete Raspberry Pi
  media graph, run only one camera process, and ensure the container uses the
  Raspberry Pi libcamera stack built by the robot Dockerfile.
- **Foxglove parameter timeout for the bridge:** this is often secondary to a
  blocked serial callback. Diagnose the XGO serial owner and data first.
