# XGO Mini2 Robot Deployment

The physical robot uses a privileged, host-networked ROS 2 Jazzy container.
Host UART, camera, and timing prerequisites are listed separately in
[`raspberry-pi-setup.md`](raspberry-pi-setup.md).

## Sync from the laptop

From `robot_sim_sose/`:

```bash
bash scripts/sync_robot_runtime.sh pi@robodoge1.local
```

The sync deliberately copies runtime source/configuration, scripts, Docker
files, and robot documentation. It does not copy bags, evaluation results, or
colcon build products.

## Build and enter the container

On the robot:

```bash
cd ~/robot_sim_sose
docker compose -f docker-compose.robot.yml build
docker compose -f docker-compose.robot.yml up -d
docker compose -f docker-compose.robot.yml exec xgo-robot bash
bash scripts/robot_build_workspace.sh
```

The Compose service is `xgo-robot`; the actual container name is
`xgo_mini2_robot`. To enter by container name instead, use:

```bash
docker exec -it xgo_mini2_robot bash
```

The source workspace is bind-mounted from the host. File edits survive
container removal. Packages installed interactively with `apt` survive a stop
and restart of the same container, but are lost when it is recreated. The
Dockerfile already installs SLAM Toolbox, Cartographer, RTAB-Map, camera_ros,
Foxglove Bridge, rosbag2/MCAP, and the official `xgolib` version used by this
project; rebuild the image for reproducible deployment.

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

For deliberate teleoperation without mapping:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true slam_method:=none
```

For a compute-oriented sensor run without camera or visualization:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true start_camera:=false start_foxglove:=false \
  start_dynamic_filter:=false slam_method:=none
```

Never run a second XGO bridge at the same time. Only one process can own
`/dev/ttyAMA0`, and two `/cmd_vel` consumers make motion behavior unsafe.

## Select a mapper

The same bringup can start any supported mapper:

```bash
# SLAM Toolbox with filtered LiDAR
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true slam_method:=slam_toolbox \
  slam_scan_topic:=/scan_filtered

# Cartographer with raw LiDAR, external odometry, and IMU
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true slam_method:=cartographer slam_scan_topic:=/scan

# RTAB-Map with filtered LiDAR and RGB loop recognition
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true slam_method:=rtabmap \
  slam_scan_topic:=/scan_filtered rtabmap_use_camera:=true \
  rtabmap_delete_database_on_start:=true
```

Use `/scan` for a raw trial. Use `/scan_filtered` only when
`start_dynamic_filter:=true`. Live mapper configuration belongs in
`src/xgo_driver_bridge/config/`; similarly named files in `xgo_description`
are for simulation or replay.

Cartographer uses `/odom` as a local motion prior and `/imu/data` for angular
orientation/rate constraints according to its Lua configuration. RTAB-Map can
use images for visual place recognition, but still needs valid camera
calibration and synchronized image/CameraInfo messages.

## What the XGO bridge publishes

The Mini2 controller is opened through the official Python `xgolib` on
`/dev/ttyAMA0`. The bridge reads pitch, roll, and yaw orientation registers
rather than relying on the controller's incomplete auto-feedback frames. It
publishes:

- `/imu/data`: orientation quaternion derived from those angles;
- `/xgo/yaw_deg`: diagnostic heading;
- `/xgo/applied_vel`: velocity actually accepted by the bridge;
- `/odom` and `odom -> base_link`: position integrated from applied velocity,
  with orientation corrected from the controller reading;
- `/battery_state`; and
- optional `/cmd_vel` motion control.

This is command-integrated odometry, not measured leg/wheel displacement. Slip,
foot impacts, uncommanded motion, and translation while carried are not directly
observed. SLAM scan matching corrects some accumulated error through
`map -> odom`; it does not turn `/odom` into ground truth.

## Verify before walking

```bash
bash scripts/robot_stack.sh check
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /cmd_vel -v
```

Rotate the robot and confirm that yaw changes smoothly and in the expected
direction. Then issue a stop and, only with space around the robot, the provided
very slow motion test:

```bash
bash scripts/robot_stack.sh stop
bash scripts/robot_stack.sh tiny-forward
```

## Lichtblick/Foxglove connection

The bridge listens on port `8766`. Add a Foxglove WebSocket connection in
Lichtblick using:

```text
ws://<robot-ip>:8766
```

The allowlist in `src/xgo_driver_bridge/config/foxglove_bridge_robot.yaml`
limits published/control topics, including `/cmd_vel`. Use a distinct ROS
domain and robot IP to avoid connecting teleoperation panels to another robot.

For the camera panel, select `/camera/image_raw/compressed` as the image and
`/camera/camera_info` as calibration. A missing calibration YAML warning from
camera_ros does not by itself prevent images; the node should still publish a
CameraInfo message with the configured 640x480 dimensions. “Invalid image size
0x0” means Lichtblick has not received a matching CameraInfo message. Check:

```bash
ros2 topic hz /camera/image_raw/compressed
ros2 topic echo /camera/camera_info --once
```

## Record runs and CPU use

When bringup is already running, attach the standalone recorder:

```bash
bash scripts/record_robot_run.sh sensor_only_run1
```

For a script-owned mapper trial:

```bash
bash scripts/robot_evaluation_run.sh cartographer filtered cart_filtered_run1
```

Press `Ctrl+C` once to finalize the MCAP and resource summary. Results are under
`evaluation/runs/<run_name>/`. See [`evaluation.md`](evaluation.md) for the
recorded topics and experimental protocol.

Copy a run to the laptop from the laptop shell:

```bash
scp -r pi@robodoge1.local:~/robot_sim_sose/evaluation/runs/sensor_only_run1 \
  robot_sim_sose/evaluation/runs/
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
