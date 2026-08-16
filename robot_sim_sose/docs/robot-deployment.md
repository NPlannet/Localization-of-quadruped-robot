# XGO Mini2 Robot Deployment

The robot uses a privileged, host-networked ROS 2 Jazzy container. Configure
UART and camera access first using [Raspberry Pi host setup](raspberry-pi-setup.md).

## Sync and build

From `robot_sim_sose/` on the laptop:

```bash
bash scripts/sync_robot_runtime.sh pi@ROBOT_HOST
```

On the robot:

```bash
cd ~/robot_sim_sose
docker compose -f docker-compose.robot.yml up -d --build
docker compose -f docker-compose.robot.yml exec xgo-robot bash
bash scripts/robot_build_workspace.sh
source install/setup.bash
```

In every new container shell, source the workspace if it was not sourced
automatically:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Start and verify sensors

Start without allowing motion for the first check:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=false slam_method:=none
```

This starts the XGO bridge, LD19, camera, dynamic filter, waypoint marker, and
Foxglove bridge. It publishes `/scan`, `/scan_filtered`, `/imu/data`, `/odom`,
`/battery_state`, camera topics, and TF.

Verify the stack in a second container shell:

```bash
bash scripts/robot_stack.sh check
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo odom base_link
```

Rotate the stationary robot and confirm that IMU yaw changes smoothly before
mapping.

## Select SLAM and scan input

Set `slam_method` to `slam_toolbox`, `cartographer`, or `rtabmap`. Select
`/scan` for raw input or `/scan_filtered` for the dynamic filter.

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true \
  slam_method:=slam_toolbox \
  slam_scan_topic:=/scan_filtered
```

For RTAB-Map RGB loop recognition, keep the camera enabled and add:

```text
rtabmap_use_camera:=true rtabmap_delete_database_on_start:=true
```

Deleting the RTAB-Map database at startup is appropriate for independent
evaluation runs, not for continuing an existing map.

The launch file never commands movement by itself. With
`enable_motion:=true`, it only allows incoming `/cmd_vel` messages to reach the
robot. Use `bash scripts/robot_stack.sh stop` for an explicit zero command.

## Lichtblick/Foxglove

Connect Lichtblick to:

```text
ws://<robot-ip>:8766
```

Use `/camera/image_raw/compressed` with `/camera/camera_info` for the camera
panel. The topic allowlist is in
`src/xgo_driver_bridge/config/foxglove_bridge_robot.yaml`.

## Record and copy a run

With bringup already running:

```bash
bash scripts/record_robot_run.sh RUN_NAME
```

Press `Ctrl+C` once to finalize the bag, resource summary, and battery summary.
Copy it from the laptop with:

```bash
scp -r pi@ROBOT_HOST:~/robot_sim_sose/evaluation/runs/RUN_NAME \
  evaluation/runs/
```

Waypoint labeling and offline benchmarks are described in
[the evaluation guide](evaluation.md).

## Save a map

While `/map` is being published:

```bash
bash scripts/robot_stack.sh save-map
```

The default output is `evaluation/results/live/maps/xgo_map.{yaml,pgm}`.

## Common failures

- **No XGO data:** stop other users of `/dev/ttyAMA0` and verify the host UART.
- **No map:** check the selected scan topic and the complete
  `map -> odom -> base_link -> laser` transform chain.
- **Cartographer drops earlier points:** inspect LaserScan timestamps and enable
  scan normalization for affected legacy bags.
- **No camera:** ensure only one camera process runs and all Pi media devices
  are available inside the container.
- **Wrong robot receives commands:** verify robot IP, `ROS_DOMAIN_ID`, Foxglove
  connection, and `/cmd_vel` subscribers before enabling motion.
