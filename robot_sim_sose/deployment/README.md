# XGO Mini2 Robot Deployment
Deployment-specific docs and implementation scripts now live under `deployment/`.
Public command entrypoints still stay compatible through the wrappers in `scripts/`.

Connect to the robot via the right WIFI connection and ssh pi@robodogeX.local where X is the right number. Passwords are listed in the private WhatsApp Group.

pw robo2 = sughie6Eerasha7i
## 2. Start The Robot Container

## 2. Start The Robot Container

Sync only the robot runtime files from your laptop/WSL to the robot:

```bash
cd ~/Uni/Localization-of-quadruped-robot/robot_sim_sose
bash scripts/sync_robot_runtime.sh pi@ROBOT_IP
```
From the synced repo root on the robot:

```bash
cd ~/robot_sim_sose
export ROS_DOMAIN_ID=42
 docker compose -f docker-compose.robot.yml build --no-cache
 docker compose -f docker-compose.robot.yml up -d
 docker compose -f docker-compose.robot.yml exec xgo-robot bash
```



Launch the full stack into real terminal windows:

bash scripts/robot_nav_tmux.sh pi@robodoge2.local --skip-build

Each terminal logs into the robot, enters the Docker container, starts its role, and stays usable after you stop that role with `Ctrl+C`.

On your WSL setup with `gnome-terminal`, the launcher can stay fully on the Linux side and group the roles into one terminal window with tabs:

```bash
bash scripts/robot_nav_tmux.sh pi@robodoge2.local --skip-build --terminal-app gnome-terminal --terminal-layout tabs
```

From WSL, the launcher can open a single Windows Terminal window instead:

```bash
bash scripts/robot_nav_tmux.sh pi@robodoge2.local --skip-build --terminal-app windows-terminal --terminal-layout panes
```

If you prefer one window with tabs instead of split panes:

```bash
bash scripts/robot_nav_tmux.sh pi@robodoge2.local --skip-build --terminal-app windows-terminal --terminal-layout tabs
```

The real robot and bag replay now share the same dynamic filter tuning file:

```text
src/xgo_description/config/dynamic_scan_filter_bag_replay.yaml
```

Lichtblick is intended to use the compressed camera topic, not the raw one:

```text
/camera/image_raw/compressed
```

The deployment Foxglove bridge is now configured to expose the compressed camera stream and not the raw image topic.


Inside the container:

```bash
bash scripts/robot_build_workspace.sh
source install/setup.bash
```

## 3. Verify Robot Topics

Inside the container:

```bash
bash scripts/robot_stack.sh check
```

Then carefully test motion with enough free space around the robot:

```bash
bash scripts/robot_stack.sh xgo-motion
```

Leave that running in its own shell. In another shell:

```bash
bash scripts/robot_stack.sh tiny-forward
bash scripts/robot_stack.sh stop
```

## 4. Run Mapping And Navigation

Use separate container shells:

```bash
docker compose -f docker-compose.robot.yml exec xgo-robot bash
```

Terminal 1:

```bash

```

This starts the XGO SDK bridge with motion disabled. It should publish `/imu`, `/battery_state`, `/odom`, and TF from `odom` to `base_link`.

Only after verifying the bridge topics, restart it with motion enabled:

```bash
bash scripts/robot_stack.sh xgo-motion
bash scripts/robot_stack.sh xgo-bridge (motion disabled)
```

Terminal 2:

```bash
bash scripts/robot_stack.sh lidar
```

Terminal 3:

```bash
bash scripts/robot_stack.sh camera
```

Terminal 4:

```bash
bash scripts/robot_stack.sh filter
```

Terminal 5:

```bash
bash scripts/robot_stack.sh slam
```

Terminal 6:

```bash
bash scripts/robot_stack.sh nav2
```

Terminal 7, after manual Nav2 goals work:

```bash
bash scripts/robot_stack.sh explore
```

## 5. Lichtblick Visualization

Terminal 5:

```bash
bash scripts/robot_stack.sh foxglove
```

Connect Lichtblick from your laptop to:

```text
ws://robodoge2.local:8766 (With the right number)
```

In Lichtblick, add an `Image` panel and set:

```text
Image topic: /camera/image_raw/compressed
Camera info: /camera/camera_info
```

The deployment bridge intentionally exposes the compressed topic and hides `/camera/image_raw` to avoid unnecessary bandwidth and visualization delay.

If direct access fails:

```bash
ssh -L 8766:localhost:8766 pi@ROBOT_IP
```

Then connect Lichtblick to:

```text
ws://localhost:8766
```

## 6. Save A Map

```bash
bash scripts/robot_stack.sh save-map
```

The map is written to:

```text
maps/xgo_map.yaml
maps/xgo_map.pgm
```


Notes:

## 1. Robot Hardware Interface

The required real-robot ROS interface is:

```text
/scan       LaserScan from the XGO 360 ToF/LiDAR
/odom       Odometry
/tf         odom -> base_link -> lidar frame
/cmd_vel    velocity command input
```

The current robot setup uses:

```text
/dev/ttyS0    XGO body controller through xgolib
/dev/ttyUSB0  LD19/LDROBOT LiDAR through ldlidar_stl_ros2
/dev/video*   Camera through camera_ros
```

### Driver Installation Rule

You can include user-space robot SDKs in `.devcontainer/Dockerfile.robot`, especially Python packages such as an XGO SDK. Keep host-level hardware setup on the robot OS:

```text
Good Dockerfile candidates:
  Python SDK packages
  ROS 2 hardware bridge node
  serial/I2C/SPI Python libraries

Keep on host:
  kernel modules
  Raspberry Pi overlays
  firmware
  udev rules unless explicitly mounted/applied
  vendor services that must start before Docker
```

The robot container installs `xgolib` and common serial/GPIO/I2C helper packages by default. The XGO body controller responds on `/dev/ttyS0`. The CP2102 adapter at `/dev/ttyUSB0` is the expected LD19/LDROBOT LiDAR path. Start it with:

```bash
bash scripts/robot_stack.sh lidar
```

This wraps:

```bash
ros2 launch ldlidar_stl_ros2 ld19.launch.py serial_port:=/dev/ttyUSB0
```

The Dockerfile tries to install `ros-jazzy-ldlidar-stl-ros2` when it is available from the ROS package repositories. If that package is not available for your base image, add the `ldlidar_stl_ros2` package to `src/ldlidar_stl_ros2` in this workspace and rerun:

```bash
bash scripts/robot_build_workspace.sh
```

That script now checks that `ldlidar_stl_ros2` is available either from apt or from the workspace before you continue with the robot stack.

The camera can be started with:

```bash
bash scripts/robot_stack.sh camera
```

This now uses the deployment camera config:

```text
deployment/config/camera_ros_robot.yaml
```

The current robot camera settings are:

```text
camera="0"
format="BGR888"
width=640
height=480
```

That matches what we found on the robot:

- The package accepted the default camera selector path, but rejected the short name `ov5647`.
- The repo's camera consumers convert `/camera/image_raw` to `bgr8`, so `BGR888` is the safest explicit format.
- `640x480` is a better default than the previous auto-selected `800x600` for bandwidth and visualization delay.

You may still see one calibration warning until you provide a real camera calibration file. That warning is expected and does not mean the image stream itself failed to start.

For Lichtblick, use `/camera/image_raw/compressed`.
