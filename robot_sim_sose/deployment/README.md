# XGO Mini2 Robot Deployment
Deployment-specific docs and implementation scripts now live under `deployment/`.
Public command entrypoints still stay compatible through the wrappers in `scripts/`.

Connect to the robot via the right WIFI connection and ssh pi@robodogeX.local where X is the right number. Passwords are listed in the private WhatsApp Group.

pw robo1 = AeJoh9oole5Eenu2
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
export ROS_DOMAIN_ID=60
 docker compose -f docker-compose.robot.yml build --no-cache
 docker compose -f docker-compose.robot.yml up -d
docker compose -f docker-compose.robot.yml exec xgo-robot bash
```


```

The live robot uses a dedicated dynamic-filter configuration:

```text
src/xgo_driver_bridge/config/dynamic_scan_filter_robot.yaml
```

Unlike bag replay, the live filter consumes `/scan` directly. The offline scan
normalizer is only needed to repair irregular timing and beam counts in the
recorded W1 data.

Lichtblick uses the compressed camera topic:

```text
/camera/image_raw/compressed
```
Inside the container:

```bash
bash scripts/robot_build_workspace.sh
source install/setup.bash
```

## 3. Start All Evaluation Publishers

After building and sourcing the workspace, start the complete sensor input
stack in one container terminal:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py
```

The compatibility wrapper starts the same launch:

```bash
bash scripts/robot_stack.sh sensors
```

This single process group starts:

- the XGO bridge (`/imu/data`, `/battery_state`, `/odom`,
  `/xgo/applied_vel`, and `/tf`);
- the LD19 driver (`/scan` and the LiDAR static transform);
- `camera_ros` (`/camera/image_raw`, compressed image, and camera info);
- the dynamic filter (`/scan` to `/scan_filtered`);
- the camera optical-frame transform; and
- Foxglove Bridge on port `8766` for Lichtblick.

It can also start exactly one mapping algorithm. Sensor-only operation remains
the default (`slam_method:=none`). Select a mapper with:

```bash
# SLAM Toolbox with dynamically filtered scans
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  slam_method:=slam_toolbox slam_scan_topic:=/scan_filtered

# Cartographer with raw scans and external odometry
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  slam_method:=cartographer slam_scan_topic:=/scan

# RTAB-Map with filtered scans and RGB visual loop recognition
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  slam_method:=rtabmap slam_scan_topic:=/scan_filtered \
  rtabmap_use_camera:=true \
  rtabmap_delete_database_on_start:=true
```

All three variants publish the occupancy map on `/map` and the map transform
relative to `odom`. Cartographer consumes `/scan` and `/odom`.
RTAB-Map consumes `/scan`, `/odom`, and, when `rtabmap_use_camera:=true`,
`/camera/image_raw` plus `/camera/camera_info`.

The XGO bridge polls the controller's roll, pitch, and yaw registers. Yaw is
zero-referenced when the bridge starts and unwrapped between samples. The
registers do not provide angular velocity or linear acceleration, so both are
explicitly marked unavailable in `/imu/data`. Measured yaw is used for `/odom`
heading, but live Cartographer is configured with `use_imu_data=false` because
Cartographer requires valid angular velocity.

For independent RTAB-Map evaluation runs, use a different
`rtabmap_database_path` for each run or set
`rtabmap_delete_database_on_start:=true`. The latter intentionally erases the
database at the selected path.

Motion remains disabled by default. If this bridge is also responsible for
executing `/cmd_vel`, explicitly enable it:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py enable_motion:=true
```

The live bridge already performs the same command-plus-IMU-yaw integration as
`xgo_offline_odom_node`, then publishes `/odom` and `odom -> base_link`.
The offline node is only launched by the bag-replay launch files. Do not start
it beside the live bridge, because both nodes would publish the same odometry
topic and TF transform. For correct live translation, teleoperation must publish
`/cmd_vel` through this bridge; direct Bluetooth motion is not observable.

The launch itself never publishes a movement command. Do not run the individual
`xgo-bridge`, `lidar`, `camera`, `filter`, or `foxglove` commands at the same
time as this launch because that would duplicate nodes and contend for hardware.
`Ctrl+C` stops the complete publisher group.

List all switches with:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py --show-args
```

Useful evaluation variants:

```bash
# SLAM Toolbox compute run without camera or visualization overhead
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  slam_method:=slam_toolbox start_camera:=false start_foxglove:=false

# Raw-scan compute baseline without paying the filter's CPU cost
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  slam_method:=slam_toolbox slam_scan_topic:=/scan \
  start_dynamic_filter:=false start_camera:=false start_foxglove:=false
```

For localization-accuracy comparisons, leave the filter enabled so `/scan` and
`/scan_filtered` come from the same live run, then switch only
`slam_scan_topic`. Do not start a second mapping launch simultaneously.

RTAB-Map's RGB loop recognition requires valid, calibrated `CameraInfo`.
Camera images can still be displayed without calibration, but an all-zero or
missing camera matrix is not suitable for the RGB SLAM input. Use
`rtabmap_use_camera:=false` until the OV5647 calibration YAML is installed.

The wrapper accepts the same selection through environment variables:

```bash
ENABLE_MOTION=true SLAM_METHOD=cartographer SLAM_SCAN_TOPIC=/scan \
  bash scripts/robot_stack.sh sensors
```

## 4. Record An Evaluation Run And CPU Use

The robot image explicitly installs `rosbag2` and its MCAP storage plugin. A
single command can start one mapper, record its ROS topics, and sample CPU, RAM,
CPU frequency, and Raspberry Pi temperature:

```bash
bash scripts/robot_evaluation_run.sh \
  slam_toolbox filtered slam_toolbox_filtered_run1
```

The accepted mapper names are `slam_toolbox`, `cartographer`, and `rtabmap`.
The scan variant is `raw` or `filtered`. For example:

```bash
bash scripts/robot_evaluation_run.sh \
  cartographer raw cartographer_raw_run1

bash scripts/robot_evaluation_run.sh \
  rtabmap filtered rtabmap_filtered_run1
```

Do not start `robot_sensor_bringup.launch.py` separately. This command starts
and stops the complete sensor/SLAM process group itself. Drive the route after
the "Recording has started" message and press `Ctrl+C` once at the end.

By default, camera input is enabled for RTAB-Map and disabled for SLAM Toolbox
and Cartographer. Foxglove/Lichtblick is disabled for all three to avoid adding
network and serialization work to the compute measurement. Override these
settings when the purpose of a run is live visualization:

```bash
START_CAMERA=true RECORD_CAMERA=true START_FOXGLOVE=true \
  bash scripts/robot_evaluation_run.sh \
  slam_toolbox filtered slam_toolbox_filtered_visualized_run1
```

Each run is kept in its own directory:

```text
evaluation_runs/<run_name>/
├── bag/                    # MCAP rosbag and metadata.yaml
├── bag_info.txt            # Topic/message inventory from ros2 bag info
├── launch.log              # ROS launch and node output
├── resources/
│   ├── system.csv          # Whole-Pi CPU, RAM, load, temperature, frequency
│   ├── processes.csv       # Per-process CPU/RAM samples
│   └── summary.json        # Mean and maximum values by component
└── run_config.txt          # Mapper and launch settings used for this run
```

`summary.json` separates `slam_toolbox`, `cartographer`, `rtabmap`,
`dynamic_filter`, `camera`, `xgo_bridge`, `lidar`, `foxglove`, and
`bag_recorder`. `cpu_percent_one_core=100` means one logical CPU core is fully
occupied; it can exceed 100 for multithreaded software.
`cpu_percent_total_capacity=100` means all Raspberry Pi logical CPU cores are
fully occupied. Whole-system CPU also includes the monitor and host activity.

Bag compression is intentionally disabled because compression would distort
the CPU benchmark. The bag contains TF, odometry, IMU, applied and requested
velocity, raw LiDAR, map, battery, and (when enabled) compressed camera data.
A filtered run additionally records `/scan_filtered`.

For a fair raw-versus-filtered compute comparison, the script disables the
dynamic-filter process in a raw run and enables it in a filtered run. Use the
same route, duration, speed, camera setting, Foxglove setting, and number of
runs for every variant. Three repetitions per configuration are recommended.

To monitor an already-running stack without launching or recording a bag:

```bash
python3 deployment/scripts/monitor_robot_resources.py \
  --output-dir evaluation_runs/manual_monitor/resources \
  --interval 1.0 \
  --label manual_monitor
```

Stop the monitor with `Ctrl+C`.

If the existing container predates the rosbag Dockerfile dependency, either
rebuild the image (recommended) or install the packages temporarily inside the
running container:

```bash
apt update
apt install -y ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap
```

Packages installed interactively disappear when that container is replaced.

## 5. Verify Robot Topics

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

## 6. Run Mapping And Navigation

Use separate container shells:

```bash
docker compose -f docker-compose.robot.yml exec xgo-robot bash
```

Terminal 1:

```bash
bash scripts/robot_stack.sh xgo-bridge
```

This starts the official XGO SDK bridge on `/dev/ttyAMA0` with motion
disabled. It should publish `/imu/data` (and the compatibility topic `/imu`),
`/battery_state`, `/odom`, and TF from `odom` to `base_link`.

Only after verifying the bridge topics, restart it with motion enabled:

```bash
# First stop xgo-bridge with Ctrl+C, then:
bash scripts/robot_stack.sh xgo-motion
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

## 7. Lichtblick Visualization

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

## 8. Save A Map

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
/scan        LaserScan from the LD19 LiDAR
/imu/data    XGO roll/pitch/yaw orientation (gyro and acceleration unavailable)
/odom        command-integrated X/Y with IMU yaw
/tf          odom -> base_link -> lidar frame
/cmd_vel     velocity command input
```

The current robot setup uses:

```text
/dev/ttyAMA0  XGO body controller through the official xgolib SDK
/dev/ttyUSB0  LD19/LDROBOT LiDAR through ldlidar_stl_ros2
/dev/video0   OV5647 camera stream through camera_ros
/dev/media0   Raspberry Pi camera media controller
/dev/vchiq    Raspberry Pi camera/GPU interface
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

The robot container pins the official `xgolib` package to the tested version
declared by `XGOLIB_VERSION` in `.devcontainer/Dockerfile.robot`. The XGO body
controller responds on `/dev/ttyAMA0` (`/dev/serial0` points there on the
robot). Only one process may own this serial port.

The container deliberately runs as root and privileged, with the same explicit
device mappings that made `xgo6_v6` work. This avoids host/container group-ID
mismatches for the Raspberry Pi camera and serial devices.

The CP2102 adapter at `/dev/ttyUSB0` is the expected LD19/LDROBOT LiDAR path.
Start it with:

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

This uses the camera config installed with the hardware bridge:

```text
src/xgo_driver_bridge/config/camera_ros_robot.yaml
```

The current robot camera settings are:

```text
camera=<automatically select the single detected camera>
format="BGR888"
width=640
height=480
frame_id="camera_optical_frame"
```

That matches what we found on the robot:

- `camera_ros` selects the OV5647 correctly when no explicit camera name is set.
- The `0:` shown in the startup camera list is only a display index. Passing
  `camera="0"` makes the node search for a camera literally named `0`.
- The repo's camera consumers convert `/camera/image_raw` to `bgr8`, so `BGR888` is the safest explicit format.
- `640x480` is a better default than the previous auto-selected `800x600` for bandwidth and visualization delay.

### Safe XGO Bridge Test

Build the workspace, then start the bridge without motion:

```bash
bash scripts/robot_build_workspace.sh
bash scripts/robot_stack.sh xgo-bridge
```

In another container shell:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /battery_state --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Stop the motion-disabled bridge before starting `xgo-motion`; never run two XGO
bridges at once. Test `tiny-forward` only with clear space around the robot.

You may still see one calibration warning until you provide a real camera calibration file. That warning is expected and does not mean the image stream itself failed to start.

For Lichtblick, use `/camera/image_raw/compressed`.

## Raspberry Pi Host Configuration Checklist

These settings live on the Raspberry Pi host and are not included in a Docker
image or `docker commit`. Check them whenever a new robot or SD card is
prepared. Make a backup before changing a boot file.

### UART And Bluetooth

The tested XGO setup requires the stable PL011 UART for the body controller:

```text
/dev/ttyAMA0
baud: 115200
```

On Ubuntu for Raspberry Pi, edit `/boot/firmware/config.txt`. Raspberry Pi OS
may instead use `/boot/config.txt`. Ensure that the following settings occur
only once:

```ini
enable_uart=1
dtoverlay=miniuart-bt
```

`miniuart-bt` moves Bluetooth to the less stable mini UART and leaves
`ttyAMA0` available for the XGO controller. Bluetooth can consequently be less
stable under heavy CPU-frequency changes.

The kernel command line is normally `/boot/firmware/cmdline.txt`. It must remain
one single line. Remove serial-console arguments such as:

```text
console=serial0,115200
console=ttyAMA0,115200
```

Do not copy the complete `cmdline.txt` from another SD card because it can
contain machine-specific root-filesystem parameters.

Disable any serial getty that could open the XGO UART:

```bash
sudo systemctl disable --now serial-getty@ttyAMA0.service
sudo systemctl disable --now serial-getty@serial0.service
sudo systemctl disable --now serial-getty@ttyS0.service
```

After editing the boot configuration, reboot and verify:

```bash
readlink -f /dev/serial0
ls -l /dev/ttyAMA0 /dev/ttyS0
```

For the current setup, `/dev/serial0` should resolve to `/dev/ttyAMA0`. Only one
container or host process may open this port at a time.

### Serial And Camera Permissions

The container currently runs as root and privileged, but correct host
permissions are still useful for diagnostics:

```bash
sudo usermod -aG dialout,tty,video "$USER"
```

Log out and back in after changing groups. A suitable host udev file such as
`/etc/udev/rules.d/99-robot-devices.rules` can contain:

```udev
KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", GROUP="dialout", MODE="0660"
```

Apply the rules with:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The LD19 currently appears as `/dev/ttyUSB0`. This number can change when
additional USB serial adapters are connected. Check the adapter identity with:

```bash
udevadm info --query=property --name=/dev/ttyUSB0
```

If multiple serial adapters will be used, create a model/serial-specific udev
symlink and update `docker-compose.robot.yml` and the LiDAR launch argument to
use that stable path.

### Raspberry Pi Camera

The OV5647 must be detected by the host before Docker can use it. Depending on
the host image, camera auto-detection may be enabled with:

```ini
camera_auto_detect=1
```

Some older host images instead require the explicit overlay:

```ini
dtoverlay=ov5647
```

Do not enable both approaches unnecessarily. After rebooting, inspect the media
graph:

```bash
v4l2-ctl --list-devices
ls -l /dev/video* /dev/media* /dev/vchiq
```

Media device numbers such as `/dev/media0` and `/dev/media2` are not guaranteed
to be identical on every host. The current privileged container can see the
whole media graph, and `/run/udev` is mounted read-only so `libcamera` can
discover it.

Camera calibration is robot/camera-specific. Preserve the calibration YAML
separately and verify that `camera_ros` loads it. A file created at
`/root/.ros/camera_info/` inside the container is included in a Docker snapshot,
but a file in this repository is a bind-mounted host file and must be
transferred with the repository.

### Docker Runtime And ROS Network

The hardware container relies on these settings in
`docker-compose.robot.yml`:

```text
privileged: true
network_mode: host
ipc: host
/dev/ttyAMA0
/dev/ttyUSB0
/dev/video0
/dev/media0
/dev/vchiq
/run/udev:/run/udev:ro
```

The Docker image does not preserve this Compose configuration, so copy the
repository and Compose file together with the image.

Give each physical robot its own `ROS_DOMAIN_ID`. Otherwise teleoperation,
parameter requests, images, and velocity commands can cross between robots on
the same network. For example, put this in a `.env` file beside
`docker-compose.robot.yml`:

```dotenv
ROS_DOMAIN_ID=61
```

Use a different number for every independently operated robot. A ROS laptop
must use the domain ID of the robot it is intentionally connecting to.
Lichtblick does not set a ROS domain itself; connect it only to the intended
robot's Foxglove Bridge URL. Also ensure that only one hardware container is
running on a robot:

```bash
docker ps
docker compose -f docker-compose.robot.yml config
```

### Time, Power, Cooling, And Storage

Incorrect time causes confusing bag timestamps and TF errors. Verify host time
and synchronization:

```bash
timedatectl status
```

CPU throttling or undervoltage can make performance measurements
non-reproducible. When `vcgencmd` is available, check:

```bash
vcgencmd get_throttled
vcgencmd measure_temp
```

`get_throttled=0x0` is the desired result. Use the same power supply and cooling
arrangement for every compute benchmark. Check storage before recording camera
bags:

```bash
df -h
```

### Versions And Per-Robot State

Record these values with every robot deployment:

```bash
cat /etc/os-release
uname -a
docker version
docker compose version
```

Also record the Raspberry Pi firmware, XGO controller firmware, camera
calibration, LiDAR adapter identity, hostname, and assigned `ROS_DOMAIN_ID`.
Controller firmware and physical sensor calibration are not stored in the
Docker image and can differ between nominally identical robots.

After every host change or reboot, perform the final container check:

```bash
docker compose -f docker-compose.robot.yml up -d
docker compose -f docker-compose.robot.yml exec xgo-robot bash
bash scripts/robot_stack.sh check
```
