# Localization of a Quadruped Robot

ROS 2 Jazzy workspace for evaluating mapping and localization on the XGO
Mini2 quadruped robot. The project supports Gazebo simulation, a physical
Raspberry Pi robot, repeatable rosbag replay, and comparisons between:

- SLAM Toolbox;
- Cartographer;
- RTAB-Map with optional RGB loop recognition; and
- raw versus dynamically filtered LiDAR scans.

## Repository Guide

The ROS workspace is in `robot_sim_sose/`:

```text
robot_sim_sose/
├── docs/            Project guides
├── scripts/         User-facing build, deployment, recording, and analysis tools
├── evaluation/      Datasets, generated runs, and evaluation results
├── src/             Project-owned ROS 2 packages
├── third_party/     Vendored ROS packages
├── .devcontainer/   Development and robot Dockerfiles
└── docker-compose*.yml
```

Start with the guide matching your task:

- [Architecture and package responsibilities](robot_sim_sose/docs/architecture.md)
- [Gazebo simulation](robot_sim_sose/docs/simulation.md)
- [Physical robot deployment](robot_sim_sose/docs/robot-deployment.md)
- [Raspberry Pi host configuration](robot_sim_sose/docs/raspberry-pi-setup.md)
- [Bag replay and evaluation](robot_sim_sose/docs/evaluation.md)

## Development Container

From the repository root:

```bash
cd robot_sim_sose
docker compose up -d --build
docker compose exec ros-dev bash
```

For NVIDIA GPU support:

```bash
docker compose -f docker-compose.yml -f docker-compose.nvidia.yml up -d --build
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Start: Simulation

Inside the development container:

```bash
ros2 launch xgo_description simulation.launch.py
```

See [docs/simulation.md](robot_sim_sose/docs/simulation.md) for display,
world, RViz, camera, and exploration instructions.

## Quick Start: Physical Robot

Sync the runtime from the laptop:

```bash
cd robot_sim_sose
bash scripts/sync_robot_runtime.sh pi@robodoge1.local
```

On the robot, enter the hardware container and build the workspace:

```bash
cd ~/robot_sim_sose
docker compose -f docker-compose.robot.yml up -d
docker compose -f docker-compose.robot.yml exec xgo-robot bash
bash scripts/robot_build_workspace.sh
```

Start the robot without SLAM while retaining motion, Lichtblick, both LiDAR
topics, camera, IMU, and odometry:

```bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=true \
  start_foxglove:=true \
  start_dynamic_filter:=true \
  slam_method:=none
```

See [docs/robot-deployment.md](robot_sim_sose/docs/robot-deployment.md) before
operating the physical robot.

## Quick Start: Record A Dataset

With the robot bringup already running, open a second shell in the same
container:

```bash
bash scripts/record_robot_run.sh sensor_only_run1
```

Press `Ctrl+C` once to finalize the MCAP bag and resource summary. The existing
robot bringup remains running. New runs are written under:

```text
robot_sim_sose/evaluation/runs/<run_name>/
```

## Quick Start: Replay W1

The replay launches default to the structured W1 dataset and result paths:

```bash
ros2 launch xgo_description bag_replay.launch.py \
  use_dynamic_filter:=true

ros2 launch xgo_description bag_replay_cartographer.launch.py \
  use_dynamic_filter:=false

ros2 launch xgo_description bag_replay_rtabmap.launch.py \
  use_dynamic_filter:=true use_camera:=true
```

See [docs/evaluation.md](robot_sim_sose/docs/evaluation.md) for methodology,
output files, and plotting.

## Main ROS Packages

| Package | Responsibility |
| --- | --- |
| `xgo_description` | URDF, Gazebo simulation, visualization, and offline replay launches |
| `xgo_driver_bridge` | Physical XGO SDK bridge, odometry, robot bringup, and mapper selection |
| `dynamic_scan_filter` | Filters dynamic LiDAR returns into `/scan_filtered` |
| `nav2_wfd` | Wavefront-frontier exploration using Nav2 |
| `yolo_detector` | Optional camera object detection experiment |
| `ldlidar_stl_ros2` | Third-party LD19 driver under `third_party/` |

## Generated And Large Data

Raw MCAP bags, RTAB-Map databases, and temporary runs are intentionally kept
out of ordinary Git tracking. Store important datasets externally or with Git
LFS. Small reproducible artifacts such as metrics JSON, map YAML/PGM files,
and presentation plots belong under `evaluation/results/`.
