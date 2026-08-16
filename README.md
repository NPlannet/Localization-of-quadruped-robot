# Localization of a Quadruped Robot

ROS 2 Jazzy project for comparing localization and mapping on an XGO Mini2
quadruped robot. It supports:

- SLAM Toolbox, Cartographer, and RTAB-Map;
- raw and dynamically filtered LD19 LiDAR scans;
- RGB loop recognition with RTAB-Map;
- Gazebo simulation and physical Raspberry Pi deployment; and
- repeatable bag replay with accuracy, CPU, memory, and battery measurements.

## Repository layout

```text
robot_sim_sose/
├── src/            ROS 2 packages
├── scripts/        Recording, deployment, and evaluation tools
├── evaluation/     Dataset sidecars and retained results
├── docs/           Task-specific guides
├── third_party/    Vendored LD19 driver
└── .devcontainer/  Desktop and Raspberry Pi Dockerfiles
```

The main project packages are:

- `dynamic_scan_filter`: dynamic LiDAR filtering, scan normalization, and map
  cleanup prototype;
- `xgo_driver_bridge`: XGO SDK bridge, IMU/odometry, robot bringup, and bag
  evaluation nodes; and
- `xgo_description`: Gazebo model, worlds, RViz, and desktop bag replay.

## Desktop setup

From `robot_sim_sose/`:

```bash
docker compose up -d --build
docker compose exec ros-dev bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Simulation demo

Run the demo script inside the built desktop container:

```bash
bash scripts/demo_simulation.sh
```

It starts Gazebo, the dynamic LiDAR filter, SLAM Toolbox, and Nav2. The direct
ROS launch command is:

```bash
ros2 launch xgo_description simulation.launch.py
```

Pass launch arguments through the script when needed, for example
`bash scripts/demo_simulation.sh gui:=false`. This is the maintained Gazebo
demo; Cartographer and RTAB-Map are evaluated through the common robot/bag
pipeline described below.

## Physical robot

Sync the runtime from the laptop:

```bash
bash scripts/sync_robot_runtime.sh pi@ROBOT_HOST
```

On the robot, build and start the container, then launch the sensor stack:

```bash
cd ~/robot_sim_sose
docker compose -f docker-compose.robot.yml up -d --build
docker compose -f docker-compose.robot.yml exec xgo-robot bash
bash scripts/robot_build_workspace.sh
source install/setup.bash
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:=false slam_method:=none
```

See [robot deployment](robot_sim_sose/docs/robot-deployment.md) before enabling
motion. The exact live and replay configuration files are indexed in
[robot runtime configuration](robot_sim_sose/src/xgo_driver_bridge/config/README.md).

## Record and evaluate

With robot bringup already running:

```bash
bash scripts/record_robot_run.sh RUN_NAME
```

For one headless replay evaluation:

```bash
bash scripts/robot_bag_benchmark.sh \
  cartographer raw /path/to/bag /path/to/waypoints.json
```

Runs are written below `evaluation/runs/`; retained summaries belong below
`evaluation/results/`. Raw bags and RTAB-Map databases are intentionally not
stored in ordinary Git.

## Guides

- [Architecture and coordinate frames](robot_sim_sose/docs/architecture.md)
- [Gazebo simulation](robot_sim_sose/docs/simulation.md)
- [Physical robot deployment](robot_sim_sose/docs/robot-deployment.md)
- [Raspberry Pi host setup](robot_sim_sose/docs/raspberry-pi-setup.md)
- [Recording and evaluation](robot_sim_sose/docs/evaluation.md)
- [Script reference](robot_sim_sose/scripts/README.md)
