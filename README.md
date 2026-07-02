Welcome to our semester project, the localization of a quadruped robot.

## Running On The Real XGO Mini2

For the physical robot, use the robot container deployment instead of the Gazebo simulation launch files:

```bash
cd robot_sim_sose
docker compose -f docker-compose.robot.yml build
docker compose -f docker-compose.robot.yml up -d
```

Then follow the full checklist in [`robot_sim_sose/ROBOT_DEPLOYMENT.md`](robot_sim_sose/ROBOT_DEPLOYMENT.md).

## 1. Install Docker

Install Docker before building the project.

Check that Docker works:

```bash
docker --version
docker compose version
```

## 2. Clone the Repository

```bash
git clone <repository-url>
cd Localization-of-quadruped-robot/robot_sim_sose
```

## 3. Start the Development Container

Build and start the container:

```bash
docker compose up -d --build
```

For an NVIDIA GPU-enabled setup, add the override file:

```bash
docker compose -f docker-compose.yml up -d --build
docker compose -f docker-compose.nvidia.yml up -d --build
```

Open a shell inside the container:

```bash
docker compose exec ros-dev bash
```

## 4. Build the ROS Workspace

Run this inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select xgo_description --symlink-install #updated !!
source install/setup.bash
```

## 5. Run the simulation, including SLAM and navigation

(not necessary in WSL2)
```bash
export DISPLAY=host.docker.internal:0.0
```
start XLaunch with
- multiple windows
- start no client
- disable access control


for running the default world
```bash
ros2 launch xgo_description simulation.launch.py
```

for running the real objects world with explicit path:
```bash
ros2 launch xgo_description simulation.launch.py world:=/workspaces/robot_sim_sose/src/xgo_description/worlds/real_objects_world.sdf
```

The Gazebo-based launches now start a `dynamic_scan_filter` node automatically.
- Raw LiDAR stays on `/scan` for live obstacle handling.
- Filtered LiDAR is published on `/scan_filtered` and is used by `slam_toolbox`.
- Cluster markers are published on `/dynamic_scan_filter/cluster_markers` and shown in the saved RViz layout.
- When you launch `real_objects_world.sdf`, the slow gliding movers run through a native Gazebo plugin inside the simulation loop.
- To add another mover, copy one of the existing `<plugin filename="xgo_glide_trajectory_system" ...>` blocks in `real_objects_world.sdf` and change `offset` or `start` / `end`, `period`, and `phase_offset`.


# 6. run YOLO-detector and exploration: 
```bash
ros2 run yolo_detector detector_node
python3 explore.py
```

The detector reads the `YOLO_DEVICE` environment variable.
- Default: `auto` -> uses CUDA when available, otherwise CPU
- AMD / non-CUDA systems: keep the default or set `YOLO_DEVICE=cpu`
- NVIDIA systems: use `docker-compose.nvidia.yml`, which sets `YOLO_DEVICE=cuda`


# 7. Visuals (RVIZ, and camera image)
open a new terminal in the docker image and run:
```bash
ros2 launch xgo_description rviz_slam.launch.py
ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/camera/image_raw
```

EXTRAS:


Server-only mode:
```bash
ros2 launch xgo_description gazebo_fast.launch.py gui:=false

Stop the Container:

From the host terminal in `robot_sim_sose`:

```bash
docker compose down
```
You can also stop the container from Docker Desktop.
