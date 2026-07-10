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
docker compose -f docker-compose.yml -f docker-compose.nvidia.yml up -d --build
```

Open a shell inside the container:

```bash
docker compose exec ros-dev bash
```

## 4. Build the ROS Workspace

Run this inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dynamic_scan_filter --symlink-install
colcon build --packages-select xgo_description --symlink-install
source install/setup.bash
```

## 5. Run the simulation, including SLAM and navigation

(not necessary in WSL2)
```bash
to Connect to an X server running on your Local machine:

export DISPLAY=:1
to Connect to an X server running on another machine:

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

The Gazebo-based launches start a `dynamic_scan_filter` node automatically.
- Raw LiDAR stays on `/scan` for live obstacle handling.
- Filtered LiDAR is published on `/scan_filtered` and is used by `slam_toolbox`.
- Cluster markers are published on `/dynamic_scan_filter/cluster_markers` and shown in the saved RViz layout..

# 6. run exploration: 
```bash
python3 explore.py
```


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




## 8. Replay Real Robot Bags Without Gazebo

If you recorded data from the real robot into `robot_sim_sose/bag/...`, you can replay that data directly into the localization stack without starting Gazebo.

Build the needed workspace packages inside the dev container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select xgo_description dynamic_scan_filter
source install/setup.bash
```

Then launch bag replay with SLAM and RViz:

```bash
ros2 launch xgo_description bag_replay.launch.py \
  bag_path:=/workspaces/robot_sim_sose/bag/bag/round_002 \
  use_nav2:=false use_dynamic_filter:=true

ros2 launch xgo_description bag_replay_cartographer.launch.py \
  bag_path:=/workspaces/robot_sim_sose/bag/bag/round_002 \
  use_dynamic_filter:=true
```

Notes:
- Use any bag directory that contains a `metadata.yaml`, for example `round_002` or `straightline1`.
- `use_nav2:=true` can be added if you want to inspect costmaps and planning, but replayed bags are not interactive robot control.
