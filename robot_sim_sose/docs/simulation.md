# Gazebo Simulation

## Start the development container

From `robot_sim_sose/` on the laptop:

```bash
docker compose up -d --build
docker compose exec ros-dev bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch xgo_description simulation.launch.py
```

The default launch starts Gazebo with its GUI. Select another world with an
absolute SDF path or run headlessly:

```bash
ros2 launch xgo_description simulation.launch.py \
  world:=/workspaces/robot_sim_sose/src/xgo_description/worlds/dynamic_obstacles_world.sdf

ros2 launch xgo_description simulation.launch.py gui:=false
```

## Inspect the data

Check that the simulated clock, TF, scan, and camera are alive:

```bash
ros2 topic hz /scan
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic list | grep camera
```

Start the project's SLAM-oriented RViz configuration in another sourced shell:

```bash
ros2 launch xgo_description rviz_slam.launch.py
```

For ROS nodes in simulation, keep `use_sim_time:=true`. A mixture of wall time
and simulation time commonly causes RViz message-filter drops.

## Exploration

The Nav2 parameters and exploration package live in
`src/xgo_description/config/nav2_params.yaml` and
`src/nav2_wavefront_frontier_exploration/`. Build and source the workspace
before launching them. Keep robot motion conservative when transferring tuning
from Gazebo to the quadruped; simulated traction and body motion are idealized.
