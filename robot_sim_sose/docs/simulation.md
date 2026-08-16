# Gazebo Simulation

Start the desktop container from `robot_sim_sose/`:

```bash
docker compose up -d --build
docker compose exec ros-dev bash
```

Build and launch inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch xgo_description simulation.launch.py
```

The default launch starts Gazebo, the dynamic scan filter, SLAM Toolbox, and
Nav2. Run without the Gazebo GUI with:

```bash
ros2 launch xgo_description simulation.launch.py gui:=false
```

Use the moving-object world and map-cleanup prototype with:

```bash
ros2 launch xgo_description real_objects.launch.py
```

Open the project RViz configuration in another sourced shell:

```bash
ros2 launch xgo_description rviz_slam.launch.py
```

Basic checks:

```bash
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
```

All simulation nodes must use simulation time. Mixing wall time and `/clock`
causes TF and RViz message-filter errors.
