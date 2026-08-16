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

Use the moving-object world with any supported mapping algorithm:

```bash
bash scripts/demo_simulation.sh slam_method:=slam_toolbox
bash scripts/demo_simulation.sh slam_method:=cartographer
bash scripts/demo_simulation.sh slam_method:=rtabmap
```

Run one variant at a time. The default is SLAM Toolbox. RTAB-Map uses the
simulated RGB camera unless `rtabmap_use_camera:=false` is passed.

Open the project RViz configuration in another sourced shell:

```bash
ros2 launch xgo_description rviz_slam.launch.py
```

To let the robot automatically choose Nav2 goals and explore, open a third
sourced shell and run:

```bash
ros2 run nav2_wfd explore
```

The simulation launch starts Nav2, but Nav2 does not move without a goal. The
`nav2_wfd` process supplies reachable free-space frontier goals while RViz
displays the generated map. Wait until Nav2 reports that its managed nodes are
active before expecting movement; the explorer waits for that state itself.

Basic checks:

```bash
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
```

All simulation nodes must use simulation time. Mixing wall time and `/clock`
causes TF and RViz message-filter errors.
