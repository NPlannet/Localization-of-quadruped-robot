Welcome to our semester project, the localization of a quadruped robot.
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

Open a shell inside the container:

```bash
docker compose exec ros-dev bash
```

## 4. Build the ROS Workspace

Run this inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Run the simulation, including SLAM and navigation

```bash
(not necessary in WSL2)
export DISPLAY=host.docker.internal:0.0
start XLaunch with
- multiple windows
- start no client
- disable access control

ros2 launch xgo_description gazebo_fast.launch.py
```

# 6. run YOLO-detector and exploration: 
```bash
ros2 run yolo_detector detector_node
python3 explore.py
```


# 7. Visuals (RVIZ, and camera image)
open a new terminal in the docker image and run:
```bash
ros2 launch xgo_description rviz_slam.launch.py
ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/camera/image_raw
```



