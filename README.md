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

## 5. Run the simulation

```bash
(not necessary in WSL2)
export DISPLAY=host.docker.internal:0.0
start XLaunch with
- multiple windows
- start no client
- disable access control

ros2 launch xgo_description gazebo_fast.launch.py
```

# 6. run SLAM
```bash
open a new terminal in the docker image and run:
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/workspaces/robot_sim_sose/src/xgo_description/config/slam_toolbox.yaml

you can look at the map being build using rviz2:
    1. open a new terminal in the docker image
    2. run rviz2
    3. click on ADD button on the bottom left and add map aswell as odom
```


# 7. run the exploration
open a new terminal in the docker image and run:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/workspaces/robot_sim_sose/src/xgo_description/config/nav2_params.yaml
python3 explore.py
```


## 8. save images

Run the bridge and image saver file
```bash
python3 image_saver.py
```
