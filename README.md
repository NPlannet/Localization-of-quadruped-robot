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
export DISPLAY=host.docker.internal:0.0
start XLaunch with
- multiple windows
- start no client
- disable access control

ros2 launch xgo_description gazebo_fast.launch.py
```

## 6. save images

Run the bridge and image saver file
```bash
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
python3 image_saver.py
```