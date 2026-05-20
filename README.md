Welcome to our semester project, the localization of a quadruped robot.
## 1. Install Docker

Install Docker before building the project.

- Linux: install Docker Engine or Docker Desktop.
- Windows: install Docker Desktop and enable WSL 2 integration for your Linux distro.
- macOS: install Docker Desktop. Gazebo GUI support may depend on your machine and display setup.

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
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
```
