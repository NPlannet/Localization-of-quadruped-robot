# xgo_description

This package contains the configuration files for building a devcontainer that uses URDF description and Gazebo Sim version 8.11.0 (Harmonic) and ROS 2 Jazzy for the Simulation.


## 📋 Prerequisites

Before running this package, ensure you have the following installed and configured:


    ```bash
 1- sudo apt update
    sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces ros-jazzy-robot-state-publisher ros-jazzy-xacro
    ```
 2- install toolkit so your container can display frames in your graphics card, for Nvidia: sudo apt-get install -y nvidia-container-toolkit.
 
 
 3- the devcontainer.json file in this package is configured to send display frames to X11 display server for linux, if you are using another dispaly server in linux,
     you have to reconfigure the field : "mounts": [
        "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached"
    ]. you can know which display server you linux is using with the command: echo $XDG_SESSION_TYPE
    
4- you have to give that the container uses you display server by running the following command in your Terminal: xhost +local:docker

5- change the username that appear in different commands in the Docker file and devcontainer.json file to math the user name of your device.



## ⚙️ creating and running the devcontainer

- in your IDE click reopen in container, and the IDE will run and configure the container according to the commands in dockerfile and in devcontainer.json


##  Build the XGO_robot package:
    ```bash
    befor you build XGO_robot, make sure you are in the the correct path: /workspaces/robot_sim_sose
   
2. build XGO_robot package: colcon build --symlink-install
    

3. if you did not like a build and you want to change the configuration and build again, you can remove the previous build with the command:
   
   rm -rf build/ install/ log/



##  Running the Simulation

source install/setup.bash
ros2 launch xgo_description gazebo.launch.py or gz sim if you want to choose gazebo built-in robots.

to run the demo node from your container shell.: python3 .devcontainer/check.py 



