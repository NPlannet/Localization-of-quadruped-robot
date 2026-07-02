#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/workspaces/robot_sim_sose}
ROS_DISTRO=${ROS_DISTRO:-jazzy}
SLAM_PARAMS=${SLAM_PARAMS:-${WORKSPACE}/src/xgo_description/config/slam_toolbox_robot.yaml}
NAV2_PARAMS=${NAV2_PARAMS:-${WORKSPACE}/src/xgo_description/config/nav2_params.yaml}
LIDAR_SERIAL_PORT=${LIDAR_SERIAL_PORT:-/dev/ttyUSB0}
FOXGLOVE_PORT=${FOXGLOVE_PORT:-8766}

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi

usage() {
  cat <<EOF
Usage: bash scripts/robot_stack.sh <command>

Commands:
  check          Check required robot topics and TF.
  xgo-bridge     Start the XGO SDK bridge with motion disabled.
  xgo-motion     Start the XGO SDK bridge with /cmd_vel motion enabled.
  lidar          Start the LD19/LDROBOT LiDAR driver on ${LIDAR_SERIAL_PORT}.
  camera         Start camera_ros and remap image topics to /camera/*.
  sensors        Print commands for starting robot, LiDAR, camera, and Foxglove.
  filter         Start dynamic_scan_filter: /scan -> /scan_filtered.
  slam           Start slam_toolbox with real-robot params.
  nav2           Start Nav2 with real time.
  explore        Start the Python exploration script.
  foxglove       Start Foxglove Bridge for Lichtblick on ${FOXGLOVE_PORT}.
  tiny-forward   Publish a short, very slow forward velocity command.
  stop           Publish repeated zero velocity commands.
  save-map       Save the current map to maps/xgo_map.
EOF
}

check_topics() {
  ros2 topic list
  timeout 5 ros2 topic hz /scan || true
  timeout 5 ros2 topic echo /odom --once || true
  timeout 5 ros2 run tf2_ros tf2_echo odom base_link || true
}

publish_stop() {
  ros2 topic pub --times 10 -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
}

case "${1:-}" in
  check)
    check_topics
    ;;
  xgo-bridge)
    ros2 launch xgo_driver_bridge xgo_bridge.launch.py enable_motion:=false
    ;;
  xgo-motion)
    ros2 launch xgo_driver_bridge xgo_bridge.launch.py enable_motion:=true
    ;;
  lidar)
    ros2 launch ldlidar_stl_ros2 ld19.launch.py serial_port:="${LIDAR_SERIAL_PORT}"
    ;;
  camera)
    ros2 run camera_ros camera_node --ros-args \
      -r image_raw:=/camera/image_raw \
      -r image_raw/compressed:=/camera/image_raw/compressed \
      -r camera_info:=/camera/camera_info
    ;;
  sensors)
    cat <<EOF
Open one container shell per command:

  bash scripts/robot_stack.sh xgo-bridge
  bash scripts/robot_stack.sh lidar
  bash scripts/robot_stack.sh camera
  bash scripts/robot_stack.sh foxglove

After those are running, verify:

  ros2 topic hz /scan
  ros2 topic echo /scan --once
  ros2 topic hz /camera/image_raw

EOF
    ;;
  filter)
    ros2 run dynamic_scan_filter dynamic_scan_filter_node
    ;;
  slam)
    ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:="${SLAM_PARAMS}" \
      use_sim_time:=false
    ;;
  nav2)
    ros2 launch nav2_bringup navigation_launch.py \
      params_file:="${NAV2_PARAMS}" \
      use_sim_time:=false
    ;;
  explore)
    cd "${WORKSPACE}"
    python3 explore.py
    ;;
  foxglove)
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="${FOXGLOVE_PORT}"
    ;;
  tiny-forward)
    ros2 topic pub --times 10 -r 10 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    publish_stop
    ;;
  stop)
    publish_stop
    ;;
  save-map)
    mkdir -p "${WORKSPACE}/maps"
    ros2 run nav2_map_server map_saver_cli -f "${WORKSPACE}/maps/xgo_map"
    ;;
  *)
    usage
    exit 1
    ;;
esac
