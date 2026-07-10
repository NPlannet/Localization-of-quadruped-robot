#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/workspaces/robot_sim_sose}
ROS_DISTRO=${ROS_DISTRO:-jazzy}
SLAM_PARAMS=${SLAM_PARAMS:-${WORKSPACE}/src/xgo_description/config/slam_toolbox_robot.yaml}
NAV2_PARAMS=${NAV2_PARAMS:-${WORKSPACE}/src/xgo_description/config/nav2_params.yaml}
DYNAMIC_FILTER_PARAMS=${DYNAMIC_FILTER_PARAMS:-${WORKSPACE}/src/xgo_description/config/dynamic_scan_filter_bag_replay.yaml}
LIDAR_SERIAL_PORT=${LIDAR_SERIAL_PORT:-/dev/ttyUSB0}
FOXGLOVE_PORT=${FOXGLOVE_PORT:-8766}
FOXGLOVE_PARAMS=${FOXGLOVE_PARAMS:-${WORKSPACE}/src/xgo_description/config/foxglove_bridge_robot.yaml}
CAMERA_PARAMS=${CAMERA_PARAMS:-${WORKSPACE}/deployment/config/camera_ros_robot.yaml}

source_setup() {
  set +u
  # ROS setup scripts reference unset helper vars internally.
  # Temporarily relaxing nounset keeps this script strict while remaining compatible.
  source "$1"
  set -u
}

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source_setup "${WORKSPACE}/install/setup.bash"
fi

usage() {
  cat <<EOF
Usage: bash scripts/robot_stack.sh <command>

Commands:
  check          Check required robot topics and TF.
  xgo-bridge     Start the XGO SDK bridge with motion disabled.
  xgo-motion     Start the XGO SDK bridge with /cmd_vel motion enabled.
  lidar          Start the LD19/LDROBOT LiDAR driver on ${LIDAR_SERIAL_PORT}.
  camera         Start camera_ros with the robot camera config and publish /camera/image_raw/compressed.
  sensors        Print commands for starting robot, LiDAR, camera, and Foxglove.
  filter         Start dynamic_scan_filter: /scan -> /scan_filtered.
  slam           Start slam_toolbox with real-robot params.
  nav2           Start Nav2 with real time.
  explore        Start the Python exploration script.
  foxglove       Start Foxglove Bridge for Lichtblick on ${FOXGLOVE_PORT} with compressed camera only.
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

cmd_vel_subscriber_count() {
  local info

  if ! info=$(ros2 topic info /cmd_vel 2>/dev/null); then
    echo 0
    return 0
  fi

  awk '/Subscription count:/ {print $3}' <<<"${info}"
}

require_cmd_vel_subscriber() {
  local count

  count=$(cmd_vel_subscriber_count)
  if [ "${count}" -ge 1 ]; then
    return 0
  fi

  cat >&2 <<EOF
No /cmd_vel subscriber is running right now.

Start the motion bridge in another shell first:
  bash scripts/robot_stack.sh xgo-motion

Then retry:
  bash scripts/robot_stack.sh tiny-forward
EOF
  exit 1
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
      --params-file "${CAMERA_PARAMS}" \
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
  ros2 topic hz /camera/image_raw/compressed

EOF
    ;;
  filter)
    ros2 run dynamic_scan_filter dynamic_scan_filter_node --ros-args \
      --params-file "${DYNAMIC_FILTER_PARAMS}"
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
    ros2 run foxglove_bridge foxglove_bridge --ros-args \
      --params-file "${FOXGLOVE_PARAMS}" \
      -p port:="${FOXGLOVE_PORT}"
    ;;
  tiny-forward)
    require_cmd_vel_subscriber
    ros2 topic pub --times 10 -r 10 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    publish_stop
    ;;
  stop)
    require_cmd_vel_subscriber
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
