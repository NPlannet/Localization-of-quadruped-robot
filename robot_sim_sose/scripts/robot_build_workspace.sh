#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/workspaces/robot_sim_sose}
ROS_DISTRO=${ROS_DISTRO:-jazzy}

source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${WORKSPACE}"

colcon build --symlink-install --packages-select \
  xgo_driver_bridge \
  dynamic_scan_filter \
  nav2_wfd \
  yolo_detector

echo
echo "Workspace built. In each new shell run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WORKSPACE}/install/setup.bash"
