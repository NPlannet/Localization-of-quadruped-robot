#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/workspaces/robot_sim_sose}
ROS_DISTRO=${ROS_DISTRO:-jazzy}
REQUIRE_LIDAR_DRIVER=${REQUIRE_LIDAR_DRIVER:-true}

source_setup() {
  set +u
  # ROS setup scripts reference unset helper vars internally.
  # Temporarily relaxing nounset keeps our wrapper strict while remaining compatible.
  source "$1"
  set -u
}

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${WORKSPACE}"

PACKAGES=(
  xgo_driver_bridge
  dynamic_scan_filter
  nav2_wfd
)

if [ -d "${WORKSPACE}/src/ldlidar_stl_ros2" ]; then
  PACKAGES+=(ldlidar_stl_ros2)
fi

colcon build --symlink-install --packages-select "${PACKAGES[@]}"

if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source_setup "${WORKSPACE}/install/setup.bash"
fi

if ros2 pkg prefix ldlidar_stl_ros2 >/dev/null 2>&1; then
  echo "LiDAR driver package ldlidar_stl_ros2 is available."
elif [ "${REQUIRE_LIDAR_DRIVER}" = "true" ]; then
  echo "ERROR: ldlidar_stl_ros2 is not available after the workspace build." >&2
  echo "Install ros-${ROS_DISTRO}-ldlidar-stl-ros2 or add src/ldlidar_stl_ros2 to this workspace." >&2
  exit 1
else
  echo "WARNING: ldlidar_stl_ros2 is not available after the workspace build." >&2
fi

echo
echo "Workspace built. In each new shell run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WORKSPACE}/install/setup.bash"
