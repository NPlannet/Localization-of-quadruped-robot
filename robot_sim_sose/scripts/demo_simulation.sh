#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE=$(cd -- "${SCRIPT_DIR}/.." && pwd)

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy was not found. Run this script inside the desktop container." >&2
  exit 1
fi

if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
  echo "The workspace is not built. Run: colcon build --symlink-install" >&2
  exit 1
fi

cd "${WORKSPACE}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Pass optional ROS launch arguments through, for example: gui:=false
exec ros2 launch xgo_description simulation.launch.py "$@"
