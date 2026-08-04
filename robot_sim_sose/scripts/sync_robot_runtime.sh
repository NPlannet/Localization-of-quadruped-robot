#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

if [ $# -lt 1 ]; then
  cat <<EOF
Usage: bash scripts/sync_robot_runtime.sh <ssh-target> [remote-dir]

Examples:
  bash scripts/sync_robot_runtime.sh pi@192.168.178.42
  bash scripts/sync_robot_runtime.sh pi@192.168.178.42 ~/robot_sim_sose
EOF
  exit 1
fi

TARGET=$1
REMOTE_DIR=${2:-/home/pi/robot_sim_sose}

cd "${REPO_DIR}"

if [[ "${TARGET}" != *@* && "${TARGET}" != *:* ]]; then
  TARGET="pi@${TARGET}"
fi

if [[ "${REMOTE_DIR}" =~ ^[0-9]+(\.[0-9]+){3}$ ]]; then
  echo "Refusing to use '${REMOTE_DIR}' as a remote directory because it looks like an IP address." >&2
  echo "Usage: bash scripts/sync_robot_runtime.sh <robot-ip-or-ssh-target> [/home/pi/robot_sim_sose]" >&2
  exit 2
fi

echo "Sync target: ${TARGET}"
echo "Remote directory: ${REMOTE_DIR}"

ssh "${TARGET}" "mkdir -p ${REMOTE_DIR}"

RSYNC_PATHS=(
  ./.dockerignore
  ./docker-compose.robot.yml
  ./docs/robot-deployment.md
  ./docs/raspberry-pi-setup.md
  ./.devcontainer/Dockerfile.robot
  ./scripts/
  ./src/xgo_driver_bridge/
  ./src/dynamic_scan_filter/
  ./src/nav2_wavefront_frontier_exploration/
  ./src/xgo_description/config/
)

if [ -d ./third_party/ldlidar_stl_ros2 ]; then
  RSYNC_PATHS+=(./third_party/ldlidar_stl_ros2/)
fi

rsync -av --relative \
  "${RSYNC_PATHS[@]}" \
  "${TARGET}:${REMOTE_DIR}/"

cat <<EOF

Robot runtime files synced to ${TARGET}:${REMOTE_DIR}

Next on the robot:
  cd ${REMOTE_DIR}
  docker compose -f docker-compose.robot.yml build
  docker compose -f docker-compose.robot.yml up -d
EOF
