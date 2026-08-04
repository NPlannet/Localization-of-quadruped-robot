#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/workspaces/robot_sim_sose}
ROS_DISTRO=${ROS_DISTRO:-jazzy}
OUTPUT_ROOT=${OUTPUT_ROOT:-${WORKSPACE}/evaluation/runs}
RESOURCE_INTERVAL=${RESOURCE_INTERVAL:-1.0}
ENABLE_MOTION=${ENABLE_MOTION:-true}
START_FOXGLOVE=${START_FOXGLOVE:-false}
WARMUP_SECONDS=${WARMUP_SECONDS:-5}

source_setup() {
  set +u
  source "$1"
  set -u
}

usage() {
  cat <<EOF
Usage:
  bash scripts/robot_evaluation_run.sh <slam_method> <scan_variant> <run_name>

Arguments:
  slam_method   slam_toolbox, cartographer, or rtabmap
  scan_variant  raw or filtered
  run_name      Unique output-directory name, for example:
                slam_toolbox_filtered_run1

The command starts the complete robot stack, resource monitoring, and rosbag
recording. Walk the evaluation route, then press Ctrl+C once to stop and save.

Optional environment variables:
  OUTPUT_ROOT=/workspaces/robot_sim_sose/evaluation/runs
  RESOURCE_INTERVAL=1.0
  ENABLE_MOTION=true
  START_CAMERA=true|false
  RECORD_CAMERA=true|false
  START_FOXGLOVE=false
  WARMUP_SECONDS=5
EOF
}

if [ "$#" -ne 3 ]; then
  usage
  exit 2
fi

SLAM_METHOD=$1
SCAN_VARIANT=$2
RUN_NAME=$3

case "${SLAM_METHOD}" in
  slam_toolbox|cartographer|rtabmap) ;;
  *)
    echo "Unsupported SLAM method: ${SLAM_METHOD}" >&2
    usage
    exit 2
    ;;
esac

case "${SCAN_VARIANT}" in
  raw)
    SLAM_SCAN_TOPIC=/scan
    START_DYNAMIC_FILTER=false
    ;;
  filtered)
    SLAM_SCAN_TOPIC=/scan_filtered
    START_DYNAMIC_FILTER=true
    ;;
  *)
    echo "Unsupported scan variant: ${SCAN_VARIANT}" >&2
    usage
    exit 2
    ;;
esac

if [[ ! "${RUN_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "run_name may only contain letters, numbers, dot, underscore, and dash." >&2
  exit 2
fi

if [ "${SLAM_METHOD}" = "rtabmap" ]; then
  START_CAMERA=${START_CAMERA:-true}
else
  START_CAMERA=${START_CAMERA:-false}
fi
RECORD_CAMERA=${RECORD_CAMERA:-${START_CAMERA}}

RUN_DIR=${OUTPUT_ROOT}/${RUN_NAME}
RESOURCE_DIR=${RUN_DIR}/resources
BAG_DIR=${RUN_DIR}/bag
LAUNCH_LOG=${RUN_DIR}/launch.log
QOS_OVERRIDES=${WORKSPACE}/src/xgo_driver_bridge/config/bag_record_qos.yaml
MONITOR_SCRIPT=${WORKSPACE}/scripts/monitor_robot_resources.py

if [ -e "${RUN_DIR}" ]; then
  echo "Refusing to overwrite existing run directory: ${RUN_DIR}" >&2
  exit 1
fi

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source_setup "${WORKSPACE}/install/setup.bash"
else
  echo "Workspace is not built: ${WORKSPACE}/install/setup.bash is missing." >&2
  exit 1
fi

if ! ros2 bag record --help >/dev/null 2>&1; then
  cat >&2 <<EOF
The current container does not contain rosbag2.
Rebuild it after syncing the updated Dockerfile, or install it temporarily:
  apt update
  apt install -y ros-${ROS_DISTRO}-rosbag2 ros-${ROS_DISTRO}-rosbag2-storage-mcap
EOF
  exit 1
fi

mkdir -p "${RESOURCE_DIR}"

printf '%s\n' \
  "run_name=${RUN_NAME}" \
  "slam_method=${SLAM_METHOD}" \
  "scan_variant=${SCAN_VARIANT}" \
  "slam_scan_topic=${SLAM_SCAN_TOPIC}" \
  "start_dynamic_filter=${START_DYNAMIC_FILTER}" \
  "start_camera=${START_CAMERA}" \
  "record_camera=${RECORD_CAMERA}" \
  "start_foxglove=${START_FOXGLOVE}" \
  "enable_motion=${ENABLE_MOTION}" \
  "resource_interval_s=${RESOURCE_INTERVAL}" \
  "ros_domain_id=${ROS_DOMAIN_ID:-unset}" \
  "started_at=$(date --iso-8601=seconds)" \
  > "${RUN_DIR}/run_config.txt"

LAUNCH_PID=
MONITOR_PID=
CLEANED=false

cleanup() {
  if [ "${CLEANED}" = "true" ]; then
    return
  fi
  CLEANED=true
  set +e

  if [ -n "${MONITOR_PID}" ] && kill -0 "${MONITOR_PID}" 2>/dev/null; then
    kill -INT "${MONITOR_PID}" 2>/dev/null
    wait "${MONITOR_PID}" 2>/dev/null
  fi
  if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null
    wait "${LAUNCH_PID}" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

echo "Starting ${SLAM_METHOD} with ${SCAN_VARIANT} scans..."
ros2 launch xgo_driver_bridge robot_sensor_bringup.launch.py \
  enable_motion:="${ENABLE_MOTION}" \
  start_camera:="${START_CAMERA}" \
  start_dynamic_filter:="${START_DYNAMIC_FILTER}" \
  start_foxglove:="${START_FOXGLOVE}" \
  slam_method:="${SLAM_METHOD}" \
  slam_scan_topic:="${SLAM_SCAN_TOPIC}" \
  rtabmap_use_camera:="${START_CAMERA}" \
  rtabmap_database_path:="${RUN_DIR}/rtabmap.db" \
  rtabmap_delete_database_on_start:=true \
  > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

echo "Waiting for required topics. Launch output: ${LAUNCH_LOG}"
READY=false
for _attempt in $(seq 1 45); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "The robot launch exited before becoming ready:" >&2
    tail -n 80 "${LAUNCH_LOG}" >&2
    exit 1
  fi

  TOPIC_LIST=$(timeout 3 ros2 topic list 2>/dev/null || true)
  if grep -qx "/scan" <<<"${TOPIC_LIST}" \
      && grep -qx "/odom" <<<"${TOPIC_LIST}" \
      && grep -qx "/imu/data" <<<"${TOPIC_LIST}"; then
    if [ "${SCAN_VARIANT}" = "filtered" ] \
        && ! grep -qx "/scan_filtered" <<<"${TOPIC_LIST}"; then
      sleep 1
      continue
    fi
    if [ "${START_CAMERA}" = "true" ] \
        && ! grep -qx "/camera/image_raw" <<<"${TOPIC_LIST}"; then
      sleep 1
      continue
    fi
    READY=true
    break
  fi
  sleep 1
done

if [ "${READY}" != "true" ]; then
  echo "Required robot topics did not appear within 45 seconds." >&2
  tail -n 80 "${LAUNCH_LOG}" >&2
  exit 1
fi

if [ "${WARMUP_SECONDS}" != "0" ]; then
  echo "Topics ready; allowing ${WARMUP_SECONDS}s for mapper warm-up..."
  sleep "${WARMUP_SECONDS}"
fi

python3 "${MONITOR_SCRIPT}" \
  --output-dir "${RESOURCE_DIR}" \
  --interval "${RESOURCE_INTERVAL}" \
  --label "${RUN_NAME}" &
MONITOR_PID=$!

TOPICS=(
  /tf
  /tf_static
  /odom
  /imu/data
  /xgo/yaw_deg
  /xgo/applied_vel
  /cmd_vel
  /scan
  /battery_state
  /map
  /map_metadata
)

if [ "${SCAN_VARIANT}" = "filtered" ]; then
  TOPICS+=(/scan_filtered)
fi
if [ "${RECORD_CAMERA}" = "true" ]; then
  TOPICS+=(/camera/image_raw/compressed /camera/camera_info)
fi
if [ "${SLAM_METHOD}" = "rtabmap" ]; then
  TOPICS+=(/rtabmap/info /rtabmap/mapGraph)
fi

cat <<EOF

Recording has started.
  Run:       ${RUN_NAME}
  Algorithm: ${SLAM_METHOD}
  Scan:      ${SLAM_SCAN_TOPIC}
  Output:    ${RUN_DIR}

Drive the robot through the test route now.
Press Ctrl+C once when the run is complete.

EOF

set +e
ros2 bag record \
  --storage mcap \
  --output "${BAG_DIR}" \
  --qos-profile-overrides-path "${QOS_OVERRIDES}" \
  "${TOPICS[@]}"
BAG_STATUS=$?
set -e

cleanup
trap - EXIT INT TERM

if [ -d "${BAG_DIR}" ]; then
  ros2 bag info "${BAG_DIR}" > "${RUN_DIR}/bag_info.txt" 2>&1 || true
fi

printf '\nRun saved to %s\n' "${RUN_DIR}"
printf 'Resource summary: %s\n' "${RESOURCE_DIR}/summary.json"
printf 'Bag: %s\n' "${BAG_DIR}"
printf 'Bag inventory: %s\n' "${RUN_DIR}/bag_info.txt"
printf 'Launch log: %s\n' "${LAUNCH_LOG}"

if [ "${BAG_STATUS}" -ne 0 ] && [ "${BAG_STATUS}" -ne 130 ]; then
  exit "${BAG_STATUS}"
fi
