#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/workspaces/robot_sim_sose}
ROS_DISTRO=${ROS_DISTRO:-jazzy}
OUTPUT_ROOT=${OUTPUT_ROOT:-${WORKSPACE}/evaluation/runs}
RESOURCE_INTERVAL=${RESOURCE_INTERVAL:-1.0}
RECORD_CAMERA=${RECORD_CAMERA:-true}

source_setup() {
  set +u
  source "$1"
  set -u
}

usage() {
  cat <<EOF
Usage:
  bash scripts/record_robot_run.sh <run_name>

This attaches to an already-running robot bringup. It records a rosbag and
CPU/RAM measurements, but does not launch, stop, or reconfigure any ROS node.

Example:
  bash scripts/record_robot_run.sh slam_toolbox_filtered_run1

Optional environment variables:
  OUTPUT_ROOT=/workspaces/robot_sim_sose/evaluation/runs
  RESOURCE_INTERVAL=1.0
  RECORD_CAMERA=true|false
EOF
}

if [ "$#" -ne 1 ]; then
  usage
  exit 2
fi

RUN_NAME=$1
if [[ ! "${RUN_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "run_name may only contain letters, numbers, dot, underscore, and dash." >&2
  exit 2
fi

RUN_DIR=${OUTPUT_ROOT}/${RUN_NAME}
RESOURCE_DIR=${RUN_DIR}/resources
BAG_DIR=${RUN_DIR}/bag
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
The current container does not contain rosbag2. Install it with:
  apt update
  apt install -y ros-${ROS_DISTRO}-rosbag2 ros-${ROS_DISTRO}-rosbag2-storage-mcap
EOF
  exit 1
fi

if [ ! -f "${MONITOR_SCRIPT}" ] || [ ! -f "${QOS_OVERRIDES}" ]; then
  echo "Resource monitor or bag QoS configuration is missing." >&2
  exit 1
fi

TOPIC_LIST=$(timeout 5 ros2 topic list 2>/dev/null || true)
MISSING_TOPICS=()
for topic in /scan /odom /imu/data /tf; do
  if ! grep -qx "${topic}" <<<"${TOPIC_LIST}"; then
    MISSING_TOPICS+=("${topic}")
  fi
done

if [ "${#MISSING_TOPICS[@]}" -ne 0 ]; then
  printf 'The existing bringup is missing required topic(s):' >&2
  printf ' %s' "${MISSING_TOPICS[@]}" >&2
  printf '\nStart and verify robot_sensor_bringup.launch.py first.\n' >&2
  exit 1
fi

if ! grep -qx "/map" <<<"${TOPIC_LIST}"; then
  echo "WARNING: /map is not visible yet. Recording will wait for it to appear." >&2
fi
if ! grep -qx "/scan_filtered" <<<"${TOPIC_LIST}"; then
  echo "NOTE: /scan_filtered is not currently available; raw /scan will still be recorded."
fi
if [ "${RECORD_CAMERA}" = "true" ] \
    && ! grep -qx "/camera/image_raw/compressed" <<<"${TOPIC_LIST}"; then
  echo "WARNING: compressed camera topic is not visible yet." >&2
fi

mkdir -p "${RESOURCE_DIR}"

NODE_LIST=$(timeout 5 ros2 node list 2>/dev/null || true)
printf '%s\n' "${NODE_LIST}" | sort > "${RUN_DIR}/ros_nodes.txt"
timeout 10 ros2 topic list -t 2>/dev/null \
  | sort > "${RUN_DIR}/ros_topics.txt" || true

{
  printf 'run_name=%s\n' "${RUN_NAME}"
  printf 'mode=attach_to_existing_bringup\n'
  printf 'record_camera=%s\n' "${RECORD_CAMERA}"
  printf 'resource_interval_s=%s\n' "${RESOURCE_INTERVAL}"
  printf 'ros_domain_id=%s\n' "${ROS_DOMAIN_ID:-unset}"
  printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'foxglove_detected=%s\n' "$(grep -q foxglove <<<"${NODE_LIST}" && echo true || echo false)"
  printf 'slam_toolbox_detected=%s\n' "$(grep -q slam_toolbox <<<"${NODE_LIST}" && echo true || echo false)"
  printf 'cartographer_detected=%s\n' "$(grep -q cartographer <<<"${NODE_LIST}" && echo true || echo false)"
  printf 'rtabmap_detected=%s\n' "$(grep -q rtabmap <<<"${NODE_LIST}" && echo true || echo false)"
  printf 'dynamic_filter_detected=%s\n' "$(grep -q dynamic_scan_filter <<<"${NODE_LIST}" && echo true || echo false)"
} > "${RUN_DIR}/run_config.txt"

for node in ${NODE_LIST}; do
  case "${node}" in
    *slam_toolbox*|*cartographer*|*rtabmap*|*dynamic_scan_filter*|*xgo_driver_bridge*|*foxglove*)
      {
        printf '\n===== %s =====\n' "${node}"
        timeout 5 ros2 node info "${node}" 2>&1 || true
      } >> "${RUN_DIR}/ros_node_details.txt"
      ;;
  esac
done

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
}
trap cleanup EXIT INT TERM

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
  /joint_states
  /xgo/yaw_deg
  /xgo/applied_vel
  /cmd_vel
  /scan
  /scan_filtered
  /battery_state
  /map
  /map_metadata
  /rosout
  /parameter_events
  /diagnostics
  /rtabmap/info
  /rtabmap/mapGraph
  /submap_list
)

if [ "${RECORD_CAMERA}" = "true" ]; then
  TOPICS+=(/camera/image_raw/compressed /camera/camera_info)
fi

cat <<EOF

Standalone recording has started.
  Run:    ${RUN_NAME}
  Output: ${RUN_DIR}

The existing bringup, mapper, Foxglove Bridge, and motion control are unchanged.
Drive the test route and press Ctrl+C once when recording is complete.

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
printf 'The existing robot bringup is still running.\n'

if [ "${BAG_STATUS}" -ne 0 ] && [ "${BAG_STATUS}" -ne 130 ]; then
  exit "${BAG_STATUS}"
fi
