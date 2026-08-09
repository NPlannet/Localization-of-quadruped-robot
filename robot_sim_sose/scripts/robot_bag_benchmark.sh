#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=${WORKSPACE:-/workspaces/robot_sim_sose}
ROS_DISTRO=${ROS_DISTRO:-jazzy}
OUTPUT_ROOT=${OUTPUT_ROOT:-${WORKSPACE}/evaluation/runs}
RESOURCE_INTERVAL=${RESOURCE_INTERVAL:-1.0}
PLAYBACK_RATE=${PLAYBACK_RATE:-1.0}
NORMALIZE_SCAN=${NORMALIZE_SCAN:-false}
START_FOXGLOVE=${START_FOXGLOVE:-false}
FOXGLOVE_PORT=${FOXGLOVE_PORT:-8766}
POST_PLAYBACK_DELAY_SEC=${POST_PLAYBACK_DELAY_SEC:-5.0}
VELOCITY_SCALE=${VELOCITY_SCALE:-1.0}
STATIC_CONFIRMATIONS=${STATIC_CONFIRMATIONS:-20}
STATIC_SPEED_THRESHOLD=${STATIC_SPEED_THRESHOLD:-0.06}
MOVING_CONFIRMATIONS=${MOVING_CONFIRMATIONS:-4}

source_setup() {
  set +u
  source "$1"
  set -u
}

usage() {
  cat <<EOF
Usage:
  bash scripts/robot_bag_benchmark.sh \\
    <slam_method> <scan_variant> <bag_path> [waypoints.json]

Arguments:
  slam_method   slam_toolbox, cartographer, or rtabmap
  scan_variant  raw or filtered
  bag_path      Rosbag2 directory containing metadata.yaml
  waypoints     Optional waypoint sidecar; enables accuracy evaluation

Only original inputs are replayed: /scan, /imu/data, /xgo/applied_vel, and
/tf_static, plus compressed camera images for RTAB-Map RGB. Recorded /odom,
/tf, /scan_filtered, /map, and mapper outputs are never replayed.

Optional environment variables:
  OUTPUT_ROOT=/workspaces/robot_sim_sose/evaluation/runs
  RESOURCE_INTERVAL=1.0
  PLAYBACK_RATE=1.0
  NORMALIZE_SCAN=false       Set true for the irregular legacy W1 scans
  USE_CAMERA=true|false      Defaults true for RTAB-Map, false otherwise
  START_FOXGLOVE=false       Set true for a separately labelled visual run
  FOXGLOVE_PORT=8766
  POST_PLAYBACK_DELAY_SEC=5.0
  RUN_TAG=room1              Optional label added before the unique run number
  VELOCITY_SCALE=1.0
  STATIC_CONFIRMATIONS=20    dynamic_scan_filter static_confirmations override
  STATIC_SPEED_THRESHOLD=0.06  dynamic_scan_filter static_speed_threshold override
  MOVING_CONFIRMATIONS=4     dynamic_scan_filter moving_confirmations override
EOF
}

if [ "$#" -lt 3 ] || [ "$#" -gt 4 ]; then
  usage
  exit 2
fi

SLAM_METHOD=$1
SCAN_VARIANT=$2
BAG_PATH=$3
WAYPOINTS_FILE=${4:-}
RUN_TAG=${RUN_TAG:-}

case "${SLAM_METHOD}" in
  slam_toolbox|cartographer|rtabmap) ;;
  *)
    echo "Unsupported SLAM method: ${SLAM_METHOD}" >&2
    usage
    exit 2
    ;;
esac

case "${SCAN_VARIANT}" in
  raw) USE_DYNAMIC_FILTER=false ;;
  filtered) USE_DYNAMIC_FILTER=true ;;
  *)
    echo "scan_variant must be raw or filtered." >&2
    exit 2
    ;;
esac

if [ -n "${RUN_TAG}" ] \
    && [[ ! "${RUN_TAG}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "RUN_TAG may only contain letters, numbers, dot, underscore, and dash." >&2
  exit 2
fi

if [ "${SLAM_METHOD}" = "rtabmap" ]; then
  USE_CAMERA=${USE_CAMERA:-true}
else
  USE_CAMERA=${USE_CAMERA:-false}
fi
if [ "${SLAM_METHOD}" != "rtabmap" ] && [ "${USE_CAMERA}" = "true" ]; then
  echo "USE_CAMERA=true is supported only for RTAB-Map benchmarks." >&2
  exit 2
fi

if [ ! -f "${BAG_PATH}/metadata.yaml" ]; then
  echo "Bag path does not contain metadata.yaml: ${BAG_PATH}" >&2
  exit 1
fi
BAG_PATH=$(realpath "${BAG_PATH}")

# Rosbag directories produced by the robot recorder are usually named bag or
# bag_0. Use their uniquely named parent run directory in the benchmark name so
# results remain traceable to the source recording.
SOURCE_BAG_NAME=$(basename "${BAG_PATH}")
if [[ "${SOURCE_BAG_NAME}" = "bag" || "${SOURCE_BAG_NAME}" = bag_* ]]; then
  SOURCE_BAG_NAME=$(basename "$(dirname "${BAG_PATH}")")
fi
SOURCE_BAG_NAME=$(sed -E 's/[^A-Za-z0-9._-]+/_/g' <<<"${SOURCE_BAG_NAME}")
if [ -z "${SOURCE_BAG_NAME}" ]; then
  SOURCE_BAG_NAME=unnamed_bag
fi

USE_WAYPOINT_EVALUATOR=false
if [ -n "${WAYPOINTS_FILE}" ]; then
  if [ ! -f "${WAYPOINTS_FILE}" ]; then
    echo "Waypoint file does not exist: ${WAYPOINTS_FILE}" >&2
    exit 1
  fi
  WAYPOINTS_FILE=$(realpath "${WAYPOINTS_FILE}")
  USE_WAYPOINT_EVALUATOR=true
fi

RUN_BASE=bag_${SLAM_METHOD}_${SCAN_VARIANT}_${SOURCE_BAG_NAME}
if [ "${VELOCITY_SCALE}" != "1.0" ]; then
  RUN_BASE=${RUN_BASE}_vel${VELOCITY_SCALE}
fi
if [ "${STATIC_CONFIRMATIONS}" != "20" ]; then
  RUN_BASE=${RUN_BASE}_sc${STATIC_CONFIRMATIONS}
fi
if [ "${STATIC_SPEED_THRESHOLD}" != "0.06" ]; then
  RUN_BASE=${RUN_BASE}_sst${STATIC_SPEED_THRESHOLD}
fi
if [ "${MOVING_CONFIRMATIONS}" != "4" ]; then
  RUN_BASE=${RUN_BASE}_mc${MOVING_CONFIRMATIONS}
fi

allocate_run_directory() {
  local index candidate
  mkdir -p "${OUTPUT_ROOT}"
  for index in $(seq 1 9999); do
    candidate=$(printf '%s/%s_run%03d' "${OUTPUT_ROOT}" "${RUN_BASE}" "${index}")
    if mkdir "${candidate}" 2>/dev/null; then
      RUN_DIR=${candidate}
      RUN_NAME=$(basename "${candidate}")
      return 0
    fi
  done
  echo "Could not allocate a unique run directory for ${RUN_BASE}." >&2
  return 1
}

allocate_run_directory
RESOURCE_DIR=${RUN_DIR}/resources
METRICS_DIR=${RUN_DIR}/metrics
DATABASE_DIR=${RUN_DIR}/databases
LAUNCH_LOG=${RUN_DIR}/launch.log
RUN_CONFIG=${RUN_DIR}/run_config.txt
EVALUATION_OUTPUT=${METRICS_DIR}/waypoint_accuracy.json
RTABMAP_DATABASE=${DATABASE_DIR}/rtabmap.db
MONITOR_SCRIPT=${WORKSPACE}/scripts/monitor_robot_resources.py

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source_setup "${WORKSPACE}/install/setup.bash"
else
  echo "Workspace is not built: ${WORKSPACE}/install/setup.bash is missing." >&2
  exit 1
fi

for package in xgo_driver_bridge dynamic_scan_filter "${SLAM_METHOD}"; do
  case "${package}" in
    cartographer) package=cartographer_ros ;;
    rtabmap) package=rtabmap_slam ;;
  esac
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "Required ROS package is unavailable: ${package}" >&2
    exit 1
  fi
done

mkdir -p "${RESOURCE_DIR}" "${METRICS_DIR}" "${DATABASE_DIR}"
if ! ros2 bag info "${BAG_PATH}" > "${RUN_DIR}/bag_info.txt" 2>&1; then
  echo "ros2 bag info failed; the bag may be incomplete." >&2
  sed -n '1,120p' "${RUN_DIR}/bag_info.txt" >&2
  exit 1
fi

REQUIRED_TOPICS=(/scan /imu/data /xgo/applied_vel /tf_static)
if [ "${USE_CAMERA}" = "true" ]; then
  REQUIRED_TOPICS+=(/camera/image_raw/compressed)
fi
for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! grep -Fq "Topic: ${topic} " "${RUN_DIR}/bag_info.txt"; then
    echo "Required original-input topic is missing from the bag: ${topic}" >&2
    exit 1
  fi
done

# Do not mix this replay with a live robot stack or another mapper in the same
# ROS domain. Static topics from other robots can also contaminate a run.
NODE_LIST=$(timeout 5 ros2 node list 2>/dev/null || true)
if grep -Eq \
    'xgo_driver_bridge|slam_toolbox|cartographer|rtabmap|dynamic_scan_filter|xgo_offline_odom|foxglove_bridge' \
    <<<"${NODE_LIST}"; then
  echo "A robot/mapping pipeline is already visible in ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}:" >&2
  grep -E \
    'xgo_driver_bridge|slam_toolbox|cartographer|rtabmap|dynamic_scan_filter|xgo_offline_odom|foxglove_bridge' \
    <<<"${NODE_LIST}" >&2
  echo "Stop it or use an isolated ROS_DOMAIN_ID before benchmarking." >&2
  exit 1
fi

printf '%s\n' \
  "run_name=${RUN_NAME}" \
  "run_tag=${RUN_TAG}" \
  "mode=headless_input_only_bag_benchmark" \
  "slam_method=${SLAM_METHOD}" \
  "scan_variant=${SCAN_VARIANT}" \
  "use_dynamic_filter=${USE_DYNAMIC_FILTER}" \
  "normalize_scan=${NORMALIZE_SCAN}" \
  "use_camera=${USE_CAMERA}" \
  "start_foxglove=${START_FOXGLOVE}" \
  "foxglove_port=${FOXGLOVE_PORT}" \
  "playback_rate=${PLAYBACK_RATE}" \
  "velocity_scale=${VELOCITY_SCALE}" \
  "static_confirmations=${STATIC_CONFIRMATIONS}" \
  "static_speed_threshold=${STATIC_SPEED_THRESHOLD}" \
  "moving_confirmations=${MOVING_CONFIRMATIONS}" \
  "post_playback_delay_sec=${POST_PLAYBACK_DELAY_SEC}" \
  "bag_path=${BAG_PATH}" \
  "source_bag_name=${SOURCE_BAG_NAME}" \
  "replayed_topics=${REQUIRED_TOPICS[*]}" \
  "recorded_derived_topics_replayed=false" \
  "use_waypoint_evaluator=${USE_WAYPOINT_EVALUATOR}" \
  "waypoints_file=${WAYPOINTS_FILE}" \
  "resource_interval_s=${RESOURCE_INTERVAL}" \
  "ros_domain_id=${ROS_DOMAIN_ID:-unset}" \
  "started_at=$(date --iso-8601=seconds)" \
  > "${RUN_CONFIG}"

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
sleep 1
if ! kill -0 "${MONITOR_PID}" 2>/dev/null; then
  echo "CPU/RAM monitor exited before playback started." >&2
  cleanup
  exit 1
fi

cat <<EOF

Headless bag benchmark starting.
  Method:       ${SLAM_METHOD}
  Scan variant: ${SCAN_VARIANT}
  Camera:       ${USE_CAMERA}
  Foxglove:     ${START_FOXGLOVE}
  Bag:          ${BAG_PATH}
  Results:      ${RUN_DIR}
  Launch log:   ${LAUNCH_LOG}

The launch exits automatically after playback and the processing grace period.
EOF

SECONDS=0
set +e
ros2 launch xgo_driver_bridge headless_bag_benchmark.launch.py \
  bag_path:="${BAG_PATH}" \
  slam_method:="${SLAM_METHOD}" \
  use_dynamic_filter:="${USE_DYNAMIC_FILTER}" \
  normalize_scan:="${NORMALIZE_SCAN}" \
  use_camera:="${USE_CAMERA}" \
  playback_rate:="${PLAYBACK_RATE}" \
  post_playback_delay_sec:="${POST_PLAYBACK_DELAY_SEC}" \
  start_foxglove:="${START_FOXGLOVE}" \
  foxglove_port:="${FOXGLOVE_PORT}" \
  use_waypoint_evaluator:="${USE_WAYPOINT_EVALUATOR}" \
  waypoints_file:="${WAYPOINTS_FILE}" \
  evaluation_output_path:="${EVALUATION_OUTPUT}" \
  rtabmap_database_path:="${RTABMAP_DATABASE}" \
  velocity_scale:="${VELOCITY_SCALE}" \
  static_confirmations:="${STATIC_CONFIRMATIONS}" \
  static_speed_threshold:="${STATIC_SPEED_THRESHOLD}" \
  moving_confirmations:="${MOVING_CONFIRMATIONS}" \
  > "${LAUNCH_LOG}" 2>&1
LAUNCH_STATUS=$?
set -e
WALL_DURATION_SECONDS=${SECONDS}

cleanup
trap - EXIT INT TERM

printf '%s\n' \
  "ended_at=$(date --iso-8601=seconds)" \
  "wall_duration_s=${WALL_DURATION_SECONDS}" \
  "launch_exit_code=${LAUNCH_STATUS}" \
  >> "${RUN_CONFIG}"

if [ ! -s "${RESOURCE_DIR}/summary.json" ]; then
  echo "WARNING: resource summary was not created." >&2
fi
if [ "${USE_WAYPOINT_EVALUATOR}" = "true" ] \
    && [ ! -s "${EVALUATION_OUTPUT}" ]; then
  echo "WARNING: waypoint evaluation output was not created." >&2
fi

printf '\nBenchmark saved to %s\n' "${RUN_DIR}"
printf 'Resource summary: %s\n' "${RESOURCE_DIR}/summary.json"
printf 'Waypoint metrics: %s\n' "${EVALUATION_OUTPUT}"
printf 'Launch log: %s\n' "${LAUNCH_LOG}"

if [ "${LAUNCH_STATUS}" -ne 0 ]; then
  echo "The benchmark launch failed; inspect ${LAUNCH_LOG}." >&2
  exit "${LAUNCH_STATUS}"
fi