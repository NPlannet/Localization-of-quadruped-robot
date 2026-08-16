#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash scripts/rerun_failed_combos.sh
#
# Re-runs REPETITIONS fresh repetitions for a list of failed combos.
# Each combo specifies algorithm, scan_variant, bag_name, and an arbitrary
# set of parameter overrides (any of VELOCITY_SCALE, STATIC_CONFIRMATIONS,
# STATIC_SPEED_THRESHOLD, MOVING_CONFIRMATIONS — extend SUPPORTED_PARAMS
# below if robot_bag_benchmark.sh grows more, e.g. MOVING_SPEED_THRESHOLD).
#
# Existing run directories are never overwritten: robot_bag_benchmark.sh
# always allocates a new, unique run directory, so this just adds
# REPETITIONS more runs on top of whatever is already there for each combo.

RECORDED_RUNS_DIR=${RECORDED_RUNS_DIR:-evaluation/recorded_runs}
PLAYBACK_RATE=${PLAYBACK_RATE:-5.0}
REPETITIONS=${REPETITIONS:-5}

# Env vars that robot_bag_benchmark.sh actually understands as overrides.
# Add MOVING_SPEED_THRESHOLD here (and to robot_bag_benchmark.sh) if/when
# that parameter exists.
SUPPORTED_PARAMS=(
  VELOCITY_SCALE
  STATIC_CONFIRMATIONS
  STATIC_SPEED_THRESHOLD
  MOVING_CONFIRMATIONS
  MOVING_SPEED_THRESHOLD
)

# Each entry: algorithm:scan_variant:bag_name:param1=val1,param2=val2,...
# Only params you list are overridden; anything you omit keeps the script's
# own default (VELOCITY_SCALE=1.0, STATIC_CONFIRMATIONS=20,
# STATIC_SPEED_THRESHOLD=0.06, MOVING_CONFIRMATIONS=4).
COMBOS=(
  "slam_toolbox:filtered:rtab_filtered_run_wp_3:MOVING_SPEED_THRESHOLD=0.05"
  "rtabmap:filtered:rtab_filtered_run_wp_3:MOVING_SPEED_THRESHOLD=0.05"
  "cartographer:filtered:rtab_filtered_run_wp_3:MOVING_SPEED_THRESHOLD=0.05"
  
  # Examples mixing several override parameters at once:
  # "rtabmap:raw:rtabmap_dynamic_filtered_run1:VELOCITY_SCALE=0.8,STATIC_CONFIRMATIONS=30"
  # "cartographer:filtered:rtabmap_semistatic_filtered_run1:MOVING_CONFIRMATIONS=6,STATIC_SPEED_THRESHOLD=0.08"
)

TOTAL=$(( ${#COMBOS[@]} * REPETITIONS ))
COUNT=0

is_supported_param() {
  local name=$1 candidate
  for candidate in "${SUPPORTED_PARAMS[@]}"; do
    if [ "${candidate}" = "${name}" ]; then
      return 0
    fi
  done
  return 1
}

run_bag() {
  local algorithm=$1 scan_variant=$2 bag_name=$3 param_str=$4
  local bag_path="${RECORDED_RUNS_DIR}/${bag_name}/bag"
  local waypoints_file="${RECORDED_RUNS_DIR}/${bag_name}/waypoints.json"

  if [ ! -f "${bag_path}/metadata.yaml" ]; then
    echo "SKIP: bag not found at ${bag_path}" >&2
    return
  fi
  if [ ! -f "${waypoints_file}" ]; then
    echo "SKIP: waypoints not found at ${waypoints_file}" >&2
    return
  fi

  # Build an array of NAME=VALUE env assignments to prefix the benchmark call.
  local -a overrides=()
  local label_parts=""
  if [ -n "${param_str}" ]; then
    local pair name value
    IFS=',' read -ra pairs <<< "${param_str}"
    for pair in "${pairs[@]}"; do
      name=${pair%%=*}
      value=${pair#*=}
      if ! is_supported_param "${name}"; then
        echo "ERROR: unsupported parameter '${name}' in combo '${algorithm}:${scan_variant}:${bag_name}:${param_str}'." >&2
        echo "       Supported: ${SUPPORTED_PARAMS[*]}" >&2
        exit 2
      fi
      overrides+=("${name}=${value}")
      label_parts="${label_parts}_${name,,}${value}"
    done
  fi

  for rep in $(seq 1 "${REPETITIONS}"); do
    COUNT=$((COUNT + 1))
    echo ""
    echo "=== [${COUNT}/${TOTAL}] algorithm=${algorithm} bag=${bag_name} scan=${scan_variant} overrides=${param_str:-none} rep=${rep} ==="
    env "${overrides[@]}" \
      PLAYBACK_RATE="${PLAYBACK_RATE}" \
      RUN_TAG="retry${label_parts}_rep${rep}" \
      bash scripts/robot_bag_benchmark.sh \
        "${algorithm}" "${scan_variant}" "${bag_path}" "${waypoints_file}"
  done
}

for combo in "${COMBOS[@]}"; do
  IFS=':' read -r algorithm scan_variant bag_name param_str <<< "${combo}"
  run_bag "${algorithm}" "${scan_variant}" "${bag_name}" "${param_str}"
done

echo ""
echo "Rerun complete: ${COUNT} runs."