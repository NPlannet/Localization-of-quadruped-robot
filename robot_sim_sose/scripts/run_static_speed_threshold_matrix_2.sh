#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash scripts/run_static_speed_threshold_matrix_2.sh <bag_path> [waypoints.json]
#
# Runs, for the given bag:
#   3 algorithms (slam_toolbox, cartographer, rtabmap)
#   x scan_variant=filtered (static_speed_threshold only affects the filtered scan)
#   x 2 static_speed_threshold values (0.03, 0.09)
#   x 5 repetitions
#   = 30 calls to robot_bag_benchmark.sh
#
# Requires robot_bag_benchmark.sh to forward STATIC_SPEED_THRESHOLD (if set) as a
# dynamic_scan_filter node parameter override (-p static_speed_threshold:=<value>).

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "Usage: bash scripts/run_static_speed_threshold_matrix_2.sh <bag_path> [waypoints.json]" >&2
  exit 2
fi

BAG_PATH=$1
WAYPOINTS_FILE=${2:-}

PLAYBACK_RATE=${PLAYBACK_RATE:-5.0}
ALGORITHMS=(slam_toolbox cartographer rtabmap)
SCAN_VARIANT=filtered
STATIC_SPEED_THRESHOLD_VALUES=(0.03 0.09)
REPETITIONS=${REPETITIONS:-5}

TOTAL=$(( ${#ALGORITHMS[@]} * ${#STATIC_SPEED_THRESHOLD_VALUES[@]} * REPETITIONS ))
COUNT=0

for algorithm in "${ALGORITHMS[@]}"; do
  for static_speed_threshold in "${STATIC_SPEED_THRESHOLD_VALUES[@]}"; do
    for rep in $(seq 1 "${REPETITIONS}"); do
      COUNT=$((COUNT + 1))
      echo ""
      echo "=== [${COUNT}/${TOTAL}] algorithm=${algorithm} scan=${SCAN_VARIANT} static_speed_threshold=${static_speed_threshold} rep=${rep} ==="
      STATIC_SPEED_THRESHOLD="${static_speed_threshold}" \
      PLAYBACK_RATE="${PLAYBACK_RATE}" \
      RUN_TAG="sst${static_speed_threshold}_rep${rep}" \
      bash scripts/robot_bag_benchmark.sh \
        "${algorithm}" "${SCAN_VARIANT}" "${BAG_PATH}" "${WAYPOINTS_FILE}"
    done
  done
done

echo ""
echo "static_speed_threshold matrix complete: ${COUNT} runs."