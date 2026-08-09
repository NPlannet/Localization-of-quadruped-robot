#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash scripts/run_moving_confirmations_matrix.sh <bag_path> [waypoints.json]
#
# Runs, for the given bag:
#   3 algorithms (slam_toolbox, cartographer, rtabmap)
#   x scan_variant=filtered (moving_confirmations only affects the filtered scan)
#   x 3 moving_confirmations values (2, 6, 8)
#   x 5 repetitions
#   = 45 calls to robot_bag_benchmark.sh
#
# Requires robot_bag_benchmark.sh to forward MOVING_CONFIRMATIONS (if set) as a
# dynamic_scan_filter node parameter override (-p moving_confirmations:=<value>).

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "Usage: bash scripts/run_moving_confirmations_matrix.sh <bag_path> [waypoints.json]" >&2
  exit 2
fi

BAG_PATH=$1
WAYPOINTS_FILE=${2:-}

PLAYBACK_RATE=${PLAYBACK_RATE:-5.0}
ALGORITHMS=(slam_toolbox cartographer rtabmap)
SCAN_VARIANT=filtered
MOVING_CONFIRMATIONS_VALUES=(2 6 8)
REPETITIONS=${REPETITIONS:-5}

TOTAL=$(( ${#ALGORITHMS[@]} * ${#MOVING_CONFIRMATIONS_VALUES[@]} * REPETITIONS ))
COUNT=0

for algorithm in "${ALGORITHMS[@]}"; do
  for moving_confirmations in "${MOVING_CONFIRMATIONS_VALUES[@]}"; do
    for rep in $(seq 1 "${REPETITIONS}"); do
      COUNT=$((COUNT + 1))
      echo ""
      echo "=== [${COUNT}/${TOTAL}] algorithm=${algorithm} scan=${SCAN_VARIANT} moving_confirmations=${moving_confirmations} rep=${rep} ==="
      MOVING_CONFIRMATIONS="${moving_confirmations}" \
      PLAYBACK_RATE="${PLAYBACK_RATE}" \
      RUN_TAG="mc${moving_confirmations}_rep${rep}" \
      bash scripts/robot_bag_benchmark.sh \
        "${algorithm}" "${SCAN_VARIANT}" "${BAG_PATH}" "${WAYPOINTS_FILE}"
    done
  done
done

echo ""
echo "moving_confirmations matrix complete: ${COUNT} runs."