#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash scripts/run_velocity_matrix.sh <bag_path> [waypoints.json]
#
# Runs, for the given bag:
#   3 algorithms (slam_toolbox, cartographer, rtabmap)
#   x 2 scan variants (raw, filtered)
#   x 4 velocity scales (0.7, 0.8, 0.9, 1.0)
#   x 5 repetitions
#   = 120 calls to robot_bag_benchmark.sh
#
# PLAYBACK_RATE can be overridden via environment (default 4.0, per your plan).

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "Usage: bash scripts/run_velocity_matrix.sh <bag_path> [waypoints.json]" >&2
  exit 2
fi

BAG_PATH=$1
WAYPOINTS_FILE=${2:-}

PLAYBACK_RATE=${PLAYBACK_RATE:-5.0}
ALGORITHMS=(slam_toolbox cartographer rtabmap)
SCAN_VARIANTS=(raw filtered)
VELOCITY_SCALES=(0.7 0.8 0.9 1.0)
REPETITIONS=${REPETITIONS:-5}

TOTAL=$(( ${#ALGORITHMS[@]} * ${#SCAN_VARIANTS[@]} * ${#VELOCITY_SCALES[@]} * REPETITIONS ))
COUNT=0

for algorithm in "${ALGORITHMS[@]}"; do
  for scan_variant in "${SCAN_VARIANTS[@]}"; do
    for velocity_scale in "${VELOCITY_SCALES[@]}"; do
      for rep in $(seq 1 "${REPETITIONS}"); do
        COUNT=$((COUNT + 1))
        echo ""
        echo "=== [${COUNT}/${TOTAL}] algorithm=${algorithm} scan=${scan_variant} velocity_scale=${velocity_scale} rep=${rep} ==="
        VELOCITY_SCALE="${velocity_scale}" \
        PLAYBACK_RATE="${PLAYBACK_RATE}" \
        RUN_TAG="rep${rep}" \
        bash scripts/robot_bag_benchmark.sh \
          "${algorithm}" "${scan_variant}" "${BAG_PATH}" "${WAYPOINTS_FILE}"
      done
    done
  done
done

echo ""
echo "Velocity matrix complete: ${COUNT} runs."