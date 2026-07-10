#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
exec bash "${SCRIPT_DIR}/../deployment/scripts/robot_build_workspace.sh" "$@"
