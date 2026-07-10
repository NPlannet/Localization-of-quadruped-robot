#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH=$(readlink -f "$0")
INITIAL_CWD=$(pwd -P)
STATE_DIR=""
ACTIVE_MARKER=""
ROBOT_PASSWORD_FILE=""
ROBOT_TARGET=""
ROBOT_REMOTE_DIR=""
ROBOT_REMOTE_STATE_DIR=""
ROBOT_REMOTE_PASSWORD_FILE=""
CLEANUP_NEEDED=false
WSL_DISTRO=${WSL_DISTRO_NAME:-}

usage() {
  cat <<'EOF'
Usage:
  bash scripts/robot_nav_tmux.sh <ssh-target> [options]

Examples:
  bash scripts/robot_nav_tmux.sh pi@robodoge2.local
  bash scripts/robot_nav_tmux.sh robodoge2.local --skip-build --no-foxglove
  bash scripts/robot_nav_tmux.sh pi@robodoge2.local --terminal-app gnome-terminal --terminal-layout tabs
  bash scripts/robot_nav_tmux.sh pi@robodoge2.local --terminal-app gnome-terminal
  bash scripts/robot_nav_tmux.sh pi@robodoge2.local --terminal-app windows-terminal --terminal-layout panes

Options:
  --remote-dir <dir>      Remote repo path on the robot (default: /home/pi/robot_sim_sose)
  --terminal-app <app>    Terminal backend: windows-terminal, gnome-terminal,
                          mate-terminal, xfce4-terminal, konsole, or xterm
  --terminal-layout <m>   Layout: auto, windows, tabs, or panes
  --skip-build            Skip bash scripts/robot_build_workspace.sh before bring-up
  --no-foxglove           Do not open a Foxglove/Lichtblick terminal
  --bridge-only           Start xgo-bridge instead of xgo-motion
  --help                  Show this help

What it does:
  1. Prompts once for the robot password.
  2. Starts the Docker container on the robot.
  3. Optionally builds the ROS workspace once inside the container.
  4. Opens robot stack terminals in separate windows, tabs, or panes.
  5. Leaves each terminal usable after you stop that node with Ctrl+C.

Notes:
  - The file name is kept for compatibility, but it no longer uses tmux.
  - It assumes the robot uses the same password for ssh and sudo.
  - On WSL with WSLg, it prefers Linux terminal apps like gnome-terminal before Windows Terminal.

Requirements on your laptop:
  ssh
  scp
  sshpass
  one supported terminal app
EOF
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

quote_shell() {
  printf '%q' "$1"
}

shell_join() {
  local joined
  printf -v joined '%q ' "$@"
  printf '%s' "${joined% }"
}

detect_terminal_app() {
  local requested=${1:-}
  local candidate

  if [ -n "${requested}" ]; then
    case "${requested}" in
      windows-terminal|wt|wt.exe)
        require_command cmd.exe
        require_command wt.exe
        require_command wsl.exe
        if [ -z "${WSL_DISTRO}" ]; then
          echo "windows-terminal is only supported from inside WSL." >&2
          exit 1
        fi
        printf '%s\n' "windows-terminal"
        return 0
        ;;
      *)
        if command -v "${requested}" >/dev/null 2>&1; then
          printf '%s\n' "${requested}"
          return 0
        fi

        echo "Requested terminal app is not installed: ${requested}" >&2
        exit 1
        ;;
    esac
  fi

  for candidate in gnome-terminal mate-terminal xfce4-terminal konsole xterm; do
    if command -v "${candidate}" >/dev/null 2>&1; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  if [ -n "${WSL_DISTRO}" ] && command -v cmd.exe >/dev/null 2>&1 && command -v wt.exe >/dev/null 2>&1; then
    printf '%s\n' "windows-terminal"
    return 0
  fi

  echo "No supported terminal app found." >&2
  echo "Install Windows Terminal, gnome-terminal, or xterm, or pass --terminal-app <app>." >&2
  exit 1
}

detect_terminal_layout() {
  local terminal_app=$1
  local requested=${2:-auto}

  case "${requested}" in
    auto)
      case "${terminal_app}" in
        windows-terminal)
          printf '%s\n' "panes"
          ;;
        gnome-terminal|mate-terminal)
          printf '%s\n' "tabs"
          ;;
        *)
          printf '%s\n' "windows"
          ;;
      esac
      ;;
    windows)
      printf '%s\n' "windows"
      ;;
    tabs)
      case "${terminal_app}" in
        windows-terminal|gnome-terminal|mate-terminal)
          printf '%s\n' "tabs"
          ;;
        *)
          echo "Terminal app ${terminal_app} does not support tab layout." >&2
          exit 1
          ;;
      esac
      ;;
    panes)
      case "${terminal_app}" in
        windows-terminal)
          printf '%s\n' "panes"
          ;;
        *)
          echo "Pane layout is currently only supported with windows-terminal." >&2
          exit 1
          ;;
      esac
      ;;
    *)
      echo "Unknown terminal layout: ${requested}" >&2
      exit 1
      ;;
  esac
}

require_local_display_if_needed() {
  local terminal_app=$1

  case "${terminal_app}" in
    windows-terminal)
      return 0
      ;;
  esac

  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "No GUI display detected." >&2
    echo "Run this from your desktop session, not from a headless shell." >&2
    exit 1
  fi
}

run_remote() {
  local remote_cmd=$1

  sshpass -f "${ROBOT_PASSWORD_FILE}" \
    ssh -o StrictHostKeyChecking=accept-new "${ROBOT_TARGET}" "${remote_cmd}"
}

run_remote_sudo() {
  local inner_cmd=$1
  local remote_cmd

  remote_cmd="cd $(quote_shell "${ROBOT_REMOTE_DIR}") && sudo -S -p '' bash -lc $(quote_shell "${inner_cmd}") < $(quote_shell "${ROBOT_REMOTE_PASSWORD_FILE}")"
  run_remote "${remote_cmd}"
}

run_remote_interactive_sudo() {
  local inner_cmd=$1
  local remote_cmd
  local root_cmd

  root_cmd="exec </dev/tty >/dev/tty 2>/dev/tty; ${inner_cmd}"
  remote_cmd="cd $(quote_shell "${ROBOT_REMOTE_DIR}") && sudo -S -p '' bash -lc $(quote_shell "${root_cmd}") < $(quote_shell "${ROBOT_REMOTE_PASSWORD_FILE}")"

  sshpass -f "${ROBOT_PASSWORD_FILE}" \
    ssh -tt -o StrictHostKeyChecking=accept-new "${ROBOT_TARGET}" "${remote_cmd}"
}

prepare_remote_password_file() {
  local stamp

  stamp="$(date +%Y%m%d-%H%M%S)-$$-$RANDOM"
  ROBOT_REMOTE_STATE_DIR="/tmp/robot-nav-${stamp}"
  ROBOT_REMOTE_PASSWORD_FILE="${ROBOT_REMOTE_STATE_DIR}/password"

  run_remote "umask 077 && mkdir -p $(quote_shell "${ROBOT_REMOTE_STATE_DIR}")"

  sshpass -f "${ROBOT_PASSWORD_FILE}" \
    scp -q -o StrictHostKeyChecking=accept-new \
    "${ROBOT_PASSWORD_FILE}" "${ROBOT_TARGET}:${ROBOT_REMOTE_PASSWORD_FILE}"

  run_remote "chmod 600 $(quote_shell "${ROBOT_REMOTE_PASSWORD_FILE}")"
}

cleanup_remote_password_file() {
  if [ -z "${ROBOT_REMOTE_STATE_DIR:-}" ]; then
    return 0
  fi

  if [ ! -f "${ROBOT_PASSWORD_FILE:-}" ]; then
    return 0
  fi

  sshpass -f "${ROBOT_PASSWORD_FILE}" \
    ssh -o StrictHostKeyChecking=accept-new "${ROBOT_TARGET}" \
    "rm -rf $(quote_shell "${ROBOT_REMOTE_STATE_DIR}")" >/dev/null 2>&1 || true
}

write_state_file() {
  cat > "${STATE_DIR}/state.sh" <<EOF
ROBOT_TARGET=$(quote_shell "${ROBOT_TARGET}")
ROBOT_REMOTE_DIR=$(quote_shell "${ROBOT_REMOTE_DIR}")
ROBOT_PASSWORD_FILE=$(quote_shell "${ROBOT_PASSWORD_FILE}")
ROBOT_REMOTE_STATE_DIR=$(quote_shell "${ROBOT_REMOTE_STATE_DIR}")
ROBOT_REMOTE_PASSWORD_FILE=$(quote_shell "${ROBOT_REMOTE_PASSWORD_FILE}")
EOF
  chmod 600 "${STATE_DIR}/state.sh"
}

touch_pending_role_files() {
  local role

  for role in "$@"; do
    : > "${STATE_DIR}/pending.${role}"
  done
}

start_cleanup_watcher() {
  (
    while [ -d "${STATE_DIR}" ]; do
      shopt -s nullglob
      markers=("${STATE_DIR}"/pending.* "${STATE_DIR}"/active.*)
      shopt -u nullglob

      if [ "${#markers[@]}" -eq 0 ]; then
        cleanup_remote_password_file
        rm -rf "${STATE_DIR}"
        exit 0
      fi

      sleep 5
    done
  ) >/dev/null 2>&1 &
}

terminal_container_command() {
  local role=$1
  local container_cmd

  case "${role}" in
    xgo-motion|xgo-bridge|lidar|camera|filter|slam|nav2|foxglove)
      container_cmd=$(cat <<EOF
cd /workspaces/robot_sim_sose
source /opt/ros/jazzy/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
bash scripts/robot_stack.sh ${role}
status=\$?
echo
if [ \$status -eq 0 ]; then
  echo "[${role}] Command exited cleanly. Staying in the container shell."
else
  echo "[${role}] Command exited with status \$status. Staying in the container shell."
fi
exec bash -i
EOF
)
      printf '%s' \
        "docker compose -f docker-compose.robot.yml exec xgo-robot bash -ic $(quote_shell "${container_cmd}")"
      ;;
    *)
      echo "Unknown terminal role: ${role}" >&2
      exit 1
      ;;
  esac
}

mark_role_active() {
  local role=$1

  ACTIVE_MARKER="${STATE_DIR}/active.${role}.$$"
  rm -f "${STATE_DIR}/pending.${role}"
  : > "${ACTIVE_MARKER}"
  trap 'rm -f "${ACTIVE_MARKER}"' EXIT
}

terminal_mode() {
  if [ $# -ne 2 ]; then
    echo "Usage: $0 --terminal <state-dir> <role>" >&2
    exit 2
  fi

  STATE_DIR=$1
  ROLE=$2

  if [ ! -f "${STATE_DIR}/state.sh" ]; then
    echo "Missing launcher state: ${STATE_DIR}/state.sh" >&2
    exit 1
  fi

  # shellcheck disable=SC1090
  source "${STATE_DIR}/state.sh"
  mark_role_active "${ROLE}"

  echo "[${ROLE}] Connecting to ${ROBOT_TARGET}"
  echo "[${ROLE}] Stop the running node with Ctrl+C when you need to."
  echo "[${ROLE}] After that, this terminal stays inside the container shell."
  echo

  run_remote_interactive_sudo "$(terminal_container_command "${ROLE}")"
}

terminal_runner_command() {
  local state_dir=$1
  local role=$2
  local runner

  runner="$(shell_join bash "${SCRIPT_PATH}" --terminal "${state_dir}" "${role}")"

  printf '%s' \
    "${runner}; status=\$?; echo; if [ \$status -ne 0 ]; then echo \"[${role}] Session ended with status \$status.\"; fi; echo \"Local shell stays open.\"; exec bash -i"
}

windows_terminal_shell_command() {
  local role=$1

  terminal_runner_command "${STATE_DIR}" "${role}"
}

launch_windows_terminal_tabs() {
  local index=1
  local role
  local title
  local -a cmd

  cmd=(cmd.exe /c wt.exe -w -1)

  for role in "$@"; do
    printf -v title 'robot-nav %02d %s' "${index}" "${role}"

    if [ "${index}" -gt 1 ]; then
      cmd+=("\\;" "new-tab")
    else
      cmd+=("new-tab")
    fi

    cmd+=(
      "--title" "${title}"
      "wsl.exe" "-d" "${WSL_DISTRO}" "--cd" "${INITIAL_CWD}"
      "bash" "-lc" "$(windows_terminal_shell_command "${role}")"
    )
    index=$((index + 1))
  done

  "${cmd[@]}"
}

launch_windows_terminal_panes() {
  local index=1
  local role
  local title
  local orientation
  local -a cmd
  local -a orientations=("--vertical" "--horizontal")

  cmd=(cmd.exe /c wt.exe -w -1)

  for role in "$@"; do
    printf -v title 'robot-nav %02d %s' "${index}" "${role}"

    if [ "${index}" -eq 1 ]; then
      cmd+=(
        "new-tab"
        "--title" "${title}"
        "wsl.exe" "-d" "${WSL_DISTRO}" "--cd" "${INITIAL_CWD}"
        "bash" "-lc" "$(windows_terminal_shell_command "${role}")"
      )
    else
      orientation=${orientations[$(( (index - 2) % ${#orientations[@]} ))]}
      cmd+=(
        "\\;" "split-pane" "${orientation}"
        "--title" "${title}"
        "wsl.exe" "-d" "${WSL_DISTRO}" "--cd" "${INITIAL_CWD}"
        "bash" "-lc" "$(windows_terminal_shell_command "${role}")"
      )
    fi

    index=$((index + 1))
  done

  "${cmd[@]}"
}

launch_tabbed_terminals() {
  local terminal_app=$1
  shift

  local index=1
  local role
  local title
  local runner_cmd
  local -a cmd

  case "${terminal_app}" in
    windows-terminal)
      launch_windows_terminal_tabs "$@"
      return 0
      ;;
    gnome-terminal|mate-terminal)
      cmd=("${terminal_app}" "--window")
      ;;
    *)
      echo "Tabbed layout is not supported with ${terminal_app}." >&2
      exit 1
      ;;
  esac

  for role in "$@"; do
    printf -v title 'robot-nav %02d %s' "${index}" "${role}"
    runner_cmd="$(terminal_runner_command "${STATE_DIR}" "${role}")"

    if [ "${index}" -gt 1 ]; then
      cmd+=("--tab")
    fi

    case "${terminal_app}" in
      gnome-terminal|mate-terminal)
        cmd+=(
          "--title=${title}"
          "--command=$(shell_join bash -lc "${runner_cmd}")"
        )
        ;;
    esac
    index=$((index + 1))
  done

  "${cmd[@]}"
}

launch_terminal_window() {
  local terminal_app=$1
  local title=$2
  local runner_cmd=$3

  case "${terminal_app}" in
    windows-terminal)
      cmd.exe /c wt.exe -w -1 new-tab \
        --title "${title}" \
        wsl.exe -d "${WSL_DISTRO}" --cd "${INITIAL_CWD}" \
        bash -lc "${runner_cmd}"
      ;;
    gnome-terminal)
      gnome-terminal --window --title="${title}" -- bash -lc "${runner_cmd}"
      ;;
    mate-terminal)
      mate-terminal --window --title="${title}" -- bash -lc "${runner_cmd}"
      ;;
    xfce4-terminal)
      xfce4-terminal --title="${title}" -x bash -lc "${runner_cmd}"
      ;;
    konsole)
      konsole --hold -p tabtitle="${title}" -e bash -lc "${runner_cmd}"
      ;;
    xterm)
      xterm -T "${title}" -hold -e bash -lc "${runner_cmd}"
      ;;
    *)
      echo "Unsupported terminal app: ${terminal_app}" >&2
      exit 1
      ;;
  esac
}

launch_terminals() {
  local terminal_app=$1
  local terminal_layout=$2
  shift
  shift

  case "${terminal_layout}" in
    panes)
      launch_windows_terminal_panes "$@"
      return 0
      ;;
    tabs)
      launch_tabbed_terminals "${terminal_app}" "$@"
      return 0
      ;;
    windows)
      ;;
    *)
      echo "Unsupported terminal layout: ${terminal_layout}" >&2
      exit 1
      ;;
  esac

  local index=1
  local role
  local title
  local runner_cmd

  for role in "$@"; do
    printf -v title 'robot-nav %02d %s' "${index}" "${role}"
    runner_cmd="$(terminal_runner_command "${STATE_DIR}" "${role}")"
    launch_terminal_window "${terminal_app}" "${title}" "${runner_cmd}"
    sleep 0.3
    index=$((index + 1))
  done
}

cleanup_local_state() {
  cleanup_remote_password_file

  if [ -n "${STATE_DIR:-}" ] && [ -d "${STATE_DIR}" ]; then
    rm -rf "${STATE_DIR}"
  fi
}

main() {
  local target=""
  local remote_dir="/home/pi/robot_sim_sose"
  local build_workspace=true
  local include_foxglove=true
  local bridge_role="xgo-motion"
  local terminal_app=""
  local terminal_layout="auto"

  while [ $# -gt 0 ]; do
    case "$1" in
      --remote-dir)
        remote_dir=$2
        shift 2
        ;;
      --terminal-app)
        terminal_app=$2
        shift 2
        ;;
      --terminal-layout)
        terminal_layout=$2
        shift 2
        ;;
      --skip-build)
        build_workspace=false
        shift
        ;;
      --no-foxglove)
        include_foxglove=false
        shift
        ;;
      --bridge-only)
        bridge_role="xgo-bridge"
        shift
        ;;
      --help|-h)
        usage
        exit 0
        ;;
      --terminal)
        shift
        terminal_mode "$@"
        exit 0
        ;;
      -*)
        echo "Unknown option: $1" >&2
        usage
        exit 2
        ;;
      *)
        if [ -n "${target}" ]; then
          echo "Only one ssh target is supported." >&2
          exit 2
        fi
        target=$1
        shift
        ;;
    esac
  done

  if [ -z "${target}" ]; then
    usage
    exit 2
  fi

  require_command ssh
  require_command scp
  require_command sshpass
  terminal_app="$(detect_terminal_app "${terminal_app}")"
  terminal_layout="$(detect_terminal_layout "${terminal_app}" "${terminal_layout}")"
  require_local_display_if_needed "${terminal_app}"

  if [[ "${target}" != *@* && "${target}" != *:* ]]; then
    target="pi@${target}"
  fi

  local robot_password
  read -rsp "Robot password (used for ssh and sudo): " robot_password
  echo

  if [ -z "${robot_password}" ]; then
    echo "Password cannot be empty." >&2
    exit 1
  fi

  STATE_DIR=$(mktemp -d /tmp/robot-nav.XXXXXX)
  chmod 700 "${STATE_DIR}"
  CLEANUP_NEEDED=true

  trap 'status=$?; trap - EXIT; if [ "${CLEANUP_NEEDED}" = true ]; then cleanup_local_state; fi; exit "${status}"' EXIT

  ROBOT_PASSWORD_FILE="${STATE_DIR}/password"
  printf '%s\n' "${robot_password}" > "${ROBOT_PASSWORD_FILE}"
  chmod 600 "${ROBOT_PASSWORD_FILE}"
  unset robot_password

  ROBOT_TARGET=${target}
  ROBOT_REMOTE_DIR=${remote_dir}

  prepare_remote_password_file
  write_state_file

  echo "Starting robot container on ${ROBOT_TARGET}..."
  run_remote_sudo "docker compose -f docker-compose.robot.yml up -d"

  if [ "${build_workspace}" = true ]; then
    echo "Building workspace inside the container once before bring-up..."
    run_remote_sudo \
      "docker compose -f docker-compose.robot.yml exec xgo-robot bash -lc 'cd /workspaces/robot_sim_sose && bash scripts/robot_build_workspace.sh'"
  fi

  local roles=("${bridge_role}" "lidar" "camera" "filter" "slam" "nav2")
  if [ "${include_foxglove}" = true ]; then
    roles+=("foxglove")
  fi

  touch_pending_role_files "${roles[@]}"

  echo "Launching robot stack terminals with ${terminal_app} (${terminal_layout})..."
  launch_terminals "${terminal_app}" "${terminal_layout}" "${roles[@]}"
  start_cleanup_watcher

  CLEANUP_NEEDED=false

  cat <<EOF

Robot terminals are launching now.
Opened roles:
  ${bridge_role}
  lidar
  camera
  filter
  slam
  nav2
$(if [ "${include_foxglove}" = true ]; then printf '  foxglove\n'; fi)
What to expect:
  - Each terminal connects to the robot automatically.
  - Each terminal enters the Docker container automatically.
  - Ctrl+C stops that role and leaves you in the container shell.
  - Terminal backend: ${terminal_app}
  - Layout: ${terminal_layout}
EOF
}

main "$@"
