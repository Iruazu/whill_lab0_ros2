#!/usr/bin/env bash
# Initialize a colcon workspace at ~/ros2_ws and append ROS 2 sourcing to .bashrc.
#
# Idempotent — safe to re-run.

set -euo pipefail

WS_ROOT="${WS_ROOT:-${HOME}/ros2_ws}"

create_workspace() {
  mkdir -p "${WS_ROOT}/src"
  echo "workspace ready: ${WS_ROOT}"
}

initialize_rosdep() {
  if ! command -v rosdep > /dev/null; then
    echo "ERROR: rosdep not found. Install ros-dev-tools first." >&2
    exit 1
  fi
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
}

append_bashrc() {
  local rc=~/.bashrc
  local marker="# >>> ROS 2 humble (whill_lab0_ros2) >>>"
  if grep -qF "${marker}" "${rc}"; then
    echo ".bashrc already configured — leaving as-is"
    return 0
  fi
  cat >> "${rc}" <<EOF

${marker}
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f ${WS_ROOT}/install/setup.bash ]; then
    source ${WS_ROOT}/install/setup.bash
fi
# <<< ROS 2 humble (whill_lab0_ros2) <<<
EOF
  echo "appended ROS 2 sourcing to ${rc}"
}

main() {
  create_workspace
  initialize_rosdep
  append_bashrc
  echo "Done. Open a new shell or run: source ~/.bashrc"
}

main "$@"
