#!/usr/bin/env bash
# Import upstream packages declared in whill_lab.repos into the colcon workspace.
#
# Idempotent — safe to re-run; vcstool will fast-forward existing clones.
#
# Usage:
#   ./scripts/import_upstream.sh [path/to/ros2_ws]
#
# Defaults to using THIS repo as the workspace (treats the repo root as
# `<workspace>` and pulls upstream into `./src/third_party/`).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="${1:-${REPO_ROOT}}"
REPOS_FILE="${REPO_ROOT}/whill_lab.repos"

require_tools() {
  for tool in vcs rosdep colcon; do
    if ! command -v "${tool}" > /dev/null; then
      echo "ERROR: '${tool}' not found. Run scripts/install_ros2_humble.sh first." >&2
      exit 1
    fi
  done
}

import_repos() {
  if [[ ! -f "${REPOS_FILE}" ]]; then
    echo "ERROR: ${REPOS_FILE} not found." >&2
    exit 1
  fi
  mkdir -p "${WS_ROOT}/src"
  vcs import "${WS_ROOT}/src" < "${REPOS_FILE}"
}

resolve_deps() {
  # rosdep needs HTTP_PROXY / HTTPS_PROXY when on a proxied network — sudo -E
  # in case rosdep needs to apt-install missing deps.
  source /opt/ros/humble/setup.bash
  sudo -E rosdep install --from-paths "${WS_ROOT}/src" --ignore-src -r -y
}

main() {
  require_tools
  echo "[1/2] vcs import"
  import_repos
  echo "[2/2] rosdep install"
  resolve_deps
  echo "Done. Build with:  colcon build --packages-up-to whill --symlink-install"
}

main "$@"
