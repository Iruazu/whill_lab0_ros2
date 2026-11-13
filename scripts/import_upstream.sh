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

ignore_patchwork_subpackages() {
  # Patchwork++ ships three CMake subdirectories (ros/, cpp/, python/), each
  # with its own CMakeLists.txt, so colcon discovers all three as packages.
  # None of them should be built here:
  #   - cpp/    is the BSD-2-Clause core we DO use, but whill_perception
  #             consumes it via add_subdirectory(), never as a colcon package
  #   - ros/    declares GPL-3.0 in its package.xml; the permissive-only
  #             policy (ADR-0011) keeps it out of the tree's build entirely
  #   - python/ is the pip binding, unused here
  # Doing this in the script rather than leaving it as a manual step in
  # whill_lab.repos: a forgotten `touch` only shows up as a confusing build
  # error (or a GPL package silently built) on someone else's clone.
  local base="${WS_ROOT}/src/third_party/patchwork_plusplus"
  if [[ ! -d "${base}" ]]; then
    return 0
  fi
  for sub in ros cpp python; do
    if [[ -d "${base}/${sub}" ]]; then
      touch "${base}/${sub}/COLCON_IGNORE"
    fi
  done
}

resolve_deps() {
  # rosdep needs HTTP_PROXY / HTTPS_PROXY when on a proxied network — sudo -E
  # in case rosdep needs to apt-install missing deps.
  source /opt/ros/humble/setup.bash
  sudo -E rosdep install --from-paths "${WS_ROOT}/src" --ignore-src -r -y
}

main() {
  require_tools
  echo "[1/3] vcs import"
  import_repos
  echo "[2/3] mark non-colcon upstream subdirectories"
  ignore_patchwork_subpackages
  echo "[3/3] rosdep install"
  resolve_deps
  cat <<'EOF'
Done. Build the on-vehicle stack with:

  colcon build --symlink-install --packages-up-to \
    whill_sensors_bringup whill_localization whill_navigation \
    whill_perception whill_safety whill_dispatch

(`--packages-up-to whill` builds only the upstream WHILL driver metapackage —
 none of this repo's own packages.)
EOF
}

main "$@"
