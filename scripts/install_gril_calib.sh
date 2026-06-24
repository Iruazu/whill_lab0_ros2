#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Install GRIL-Calib (Taeyoung96/GRIL-Calib humble branch) for the M5-R
# Issue #64 follow-up: recalibrating LiDAR↔IMU extrinsic to fix GLIM's
# persistent "IMU prediction is not good" warning.
#
# GRIL-Calib is a *calibration tool*, not a runtime dependency, so it
# lives in its own workspace (~/calib_ws) rather than under src/third_party/.
# That keeps the whill_lab0_ros2 build tree clean and avoids tying every
# `colcon build` to a tool we only use during one-off recalibration.
#
# Idempotent: re-running with the workspace already populated just rebuilds
# (incremental). Pass --force to wipe and re-clone.
#
# Usage:
#   ./scripts/install_gril_calib.sh         # clone + build (incremental)
#   ./scripts/install_gril_calib.sh --force # wipe ~/calib_ws and start fresh

set -euo pipefail

GRIL_REPO="https://github.com/Taeyoung96/GRIL-Calib.git"
GRIL_BRANCH="humble"
CALIB_WS="${HOME}/calib_ws"

FORCE=0
if [[ "${1:-}" == "--force" ]]; then
  FORCE=1
fi

# --- preflight ---------------------------------------------------------------
# We need ROS 2 humble for `rosdep`, `colcon`, ament_cmake. The script does
# not source it for the user — they must have done that already, so that
# any custom ROS_DOMAIN_ID etc. is preserved.
if ! command -v colcon >/dev/null 2>&1; then
  echo "ERROR: colcon not found. Source /opt/ros/humble/setup.bash first." >&2
  exit 1
fi
if ! command -v rosdep >/dev/null 2>&1; then
  echo "ERROR: rosdep not found. Install with: sudo apt install python3-rosdep" >&2
  exit 1
fi

# GRIL-Calib has a hard dependency on livox_ros_driver2 (Livox vendor's ROS 2
# driver; the LOAM-derived calibration core's CMakeLists references it
# unconditionally). The whill_lab0_ros2 workspace already vendors it under
# src/third_party/livox_ros_driver2 and installs the result. We rely on the
# whill workspace install/ being on AMENT_PREFIX_PATH so GRIL-Calib's
# `find_package(livox_ros_driver2)` resolves.
WHILL_INSTALL="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install"
if [[ ! -d "${WHILL_INSTALL}/livox_ros_driver2" ]]; then
  echo "ERROR: ${WHILL_INSTALL}/livox_ros_driver2 not found." >&2
  echo "       The GRIL-Calib build needs the in-repo livox_ros_driver2 install." >&2
  echo "       Run 'colcon build' in ~/whill_lab0_ros2 first." >&2
  exit 1
fi

# --- workspace setup ---------------------------------------------------------
if [[ "${FORCE}" -eq 1 ]]; then
  echo "==> --force: wiping ${CALIB_WS}"
  rm -rf "${CALIB_WS}"
fi

mkdir -p "${CALIB_WS}/src"

if [[ -d "${CALIB_WS}/src/GRIL-Calib" ]]; then
  echo "==> ${CALIB_WS}/src/GRIL-Calib already exists, fetching latest of ${GRIL_BRANCH}"
  (cd "${CALIB_WS}/src/GRIL-Calib" && git fetch origin "${GRIL_BRANCH}" && git checkout "${GRIL_BRANCH}" && git pull --ff-only)
else
  echo "==> cloning GRIL-Calib (${GRIL_BRANCH} branch)"
  git clone -b "${GRIL_BRANCH}" "${GRIL_REPO}" "${CALIB_WS}/src/GRIL-Calib"
fi

# --- build -------------------------------------------------------------------
# Source the whill workspace so livox_ros_driver2 is findable by CMake.
# `set +u` because ROS setup.bash references unset variables under set -u.
set +u
# shellcheck disable=SC1091
source "${WHILL_INSTALL}/setup.bash"
set -u

echo "==> rosdep install (system deps only; livox_ros_driver2 comes from sourced env)"
cd "${CALIB_WS}"
# rosdep will fail on livox_ros_driver2 (no rosdep key, it's vendored). Skip
# resolution for keys we already have available via the sourced workspace.
rosdep install --from-paths src --ignore-src -y \
  --skip-keys livox_ros_driver2 2>&1 | tail -5 || true

echo "==> colcon build --symlink-install"
# tee to a log file so build errors are visible AND captured. Without
# this `tail -10` would mask compiler errors from `set -euo pipefail`,
# leaving only the post-build "ERROR: gril_calib_exec not found" with
# no clue why.
BUILD_LOG="${CALIB_WS}/build.log"
if ! colcon build --symlink-install 2>&1 | tee "${BUILD_LOG}"; then
  echo "" >&2
  echo "ERROR: colcon build failed. Last 30 lines of ${BUILD_LOG}:" >&2
  tail -30 "${BUILD_LOG}" >&2
  exit 1
fi

# --- verify ------------------------------------------------------------------
if [[ -x "${CALIB_WS}/install/gril_calib/lib/gril_calib/gril_calib_exec" ]]; then
  echo ""
  echo "GRIL-Calib install OK at ${CALIB_WS}/install/gril_calib/"
  echo ""
  echo "Next steps (Issue #64):"
  echo "  1. Record a motion bag per docs/ja/m5r-imu-diagnostic.md §2"
  echo "     (figure-8 + accel/decel + 360° rotations, 3-5 min)"
  echo "  2. Patch ~/calib_ws/src/GRIL-Calib/config/velodyne32.yaml for WHILL/VLP-16"
  echo "     (see docs/ja/m5r-imu-diagnostic.md appendix A)"
  echo "  3. Run the calibration (manual procedure in appendix A until"
  echo "     scripts/m5r4_run_gril_calib.sh is added)"
  echo "  4. Transcribe the output T_lidar_imu into scripts/m5r3_run_glim.sh"
else
  echo "ERROR: build appears to have failed — no gril_calib_exec at expected path." >&2
  exit 1
fi
