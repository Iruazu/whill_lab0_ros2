#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
#
# M6-R / Issue #64 helper: record an indoor motion bag for GRIL-Calib.
#
# Wraps `ros2 bag record` with the preflight checks that this project's
# CLAUDE.md and docs/ja/m5r-rmw-cyclonedds.md require every time hardware
# recording is done, so a hurried session cannot forget to set them:
#
#   1. RMW = rmw_cyclonedds_cpp     (fastdds throttles /velodyne_points to 1 Hz)
#   2. CPU governor = performance    (reset to powersave on every reboot)
#   3. bringup running                (odom_bringup_launch.py must publish
#                                      /velodyne_points and /imu/data_rep145)
#
# The recorded topics match the GRIL-Calib runner's required set:
#   /velodyne_points  /imu/data_rep145  /tf_static
#
# Companion doc: docs/ja/m6r-indoor-calib-bag-protocol.md.
#   §1 preflight (this script enforces)
#   §3 motion pattern (operator responsibility)
# Next stage: scripts/m5r4_run_gril_calib.sh (this script prints the command).
#
# Usage:
#   scripts/m6r_record_calib_bag.sh <run-dir>
#
#     <run-dir>   The per-run directory under docs/m5r-bench-data/. The bag
#                 is written to <run-dir>/bag/ (rosbag2 default layout).
#                 If the directory already contains a bag/ subdir, aborts
#                 unless --force is passed.
#
# The script does NOT start the bringup for you: launching whill_localization
# needs a separate terminal for its logs. It DOES check that the two required
# topics are already publishing before it opens the recorder — recording
# without bringup would produce an empty bag, and the operator only notices
# after §3 of the motion protocol is done.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat >&2 <<EOF
Usage: ${0##*/} <run-dir> [--force]

  <run-dir>  Per-run directory under docs/m5r-bench-data/ (e.g.
             docs/m5r-bench-data/2026-07-08-indoor-calib). Bag is written
             to <run-dir>/bag/.
  --force    Overwrite existing <run-dir>/bag/ contents.
  -h, --help Show this message.

Before running:
  Terminal A must have \`ros2 launch whill_localization odom_bringup_launch.py\`
  running so that /velodyne_points (10 Hz) and /imu/data_rep145 (100 Hz) are
  live. This script checks that they are before opening the recorder.

Motion protocol during recording: docs/ja/m6r-indoor-calib-bag-protocol.md §3.
EOF
  exit 2
}

case "${1:-}" in
  -h|--help) usage ;;
esac
if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
fi

RUN_DIR="$1"
FORCE=0
if [[ $# -eq 2 ]]; then
  if [[ "$2" != "--force" ]]; then
    usage
  fi
  FORCE=1
fi

mkdir -p "${RUN_DIR}"
RUN_DIR="$(cd "${RUN_DIR}" && pwd)"
BAG_DIR="${RUN_DIR}/bag"

# --- preflight ---------------------------------------------------------------

# (1) RMW: must be CycloneDDS. FastDDS causes /velodyne_points 1 Hz stalls
# (proven 2026-06-24, see docs/ja/m5r-rmw-cyclonedds.md).
if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]]; then
  echo "ERROR: RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}" >&2
  echo "       Must be rmw_cyclonedds_cpp. Fix in ~/.bashrc:" >&2
  echo "         export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >&2
  echo "       Then re-source ~/.bashrc in this shell." >&2
  exit 1
fi

# (2) CPU governor: performance. Resets to powersave on reboot.
GOVERNOR="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
if [[ "${GOVERNOR}" != "performance" ]]; then
  echo "ERROR: CPU governor is '${GOVERNOR}', must be 'performance'." >&2
  echo "       Fix:  sudo cpupower frequency-set -g performance" >&2
  exit 1
fi

# (3) ros2 on PATH.
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not on PATH. Source /opt/ros/humble/setup.bash first." >&2
  exit 1
fi

# (4) out-dir state.
if [[ -d "${BAG_DIR}" ]]; then
  if [[ "${FORCE}" -eq 0 ]]; then
    echo "ERROR: ${BAG_DIR} already exists." >&2
    echo "       Re-run with --force to overwrite, or pick a different run-dir." >&2
    exit 1
  fi
  echo "==> --force: removing existing ${BAG_DIR}"
  rm -rf "${BAG_DIR}"
fi

# (5) bringup topics live. If either is missing, the operator would
# discover the empty bag only after the full 5-minute motion protocol.
# ros2 topic list is cheap; use it for a 5s snapshot.
echo "==> checking bringup topics (5s window)"
TOPICS_SEEN="$(timeout 5 ros2 topic list 2>/dev/null || true)"
for t in "/velodyne_points" "/imu/data_rep145"; do
  if ! echo "${TOPICS_SEEN}" | grep -qE "^${t}\$"; then
    echo "ERROR: topic ${t} not visible on the graph." >&2
    echo "       Is \`ros2 launch whill_localization odom_bringup_launch.py\` running in Terminal A?" >&2
    echo "       (Also confirm the Velodyne unicast UDP is up and the RT IMU USB is plugged.)" >&2
    exit 1
  fi
done
echo "    /velodyne_points   ok"
echo "    /imu/data_rep145   ok"

# --- record ------------------------------------------------------------------

echo ""
echo "==> starting recorder → ${BAG_DIR}"
echo "    topics:  /velodyne_points  /imu/data_rep145  /tf_static"
echo "    Follow docs/ja/m6r-indoor-calib-bag-protocol.md §3 for motion."
echo "    Press Ctrl+C in this terminal to stop."
echo ""

# --storage sqlite3 (default) — GLIM/GRIL-Calib do not read mcap yet.
# No compression: zstd is not supported by GLIM (see repo convention).
ros2 bag record \
  --storage sqlite3 \
  --output "${BAG_DIR}" \
  /velodyne_points /imu/data_rep145 /tf_static

# --- post-record health check ------------------------------------------------

echo ""
echo "==> recording finished. running ros2 bag info…"
INFO="$(ros2 bag info "${BAG_DIR}" 2>&1 || true)"
echo "${INFO}"

DUR="$(echo "${INFO}" | sed -nE 's/^Duration:\s+([0-9.]+)s.*/\1/p' | head -1)"
VEL="$(echo "${INFO}" | grep -E '/velodyne_points' | sed -nE 's/.*Count:\s*([0-9]+).*/\1/p')"
IMU="$(echo "${INFO}" | grep -E '/imu/data_rep145' | sed -nE 's/.*Count:\s*([0-9]+).*/\1/p')"

echo ""
echo "==> health check"
if [[ -n "${DUR}" && -n "${VEL}" && -n "${IMU}" ]]; then
  # Expected rates from bringup: velodyne 10 Hz, imu 100 Hz.
  EXP_VEL="$(awk -v d="${DUR}" 'BEGIN{printf "%.0f", d*10}')"
  EXP_IMU="$(awk -v d="${DUR}" 'BEGIN{printf "%.0f", d*100}')"
  echo "    duration:         ${DUR} s"
  echo "    /velodyne_points: ${VEL} / expected ~${EXP_VEL}  ($(awk -v a="${VEL}" -v e="${EXP_VEL}" 'BEGIN{printf "%.1f", 100*a/e}')%)"
  echo "    /imu/data_rep145: ${IMU} / expected ~${EXP_IMU}  ($(awk -v a="${IMU}" -v e="${EXP_IMU}" 'BEGIN{printf "%.1f", 100*a/e}')%)"

  # Warn if less than 50% expected (repo convention: discard and re-record).
  VEL_LOW="$(awk -v a="${VEL}" -v e="${EXP_VEL}" 'BEGIN{print (a < e*0.5) ? "1" : "0"}')"
  IMU_LOW="$(awk -v a="${IMU}" -v e="${EXP_IMU}" 'BEGIN{print (a < e*0.5) ? "1" : "0"}')"
  if [[ "${VEL_LOW}" == "1" || "${IMU_LOW}" == "1" ]]; then
    echo ""
    echo "WARNING: message counts below 50% of expected. Repo convention" >&2
    echo "         (CLAUDE.md) is to discard the bag and re-record." >&2
  fi
else
  echo "    could not parse ros2 bag info output. Inspect manually:" >&2
  echo "         ros2 bag info ${BAG_DIR}" >&2
fi

# --- next-step hint ----------------------------------------------------------

echo ""
echo "==> next steps"
echo "  1. Verify the motion protocol was fully covered (docs/ja/m6r-indoor-calib-bag-protocol.md §3)."
echo "  2. Run GRIL-Calib on this bag:"
echo "       ./scripts/m5r4_run_gril_calib.sh ${BAG_DIR}"
echo "  3. Interpret the result (see indoor-calib protocol §6, §7)."
