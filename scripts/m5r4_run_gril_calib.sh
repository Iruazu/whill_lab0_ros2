#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
#
# Issue #64 M5-R4: run GRIL-Calib over a rosbag2 to estimate T_lidar_imu and
# diagnose the GLIM "IMU prediction is not good" warning.
#
# Automates docs/ja/m5r-imu-diagnostic.md appendix A:
#   1. Patch GRIL-Calib's velodyne32.yaml for WHILL/VLP-16 (topics, scan_line,
#      sensor height, initial T_IL guess from noetic).
#   2. Source whill_lab0_ros2 + calib_ws installs.
#   3. Launch gril_calib (rviz off) and play the bag.
#   4. Wait for GRIL-Calib to self-shutdown (it calls rclcpp::shutdown() when
#      data_sufficiency_assess() returns true) OR hit a watchdog timeout if the
#      bag lacks the motion excitation needed for sufficiency to converge.
#   5. Copy GRIL_Calib_result.txt into <out_dir> and render a copy-pasteable
#      `new_tli` block for scripts/m5r3_run_glim.sh.
#
# Usage:
#   scripts/m5r4_run_gril_calib.sh <bag_dir> [out_dir]
#
# Args:
#   bag_dir  rosbag2 directory containing /velodyne_points + /imu/data_rep145
#   out_dir  destination for result artifacts. Default: <bag_dir>/../gril-calib-out
#
# Watchdog:
#   GRIL-Calib needs strong angular motion on all axes to converge. Outdoor
#   loop bags rarely satisfy this. The script kills the calibration node when
#   the bag is fully played + GRIL_TIMEOUT_GRACE_SEC (default 60s) has elapsed,
#   preventing infinite hangs. Override with env GRIL_TIMEOUT_GRACE_SEC.
#
# Result interpretation:
#   - GRIL_Calib_result.txt with a 4x4 "Homogeneous Transformation Matrix from
#     LiDAR frame L to IMU frame I" => calibration converged. The new_tli.txt
#     companion file contains the GLIM-format quaternion ready to paste into
#     scripts/m5r3_run_glim.sh line ~275.
#   - run.log only, no result file => watchdog fired before sufficiency. Means
#     the bag did not excite enough motion (this is expected for outdoor loops;
#     a dedicated motion bag per diagnostic doc §2 is required).

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <bag_dir> [out_dir]" >&2
  exit 1
fi

BAG_DIR="$(realpath "$1")"
if [[ ! -d "${BAG_DIR}" ]]; then
  echo "ERROR: bag dir not found: ${BAG_DIR}" >&2
  exit 1
fi
if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
  echo "ERROR: not a rosbag2 (no metadata.yaml): ${BAG_DIR}" >&2
  exit 1
fi

OUT_DIR="${2:-${BAG_DIR}/../gril-calib-out}"
mkdir -p "${OUT_DIR}"
OUT_DIR="$(realpath "${OUT_DIR}")"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHILL_INSTALL="${REPO_ROOT}/install"
CALIB_WS="${HOME}/calib_ws"
GRIL_SRC="${CALIB_WS}/src/GRIL-Calib"
GRIL_RESULT_FILE="${GRIL_SRC}/result/GRIL_Calib_result.txt"
GRIL_TIMEOUT_GRACE_SEC="${GRIL_TIMEOUT_GRACE_SEC:-60}"

# --- preflight ---------------------------------------------------------------
for f in "${WHILL_INSTALL}/setup.bash" "${CALIB_WS}/install/setup.bash"; do
  if [[ ! -f "${f}" ]]; then
    echo "ERROR: missing setup.bash: ${f}" >&2
    echo "       Build the corresponding workspace first." >&2
    exit 1
  fi
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not on PATH. Source /opt/ros/humble/setup.bash first." >&2
  exit 1
fi

# --- inspect bag -------------------------------------------------------------
echo "==> inspecting ${BAG_DIR}"
BAG_INFO="$(ros2 bag info "${BAG_DIR}" 2>&1 || true)"
echo "${BAG_INFO}" | grep -E "Duration|Messages|Topic information" -A0 || true

# Extract bag duration in seconds for watchdog. `ros2 bag info` prints
# "Duration: 239.446s"; fall back to metadata.yaml nanoseconds if parsing fails.
BAG_DURATION_SEC="$(echo "${BAG_INFO}" | sed -nE 's/^Duration:\s+([0-9.]+)s.*/\1/p' | head -1)"
if [[ -z "${BAG_DURATION_SEC}" ]]; then
  # fallback: parse metadata.yaml
  NS="$(grep -oP 'nanoseconds:\s*\K[0-9]+' "${BAG_DIR}/metadata.yaml" | head -1)"
  if [[ -n "${NS}" ]]; then
    BAG_DURATION_SEC="$(awk -v ns="${NS}" 'BEGIN{printf "%.1f", ns/1e9}')"
  else
    BAG_DURATION_SEC=300
    echo "WARN: could not parse bag duration, defaulting watchdog to ${BAG_DURATION_SEC}s" >&2
  fi
fi

# Required topics
for t in "/velodyne_points" "/imu/data_rep145"; do
  if ! echo "${BAG_INFO}" | grep -q "${t}"; then
    echo "ERROR: bag does not contain ${t}" >&2
    exit 1
  fi
done

WATCHDOG_SEC="$(awk -v d="${BAG_DURATION_SEC}" -v g="${GRIL_TIMEOUT_GRACE_SEC}" 'BEGIN{printf "%d", d+g}')"
echo "==> bag duration ${BAG_DURATION_SEC}s, watchdog ${WATCHDOG_SEC}s"

# --- patch config ------------------------------------------------------------
# Don't mutate the upstream config; write a per-run copy into out_dir.
CONFIG_SRC="${GRIL_SRC}/config/velodyne32.yaml"
CONFIG_PATCHED="${OUT_DIR}/velodyne16_whill.yaml"

if [[ ! -f "${CONFIG_SRC}" ]]; then
  echo "ERROR: upstream config not found: ${CONFIG_SRC}" >&2
  exit 1
fi

# Patches (rationale tracked in docs/ja/m5r-imu-diagnostic.md appendix A):
#   imu_topic      "/imu/data" -> "/imu/data_rep145"  (PR #56 REP-145 topic)
#   scan_line      32 -> 16                          (VLP-16, not VLP-32)
#   imu_sensor_height 0.73 -> 0.47                   (PR #61 measured base_link->imu_link.z)
#   trans_IL_x/y/z 0.0 -> noetic initial guess       (faster convergence; this
#                                                     is what we're refining)
# noetic-derived extrinsic from docs/ja/m3-extrinsics-from-noetic.md:
#   extrinsic_T = [0.104136, 0.411548, 0.323704]  (LiDAR origin in IMU frame)
# GRIL-Calib's trans_IL is "from IMU frame I to LiDAR frame L" so we pass
# extrinsic_T directly (it already is the translation IMU->LiDAR per the
# noetic FAST-LIO convention).
sed \
  -e 's|imu_topic:  "/imu/data"|imu_topic:  "/imu/data_rep145"|' \
  -e 's|scan_line: 32|scan_line: 16|' \
  -e 's|imu_sensor_height : 0.73|imu_sensor_height : 0.47|' \
  -e 's|trans_IL_x : 0.0|trans_IL_x : 0.104136|' \
  -e 's|trans_IL_y : 0.0|trans_IL_y : 0.411548|' \
  -e 's|trans_IL_z : 0.0|trans_IL_z : 0.323704|' \
  "${CONFIG_SRC}" > "${CONFIG_PATCHED}"

# Verify patches landed (sed silently no-ops on mismatch)
for expected in '"/imu/data_rep145"' 'scan_line: 16' '0.47' '0.104136'; do
  if ! grep -qF "${expected}" "${CONFIG_PATCHED}"; then
    echo "ERROR: config patch did not apply (missing: ${expected}). Upstream config drift?" >&2
    diff -u "${CONFIG_SRC}" "${CONFIG_PATCHED}" >&2 || true
    exit 1
  fi
done
echo "==> patched config: ${CONFIG_PATCHED}"

# --- environment -------------------------------------------------------------
# Source both workspaces. set +u because ROS setup scripts touch unset vars.
set +u
# shellcheck disable=SC1091
source "${WHILL_INSTALL}/setup.bash"
# shellcheck disable=SC1091
source "${CALIB_WS}/install/setup.bash"
set -u

# Clear stale result file so we can tell whether *this* run produced one.
rm -f "${GRIL_RESULT_FILE}"
mkdir -p "${GRIL_SRC}/result"

# --- launch ------------------------------------------------------------------
RUN_LOG="${OUT_DIR}/run.log"
LAUNCH_LOG="${OUT_DIR}/launch.log"
BAG_LOG="${OUT_DIR}/bag-play.log"

echo "==> launching gril_calib (rviz off, use_sim_time=true), log: ${LAUNCH_LOG}"
# use_sim_time:=true is REQUIRED because the bag is played with --clock and
# its timestamps are earlier than wall clock. Without sim_time, gril_calib's
# node clock advances by wall time while incoming IMU messages carry bag time
# from the past — the reference-time delta becomes negative and gril_calib
# aborts with `cannot store a negative time point in rclcpp::Time` (2026-07-08).
ros2 launch gril_calib mapping_velodyne.launch.py \
  config_path:="${CONFIG_PATCHED}" \
  rviz:=false \
  use_sim_time:=true \
  > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

# Trap so a Ctrl-C kills the background processes too.
cleanup() {
  set +e
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null
    sleep 2
    kill -KILL "${LAUNCH_PID}" 2>/dev/null
  fi
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

# Give the node time to subscribe before bag plays start streaming.
sleep 3

echo "==> playing bag: ${BAG_DIR}"
# --clock publishes /clock so gril_calib (with use_sim_time:=true) sees
# bag time, matching the timestamps on incoming /velodyne_points and
# /imu/data_rep145. See use_sim_time note above launch invocation.
ros2 bag play "${BAG_DIR}" --clock --rate 1.0 > "${BAG_LOG}" 2>&1 &
BAG_PID=$!

# --- wait with watchdog ------------------------------------------------------
# GRIL-Calib exits when sufficiency is reached. If the bag finishes and the
# node is still running, sufficiency was not reached — kill after grace period.
echo "==> waiting (watchdog ${WATCHDOG_SEC}s)"
elapsed=0
while [[ ${elapsed} -lt ${WATCHDOG_SEC} ]]; do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "==> gril_calib exited (self-shutdown after ${elapsed}s)"
    break
  fi
  sleep 5
  elapsed=$((elapsed + 5))
done

if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "==> watchdog (${WATCHDOG_SEC}s) reached; gril_calib still running. Sending SIGINT."
  kill -INT "${LAUNCH_PID}" 2>/dev/null || true
  sleep 5
  kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
fi

# Wait for bag player to drain (if still alive). Don't fail on its exit code.
wait "${BAG_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true
trap - EXIT INT TERM

# --- collect results ---------------------------------------------------------
cp "${LAUNCH_LOG}" "${RUN_LOG}" 2>/dev/null || true

# GRIL-Calib's laserMapping.cpp opens the result file at node init (line ~1013)
# and only writes into it when fileout_calib_result() runs after sufficiency
# is reached. So an empty file means "node started but never converged" — we
# need a content check, not just file existence. Look for the result header
# the program emits in fileout_calib_result().
if [[ -s "${GRIL_RESULT_FILE}" ]] && grep -q "Homogeneous Transformation Matrix" "${GRIL_RESULT_FILE}"; then
  cp "${GRIL_RESULT_FILE}" "${OUT_DIR}/GRIL_Calib_result.txt"
  echo ""
  echo "=========================================="
  echo "GRIL-Calib converged. Result:"
  echo "=========================================="
  cat "${OUT_DIR}/GRIL_Calib_result.txt"
  echo "=========================================="

  # Convert the 4x4 T_IL (LiDAR -> IMU) to GLIM's new_tli format:
  # GLIM expects T_lidar_imu = (T_IL)^{-1}, i.e. IMU -> LiDAR, as
  # [tx, ty, tz, qx, qy, qz, qw].
  python3 - "${OUT_DIR}/GRIL_Calib_result.txt" "${OUT_DIR}/new_tli.txt" <<'PY' || echo "WARN: python conversion skipped (scipy/numpy missing?)"
import sys, re
import numpy as np
from scipy.spatial.transform import Rotation

src = sys.argv[1]
dst = sys.argv[2]
text = open(src).read()

# Grab the 4x4 matrix block after the "LiDAR frmae L to IMU frame I:" header.
m = re.search(
    r"Homogeneous Transformation Matrix from LiDAR.*?to IMU.*?:\s*\n((?:\s*-?\d.*\n){4})",
    text,
)
if not m:
    print("WARN: could not parse 4x4 matrix from result file", file=sys.stderr)
    sys.exit(0)

T_IL = np.array([
    [float(v) for v in line.split()]
    for line in m.group(1).strip().splitlines()
])
assert T_IL.shape == (4, 4), T_IL.shape

T_LI = np.linalg.inv(T_IL)
t = T_LI[:3, 3]
q = Rotation.from_matrix(T_LI[:3, :3]).as_quat()  # [qx, qy, qz, qw]

with open(dst, "w") as f:
    f.write("# Paste these 7 values into scripts/m5r3_run_glim.sh new_tli (line ~275).\n")
    f.write("# Order: tx, ty, tz, qx, qy, qz, qw (GLIM T_lidar_imu = IMU -> LiDAR).\n")
    for v in [*t, *q]:
        f.write(f"      {v:.6f},\n")

print(f"Wrote {dst}")
PY

  if [[ -f "${OUT_DIR}/new_tli.txt" ]]; then
    echo ""
    echo "GLIM new_tli (paste into scripts/m5r3_run_glim.sh):"
    cat "${OUT_DIR}/new_tli.txt"
  fi
  STATUS="converged"
else
  # Clean up the empty stub file so out_dir doesn't ship a misleading
  # zero-byte GRIL_Calib_result.txt.
  rm -f "${OUT_DIR}/GRIL_Calib_result.txt"
  echo ""
  echo "==> Calibration did NOT converge (no T_IL written)."
  echo "    GRIL-Calib did not reach data sufficiency before watchdog fired."
  echo "    This is the expected outcome for outdoor loop bags lacking the"
  echo "    motion excitation required by data_sufficiency_assess()."
  echo "    See docs/ja/m5r-imu-diagnostic.md §2 for the motion bag protocol."
  STATUS="insufficient_motion"
fi

# --- summary -----------------------------------------------------------------
cat > "${OUT_DIR}/SUMMARY.md" <<EOF
# GRIL-Calib run summary

- Bag: \`${BAG_DIR}\`
- Bag duration: ${BAG_DURATION_SEC}s
- Watchdog: ${WATCHDOG_SEC}s
- Status: **${STATUS}**
- Patched config: \`$(basename "${CONFIG_PATCHED}")\`
- Launch log: \`$(basename "${LAUNCH_LOG}")\`
- Bag-play log: \`$(basename "${BAG_LOG}")\`

EOF
if [[ "${STATUS}" == "converged" ]]; then
  cat >> "${OUT_DIR}/SUMMARY.md" <<EOF
- Result file: \`GRIL_Calib_result.txt\`
- GLIM-format extrinsic: \`new_tli.txt\`
EOF
else
  cat >> "${OUT_DIR}/SUMMARY.md" <<EOF
- Result file: not produced (insufficient motion excitation)
- Required next step: record a motion bag per
  \`docs/ja/m5r-imu-diagnostic.md\` §2 (figure-8 + accel/decel + in-place
  rotation, indoor flat floor, 3-5 min)
EOF
fi

echo ""
echo "==> output dir: ${OUT_DIR}"
echo "==> status: ${STATUS}"
