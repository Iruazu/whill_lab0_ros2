#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# #152: check velodyne_reproject_bag.py against the driver itself.
#
# Needs a bag that holds both /velodyne_packets and /velodyne_points (recorded
# with VLP16db.yaml), e.g. ~/bags/sensor_check_20261028/chk_empty_0m_160726.
#  1. re-project its /velodyne_points to Hi-Res with velodyne_reproject_bag.py
#  2. replay its /velodyne_packets into velodyne_transform_node with
#     VLP16_hires_db.yaml and record the output (the reference)
#  3. compare both point by point (velodyne_compare_clouds.py)
# The map bags have no packets, so this is the only way to test the analytic
# inverse against the driver code instead of against the same formula.
#
# The input bag is only read. The ROS graph is localhost-only on its own domain
# so the replay cannot reach a running robot or another session.
# Usage: velodyne_reproject_verify.sh <bag-with-packets> <work-dir>
set -eo pipefail
BAG=$(readlink -f "$1"); WORK=$2
[[ -n "$BAG" && -n "$WORK" ]] || { echo "usage: $0 <bag> <work-dir>" >&2; exit 2; }
# Re-runnable: each run gets a fresh work dir instead of overwriting a previous result.
[[ ! -e "$WORK" ]] || { echo "$WORK exists; pick a new work dir" >&2; exit 2; }
mkdir -p "$WORK"; WORK=$(readlink -f "$WORK")
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${WHILL_UPSTREAM_INSTALL:-$HOME/whill_lab0_ros2/install}/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-152} ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
PKG=$(ros2 pkg prefix velodyne_pointcloud)/share/velodyne_pointcloud
SRC_CAL=$PKG/params/VLP16db.yaml
DST_CAL=$PKG/params/VLP16_hires_db.yaml

python3 "$SCRIPT_DIR/velodyne_reproject_bag.py" "$BAG" "$WORK/reprojected" \
  --src-calib "$SRC_CAL" --dst-calib "$DST_CAL" --include /velodyne_points \
  2>"$WORK/reproject.log"
tail -1 "$WORK/reproject.log"

# Same node parameters as the runtime (upstream VLP16 launch), only the
# calibration differs, so min/max range and organize_cloud match the recording.
# The binary is started directly (not via `ros2 run`): SIGINT to the ros2 run
# wrapper did not reach the node, so the script hung at `wait`.
"$(ros2 pkg prefix velodyne_pointcloud)/lib/velodyne_pointcloud/velodyne_transform_node" --ros-args \
  --params-file "$PKG/config/VLP16-velodyne_transform_node-params.yaml" \
  -p calibration:="$DST_CAL" >"$WORK/transform.log" 2>&1 &
TF_PID=$!
ros2 bag record -o "$WORK/reference" /velodyne_points >"$WORK/record.log" 2>&1 &
REC_PID=$!
trap 'kill -INT $REC_PID $TF_PID 2>/dev/null || true' EXIT
# Discovery must finish before playback, otherwise the first clouds are lost
# (they are then simply not compared, but fewer clouds weaken the check).
sleep 5
# Half rate: the recorder must not drop clouds under load.
ros2 bag play "$BAG" --topics /velodyne_packets --rate 0.5 </dev/null >"$WORK/play.log" 2>&1
sleep 3
kill -INT $REC_PID; wait $REC_PID || true
kill -INT $TF_PID; wait $TF_PID || true
trap - EXIT

python3 "$SCRIPT_DIR/velodyne_compare_clouds.py" "$WORK/reference" "$WORK/reprojected" \
  | tee "$WORK/compare.txt"
# Negative control: the uncorrected recording must FAIL against the same
# reference, otherwise the comparison itself cannot tell the calibrations apart.
echo "--- control: recorded (VLP16db) vs reference (expected FAIL)"
if python3 "$SCRIPT_DIR/velodyne_compare_clouds.py" "$WORK/reference" "$BAG" \
    | tee "$WORK/control.txt"; then
  echo "control unexpectedly passed; the check is not discriminating" >&2; exit 1
fi
