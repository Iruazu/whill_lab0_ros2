#!/usr/bin/env bash
# 古典的回避の比較基準（docs/ja/plans/2026-10-28-avoidance-baseline.md）の走行・静止データを
# mcap で記録する。録画前に環境と入力トピックを確かめ、空や欠けたbagを現地で防ぐ。
# SPDX-License-Identifier: BSD-3-Clause
set -eo pipefail

PROFILE=drive
DURATION=
OUT_ROOT="${HOME}/bags"
TAG=

usage() {
    cat <<'EOF'
使用法: scripts/record_avoidance_bag.sh [オプション] <タグ>
  --profile static|drive  static: センサだけ（静止確認用） / drive: 走行一式（既定）
  --duration SEC          指定秒数で自動停止（省略時は Ctrl-C で停止）
  --out DIR               保存先の親ディレクトリ（既定: ~/bags）
  -h, --help              この説明を表示

保存先: <DIR>/<YYYYMMDD>/<タグ>_<HHMMSS>/（mcap）。bagはリポジトリに置かない。
static は m6r_bringup（realsense:=true か avoidance:=true）だけで足りる。
drive は m6r_bringup avoidance:=true と nav_launch avoidance:=true を起動しておく。
EOF
}
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

while (($#)); do
    case "$1" in
        --profile) PROFILE=${2:-}; shift 2 ;;
        --duration) DURATION=${2:-}; shift 2 ;;
        --out) OUT_ROOT=${2:-}; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*) fail "不明なオプション: $1" ;;
        *) [[ -z "$TAG" ]] || fail 'タグは1つだけ指定する'; TAG=$1; shift ;;
    esac
done
[[ -n "$TAG" ]] || { usage; exit 2; }
[[ "$TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || fail 'タグは英数字と . _ - のみ'
case "$PROFILE" in static|drive) ;; *) fail 'profile は static か drive' ;; esac
[[ -z "$DURATION" || "$DURATION" =~ ^[0-9]+$ ]] || fail 'duration は整数秒'

# --- 録画前チェック ---------------------------------------------------------
# RMW と DDS XML は全端末で揃える（CLAUDE.md）。食い違うと無言で届かない。
[[ "${RMW_IMPLEMENTATION:-}" == rmw_cyclonedds_cpp ]] \
    || fail "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-未設定}。rmw_cyclonedds_cpp にする"
[[ -n "${CYCLONEDDS_URI:-}" && -r "${CYCLONEDDS_URI#file://}" ]] \
    || fail "CYCLONEDDS_URI が未設定か読めない: ${CYCLONEDDS_URI:-未設定}"
command -v ros2 >/dev/null || fail 'ros2 が PATH にない。setup.bash を source する'
# 再起動で powersave に戻り、点群の取りこぼしが増える（m6r_record_calib_bag.sh と同じ理由）。
GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)
[[ "$GOV" == performance ]] || fail "CPU governor=${GOV}。sudo cpupower frequency-set -g performance"
# カラーは圧縮で記録する。プラグインが無いと /compressed が出ず、黙ってカラーが欠ける。
ros2 pkg prefix compressed_image_transport >/dev/null 2>&1 \
    || fail 'compressed_image_transport が無い。sudo apt install ros-humble-compressed-image-transport'

SENSOR_TOPICS=(
    /tf /tf_static /rosout
    # packets を残せば、どちらの校正（VLP16db / Hi-Res）でも点群を作り直せる。
    /velodyne_packets /velodyne_points
    /imu/data_raw /imu/data_rep145
    /camera/camera/color/image_raw/compressed /camera/camera/color/camera_info
    /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info
)
DRIVE_TOPICS=(
    /whill/odom /odometry/filtered /pcl_pose /alignment_status
    /cmd_vel_nav /cmd_vel_teleop /cmd_vel_safety /cmd_vel /cmd_vel_smoothed
    /whill/controller/cmd_vel /failsafe/state
    /scan /velodyne_points_no_ground /perception/lidar_obstacles
    /local_costmap/costmap /local_costmap/costmap_updates /local_costmap/published_footprint
    # global は全体（初回1枚、約42MB）と差分更新だけ。全体を毎回は送られない。
    /global_costmap/costmap /global_costmap/costmap_updates
    /plan /dispatch/state
    /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
    /polygon_stop /polygon_slowdown
)
REQUIRED=(/velodyne_packets /tf_static /camera/camera/depth/image_rect_raw
          /camera/camera/color/image_raw/compressed)
TOPICS=("${SENSOR_TOPICS[@]}")
if [[ "$PROFILE" == drive ]]; then
    TOPICS+=("${DRIVE_TOPICS[@]}")
    REQUIRED+=(/odometry/filtered /pcl_pose /failsafe/state /cmd_vel)
fi

# BEST_EFFORT で1通受け取れるかで確かめる。ros2 topic hz / list だけでは
# QoS の食い違い（RELIABLE と BEST_EFFORT）を見逃す（CLAUDE.md の前例）。
echo '==> 入力トピックを確認（各最大6秒）'
for t in "${REQUIRED[@]}"; do
    if timeout 6 ros2 topic echo --once --qos-reliability best_effort --no-arr "$t" >/dev/null 2>&1 \
        || timeout 6 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable --no-arr "$t" >/dev/null 2>&1; then
        printf '    %-45s ok\n' "$t"
    else
        fail "$t を受信できない。bringup・realsense・QoS・DDS設定を確認する"
    fi
done

# --- 記録 -------------------------------------------------------------------
OUT="${OUT_ROOT}/$(date +%Y%m%d)/${TAG}_$(date +%H%M%S)"
mkdir -p "$(dirname "$OUT")"
echo "==> 記録開始: ${OUT}（profile=${PROFILE}）"
# --include-hidden-topics: Nav2 のゴール状態（/navigate_to_pose/_action/*）は隠しトピック。
# 圧縮は掛けない: 画像は既に圧縮済みで、zstd は記録中のCPUを食う。
CMD=(ros2 bag record -s mcap --include-hidden-topics -o "$OUT" "${TOPICS[@]}")
if [[ -n "$DURATION" ]]; then
    # SIGINT で止めると rosbag2 がキャッシュを書き切ってから終了する。
    timeout -s INT "$DURATION" "${CMD[@]}" || true
else
    "${CMD[@]}" || true
fi

echo '==> 記録内容'
ros2 bag info "$OUT" | grep -E 'Duration|Bag size|Topic:' \
    | sed -E 's/\| Serialization Format: cdr//'
