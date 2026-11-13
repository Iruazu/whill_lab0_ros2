#!/usr/bin/env bash
# 屋外試験（#149 静止確認、#156 受け入れ、M8-4 の比較）の bag を mcap で記録する。
# 録画前に環境と入力トピックを確かめ、空や欠けた bag を現地で作らないようにする。
# 計画: docs/plans/2026-10-28-avoidance-baseline.md §4.2（静止確認）、§5（記録の仕様）。
# SPDX-License-Identifier: BSD-3-Clause
#
# 判断理由（feat/avoidance-baseline から main へ持ってくるときに変えた点）:
# - 保存先は docs/bags.md の規約 ~/bags/<run>/ に合わせ、1 回の試験を 1 つの <run> に
#   まとめる。check_object_placement.py の --bag-root と notes.csv をそのまま置ける。
# - CYCLONEDDS_URI は runbook と同じ configs/cyclonedds-runtime-min.xml に限る
#   （CLAUDE.md: 旧 runtime / bag-record XML との混在を避ける）。NIC は check_dds_nic.py。
# - 入力の確認は、型と QoS を明示して 1 通受け取る（check_failsafe_inputs.sh と同じ方法）。
#   型を省くと echo は発見の前に即座に失敗し、待たずに「届かない」と誤判定する。
# - drive の必須から /cmd_vel を外した。停止中は twist_mux が何も出さないので、
#   正常でも届かない。代わりに failsafe の入力（check_failsafe_inputs.sh）を確かめる。
# - main の m6r_bringup に avoidance 引数は無い（M8-4 #162/#163 で入る）。D435 は
#   realsense:=true で起動する。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAGS_ROOT="${WHILL_BAGS_ROOT:-${HOME}/bags}"
PROFILE=drive
DURATION=
RUN=
TAG=

usage() {
    cat <<'EOF'
使用法: scripts/record_avoidance_bag.sh [オプション] <タグ>
  --profile static|drive  static: センサだけ（#149 静止確認） / drive: 走行一式（既定、#156）
  --run NAME              1 回の試験をまとめるディレクトリ名（既定: <YYYY-MM-DD>-<profile>）
  --duration SEC          指定秒数で自動停止（省略時は Ctrl-C で停止）
  -h, --help              この説明を表示

保存先: ${WHILL_BAGS_ROOT:-~/bags}/<run>/<タグ>_<HHMMSS>/（mcap）。bag はリポジトリに置かない。
static: m6r_bringup_launch.py realsense:=true を起動しておく（D435 の深度・カラーが必須）。
drive : m6r_bringup + nav_launch を起動しておく。D435 は任意（無ければ警告して続ける）。
static の各 bag の後に、<run>/notes.csv に追記する行の雛形を表示する
（列は check_object_placement.py の bag,object,x_m,height_m）。
EOF
}
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }

need_value() { [[ $# -ge 2 && -n "$2" ]] || fail "$1 に値が要る"; }

while (($#)); do
    case "$1" in
        --profile|--run|--duration) need_value "$@" ;;
    esac
    case "$1" in
        --profile) PROFILE=${2:-}; shift 2 ;;
        --run) RUN=${2:-}; shift 2 ;;
        --duration) DURATION=${2:-}; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*) fail "不明なオプション: $1" ;;
        *) [[ -z "$TAG" ]] || fail 'タグは1つだけ指定する'; TAG=$1; shift ;;
    esac
done
[[ -n "$TAG" ]] || { usage >&2; exit 2; }
# notes.csv と check_object_placement.py の引数にそのまま使うので、空白やカンマを許さない。
[[ "$TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || fail 'タグは英数字と . _ - のみ'
case "$PROFILE" in static|drive) ;; *) fail 'profile は static か drive' ;; esac
[[ -z "$DURATION" || "$DURATION" =~ ^[1-9][0-9]*$ ]] || fail 'duration は正の整数秒'
RUN=${RUN:-$(date +%Y-%m-%d)-${PROFILE}}
[[ "$RUN" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || fail 'run は英数字と . _ - のみ'

# --- 録画前チェック ---------------------------------------------------------
# (1) RMW: FastDDS だと /velodyne_points が 1 Hz に詰まる（2026-06-24 実測、m6r_record_calib_bag.sh）。
[[ "${RMW_IMPLEMENTATION:-}" == rmw_cyclonedds_cpp ]] \
    || fail "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-未設定}。export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
# (2) DDS XML: 全端末で揃える。runbook と start_dispatch_demo.sh は runtime-min を使う。
XML="${CYCLONEDDS_URI:-}"
XML="${XML#file://}"
[[ -n "$XML" && -r "$XML" ]] || fail "CYCLONEDDS_URI が未設定か読めない: ${CYCLONEDDS_URI:-未設定}"
[[ "$(basename "$XML")" == cyclonedds-runtime-min.xml ]] \
    || fail "CYCLONEDDS_URI=${XML}。現用は configs/cyclonedds-runtime-min.xml（旧 XML と混在させない）"
# XML の NIC 名が実機の LiDAR の NIC と違っても、presence_required=false で黙って起動する。
NIC_MSG=$(python3 "${REPO_ROOT}/scripts/check_dds_nic.py" "$XML" 2>&1) \
    || fail "DDS の NIC が合わない: ${NIC_MSG}"
# (3) governor: 再起動で powersave に戻り、点群の取りこぼしが増える。
GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)
[[ "$GOV" == performance ]] || fail "CPU governor=${GOV}。sudo cpupower frequency-set -g performance"
# (4) ros2 と記録に要るプラグイン。
command -v ros2 >/dev/null || fail 'ros2 が PATH にない。setup.bash を source する'
ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1 \
    || fail 'rosbag2_storage_mcap が無い。sudo apt install ros-humble-rosbag2-storage-mcap'
# カラーは圧縮で記録する。プラグインが無いと /compressed が出ず、黙ってカラーが欠ける。
ros2 pkg prefix compressed_image_transport >/dev/null 2>&1 \
    || fail 'compressed_image_transport が無い。sudo apt install ros-humble-compressed-image-transport'

SENSOR_TOPICS=(
    /tf /tf_static /rosout
    # packets を残せば、どちらの校正表（VLP16db / Hi-Res）でも点群を作り直せる。
    /velodyne_packets /velodyne_points
    /imu/data_raw /imu/data_rep145
    /camera/camera/color/image_raw/compressed /camera/camera/color/camera_info
    /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info
)
DRIVE_TOPICS=(
    /whill/odom /odometry/filtered /pcl_pose /alignment_status
    /cmd_vel_nav /cmd_vel_teleop /cmd_vel_safety /cmd_vel /whill/controller/cmd_vel
    /scan /velodyne_points_no_ground
    /local_costmap/costmap /local_costmap/costmap_updates /local_costmap/published_footprint
    # global は全体（初回1枚）と差分更新だけ。全体を毎回は送られない。
    /global_costmap/costmap /global_costmap/costmap_updates
    /plan /dispatch/state
    /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
    # 以下は M8-4（#162/#163）で入るトピック。今の main には無いが、列挙しておけば
    # 出たときに記録される（無いトピックの指定は ros2 bag record の妨げにならない）。
    /failsafe/state /perception/lidar_obstacles /polygon_stop /polygon_slowdown
)
# 必須入力: トピック|型|QoS。QoS は配信側に合わせる。best_effort の購読は
# RELIABLE の配信にも合うが、/tf_static は transient_local でないと受け取れない。
REQUIRED=(
    '/velodyne_packets|velodyne_msgs/msg/VelodyneScan|--qos-profile sensor_data'
    '/imu/data_rep145|sensor_msgs/msg/Imu|--qos-profile sensor_data'
    '/tf_static|tf2_msgs/msg/TFMessage|--qos-reliability reliable --qos-durability transient_local'
)
CAMERA=(
    '/camera/camera/depth/image_rect_raw|sensor_msgs/msg/Image|--qos-profile sensor_data'
    '/camera/camera/color/image_raw/compressed|sensor_msgs/msg/CompressedImage|--qos-profile sensor_data'
)
TOPICS=("${SENSOR_TOPICS[@]}")
if [[ "$PROFILE" == drive ]]; then
    TOPICS+=("${DRIVE_TOPICS[@]}")
    REQUIRED+=('/odometry/filtered|nav_msgs/msg/Odometry|--qos-profile sensor_data')
fi

TIMEOUT_S=${RECORD_INPUT_TIMEOUT_S:-6}
probe() {  # probe <topic|type|qos>。届けば 0。
    local topic type qos
    IFS='|' read -r topic type qos <<<"$1"
    # shellcheck disable=SC2086
    timeout "$TIMEOUT_S" ros2 topic echo --once --no-arr $qos "$topic" "$type" >/dev/null 2>&1
}

echo "==> 入力トピックを確認（各最大${TIMEOUT_S}秒）"
for entry in "${REQUIRED[@]}"; do
    probe "$entry" || fail "${entry%%|*} を受信できない。bringup・QoS・DDS設定を確認する"
    printf '    %-45s ok\n' "${entry%%|*}"
done
for entry in "${CAMERA[@]}"; do
    if probe "$entry"; then
        printf '    %-45s ok\n' "${entry%%|*}"
    elif [[ "$PROFILE" == static ]]; then
        # 静止確認は深度の物の位置も測る（#146, #148）。カメラ無しでは目的を果たさない。
        fail "${entry%%|*} を受信できない。m6r_bringup_launch.py realsense:=true で起動する"
    else
        warn "${entry%%|*} が無い。カメラ無しで記録する（drive では任意）"
    fi
done
if [[ "$PROFILE" == drive ]]; then
    # nav_launch と localizer が動いているかを、failsafe と同じ型・QoS で確かめる。
    "${REPO_ROOT}/scripts/check_failsafe_inputs.sh" \
        || fail 'failsafe の入力が揃わない。nav_launch・localizer の起動と初期位置を確認する'
fi

# --- 記録 -------------------------------------------------------------------
RUN_DIR="${BAGS_ROOT}/${RUN}"
NAME="${TAG}_$(date +%H%M%S)"
OUT="${RUN_DIR}/${NAME}"
[[ ! -e "$OUT" ]] || fail "${OUT} が既にある。上書きしない"
mkdir -p "$RUN_DIR"
echo "==> 記録開始: ${OUT}（profile=${PROFILE}）"
# --include-hidden-topics: Nav2 のゴール状態（/navigate_to_pose/_action/*）は隠しトピック。
# 圧縮は掛けない: 画像は既に圧縮済みで、zstd は記録中の CPU を食う。
CMD=(ros2 bag record -s mcap --include-hidden-topics -o "$OUT" "${TOPICS[@]}")
if [[ -n "$DURATION" ]]; then
    # SIGINT で止めると rosbag2 がキャッシュを書き切ってから終了する。
    timeout -s INT "$DURATION" "${CMD[@]}" || true
else
    "${CMD[@]}" || true
fi

# --- 記録後の確認 -----------------------------------------------------------
echo '==> 記録内容'
INFO=$(ros2 bag info "$OUT" 2>&1) || fail "ros2 bag info が失敗した: ${OUT}"
printf '%s\n' "$INFO" | grep -E 'Duration|Bag size|Topic:' \
    | sed -E 's/\| Serialization Format: cdr//'
DUR=$(printf '%s\n' "$INFO" | sed -nE 's/^Duration:\s+([0-9.]+)s.*/\1/p' | head -1)
VEL=$(printf '%s\n' "$INFO" | grep -E 'Topic: /velodyne_points ' | sed -nE 's/.*Count:\s*([0-9]+).*/\1/p')
if [[ -n "$DUR" && -n "$VEL" ]]; then
    # 点群は 10 Hz。半分未満なら取りこぼしが多く、捨てて撮り直す（m6r_record_calib_bag.sh と同じ基準）。
    if awk -v a="$VEL" -v d="$DUR" 'BEGIN{exit !(a < d * 10 * 0.5)}'; then
        warn "/velodyne_points が ${VEL} 通（${DUR} 秒、期待 約$(awk -v d="$DUR" 'BEGIN{printf "%.0f", d*10}')）。撮り直す"
    fi
else
    warn "/velodyne_points の件数を読めない。ros2 bag info ${OUT} で確かめる"
fi

if [[ "$PROFILE" == static ]]; then
    NOTES="${RUN_DIR}/notes.csv"
    [[ -e "$NOTES" ]] || echo 'bag,object,x_m,height_m,y_m,half_width_m' > "$NOTES"
    echo
    echo "==> ${NOTES} に次の行を追記する（x_m はテープの値、height_m は物の高さ、空の基準は object=empty）:"
    echo "    ${NAME},<object>,<x_m>,<height_m>"
    echo "    解析: scripts/check_object_placement.py --bag-root ${RUN_DIR} --notes ${NOTES} --empty <空のbag> --out ${RUN_DIR}/check"
fi
