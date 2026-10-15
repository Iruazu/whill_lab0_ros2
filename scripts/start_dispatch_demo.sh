#!/usr/bin/env bash
# 実機デモの環境を揃え、所有するlaunchだけをまとめて起動・終了する。
# SPDX-License-Identifier: BSD-3-Clause
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SITE=campus
SPEED=normal
DOMAIN_ID=0
WITH_RVIZ=true
DRY_RUN=false
PREFLIGHT_ONLY=false
PIDS=()
NAMES=()
LOG_DIR=

usage() {
    cat <<'EOF'
使用法: scripts/start_dispatch_demo.sh [オプション]
  --site NAME       地図site（既定: campus）
  --speed PRESET    slow / normal / fast（既定: normal）
  --domain ID       ROS_DOMAIN_ID（既定: 0）
  --no-rviz         RVizを起動しない
  --dry-run         環境・コマンドを表示するだけ
  --preflight       同じ環境で既存preflightのみを実行する
  -h, --help        この説明を表示

既定で実機bringup・Nav2・TLS配車Web・RVizを起動する。
初期位置をRVizで設定し、別ターミナルで --preflight を実行する。
走行goalは自動投入しない。Ctrl-Cでこのスクリプトのプロセスを終了する。
AP・CPU・lo multicast・証明書の準備はrunbookを参照。
EOF
}
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
parse_args() {
    while (($#)); do
        case "$1" in
            --site|--speed|--domain)
                (($# >= 2)) || fail "$1 に値が必要です"
                case "$1" in
                    --site) SITE=$2 ;;
                    --speed) SPEED=$2 ;;
                    --domain) DOMAIN_ID=$2 ;;
                esac
                shift 2 ;;
            --no-rviz) WITH_RVIZ=false; shift ;;
            --dry-run) DRY_RUN=true; shift ;;
            --preflight) PREFLIGHT_ONLY=true; shift ;;
            -h|--help) usage; exit 0 ;;
            *) fail "不明なオプション: $1" ;;
        esac
    done
    [[ "$SITE" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || fail 'site名が不正です'
    case "$SPEED" in slow|normal|fast) ;; *) fail 'speedはslow / normal / fastです' ;; esac
    [[ "$DOMAIN_ID" =~ ^[0-9]{1,3}$ ]] || fail 'domainは0〜232の整数です'
    DOMAIN_ID=$((10#$DOMAIN_ID))
    ((DOMAIN_ID <= 232)) || fail 'domainは0〜232の整数です'
}
load_environment() {
    [[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humbleが見つかりません'
    [[ -r "$REPO_ROOT/install/setup.bash" ]] || fail '先に車載スタックをビルドしてください'
    # ROSのsetupは未定義変数を参照するためnounsetは使わない。
    source /opt/ros/humble/setup.bash
    source "$REPO_ROOT/install/setup.bash"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID="$DOMAIN_ID"
    export CYCLONEDDS_URI="file://${REPO_ROOT}/configs/cyclonedds-runtime-min.xml"
    [[ -r "${CYCLONEDDS_URI#file://}" ]] || fail "DDS設定がありません: ${CYCLONEDDS_URI#file://}"
}
start_process() {
    local name=$1
    shift
    # launchの子ノードも同じgroupへ入れ、親だけ残る・子だけ残る状態を避ける。
    setsid "$@" >"$LOG_DIR/$name.log" 2>&1 9>&- &
    PIDS+=("$!")
    NAMES+=("$name")
    printf '起動: %s（log: %s/%s.log）\n' "$name" "$LOG_DIR" "$name"
}
stop_group() {
    local pid=$1 deadline=$((SECONDS + 8))
    kill -INT -- "-$pid" 2>/dev/null || true
    while kill -0 -- "-$pid" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.2; done
    if kill -0 -- "-$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}
cleanup() {
    local result=$? i
    trap - EXIT INT TERM
    if ((${#PIDS[@]})); then
        printf '\nデモを終了します。車体の停止も確認してください。\n'
        # 追加ジョブを止めてからNav2へ取消を送る。取消返信は実車停止の証明ではない。
        for ((i=${#PIDS[@]}-1; i>=0; i--)); do
            if [[ "${NAMES[i]}" == dispatch ]]; then stop_group "${PIDS[i]}"; fi
        done
        if [[ " ${NAMES[*]} " == *' nav2 '* ]]; then
            timeout 4 ros2 service call /navigate_to_pose/_action/cancel_goal \
                action_msgs/srv/CancelGoal '{}' >"$LOG_DIR/cancel.log" 2>&1 || true
        fi
        for ((i=${#PIDS[@]}-1; i>=0; i--)); do
            [[ "${NAMES[i]}" == dispatch ]] || stop_group "${PIDS[i]}"
        done
    fi
    exit "$result"
}
check_children() {
    local i
    for ((i=0; i<${#PIDS[@]}; i++)); do
        if ! kill -0 "${PIDS[i]}" 2>/dev/null; then
            printf 'ERROR: %sが終了しました。ログ末尾:\n' "${NAMES[i]}" >&2
            tail -n 20 "$LOG_DIR/${NAMES[i]}.log" >&2
            return 1
        fi
    done
}
wait_active() {
    local node=$1 deadline=$((SECONDS + 90))
    while ((SECONDS < deadline)); do
        check_children || return 1
        if timeout 3 ros2 lifecycle get "$node" 2>/dev/null | grep -q 'active \[3\]'; then return 0; fi
        sleep 1
    done
    printf 'ERROR: %sが90秒以内にactiveになりませんでした\n' "$node" >&2
    printf '初期位置・TF・入力受信を確認してください。Nav2ログ末尾:\n' >&2
    tail -n 20 "$LOG_DIR/nav2.log" >&2
    return 1
}
confirm_initial_pose() {
    printf '\nRVizでFixed Frame=map、/mapを表示し、2D Pose Estimateで実車位置と向きを指定してください。\n'
    if ! $WITH_RVIZ; then
        printf '別ターミナルのRVizも同じRMW・DDS・ROS_DOMAIN_ID=%sを使用してください。\n' "$DOMAIN_ID"
    fi
    [[ -t 0 ]] || fail '初期位置設定の確認が必要です。対話ターミナルから起動してください'
    printf '設定後、このターミナルでEnterを押してください（終了: Ctrl-C）。\n'
    # 操作者を待っている間も、launchの終了は見逃さない。
    while ! read -r -t 1; do check_children || return 1; done
}
initialize_navigation() {
    # localizerのactiveと地図配信は、初期位置によるmap TF生成より先に成立する。
    if $WITH_RVIZ; then start_process rviz rviz2; fi
    wait_active /lidar_localization
    wait_active /map_server
    confirm_initial_pose
    wait_active /planner_server
    wait_active /controller_server
    wait_active /bt_navigator
    wait_active /velocity_smoother
}
check_requirements() {
    local command device maps_root
    for command in ros2 setsid flock timeout python3; do
        command -v "$command" >/dev/null || fail "必要なコマンドがありません: $command"
    done
    for device in /dev/whill /dev/imu; do
        [[ -c "$device" && -r "$device" && -w "$device" ]] || fail "デバイスが未接続かアクセス権がありません: $device"
    done
    maps_root="${WHILL_MAPS_ROOT:-$REPO_ROOT/docs/maps}"
    for device in "$maps_root/$SITE/static.pcd" "$maps_root/$SITE/occupancy_v2.yaml" "$maps_root/$SITE/waypoints.yaml" \
                  "$HOME/.whill_dispatch_tls/dispatch.crt" "$HOME/.whill_dispatch_tls/dispatch.key"; do
        [[ -r "$device" ]] || fail "必要なファイルがありません: $device"
    done
    # XMLのNICと実機が一致しないままloだけで起動するのを防ぐ。
    python3 - "${CYCLONEDDS_URI#file://}" "$maps_root/$SITE/occupancy_v2.yaml" <<'PY'
import pathlib, sys, xml.etree.ElementTree as ET
import yaml
for elem in ET.parse(sys.argv[1]).getroot().iter():
    if elem.tag.rsplit('}', 1)[-1] == 'NetworkInterface':
        name = elem.get('name')
        if name and not (pathlib.Path('/sys/class/net') / name).exists():
            sys.exit(f'DDS設定のNICがありません: {name}')
path = pathlib.Path(sys.argv[2])
image = pathlib.Path(yaml.safe_load(path.read_text())['image'])
if not (path.parent / image).is_file():
    sys.exit(f'地図画像がありません: {path.parent / image}')
PY
    python3 - <<'PY'
import socket
for port in (8000, 9090):
    with socket.socket() as sock:
        try:
            sock.bind(('0.0.0.0', port))
        except OSError as exc:
            raise SystemExit(f'ポート{port}を使用できません: {exc}')
PY
    local nodes
    nodes=$(timeout 10 ros2 node list 2>/dev/null) || fail 'ROSノード一覧を取得できません'
    if printf '%s\n' "$nodes" | grep -Eq '^/(whill|ekf_filter_node|lidar_localization|failsafe_node|twist_mux|controller_server|dispatch_node|velodyne_driver_node)$'; then
        fail '既存の実機スタックが動いています。起動元で停止してください'
    fi
    if $WITH_RVIZ; then
        command -v rviz2 >/dev/null || fail 'rviz2がありません'
        [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || fail '画面環境がありません。--no-rvizを指定してください'
    fi
}
main() {
    parse_args "$@"
    cd "$REPO_ROOT"
    if $DRY_RUN; then
        printf 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nROS_DOMAIN_ID=%s\nCYCLONEDDS_URI=file://%s/configs/cyclonedds-runtime-min.xml\n' "$DOMAIN_ID" "$REPO_ROOT"
        if $PREFLIGHT_ONLY; then
            printf 'timeout 180 %s/scripts/m6r_preflight.sh\n' "$REPO_ROOT"
        else
            printf 'ros2 launch whill_safety m6r_bringup_launch.py site:=%s\n' "$SITE"
            printf 'ros2 launch whill_navigation nav_launch.py site:=%s map_variant:=v2 speed:=%s\n' "$SITE" "$SPEED"
            printf 'ros2 launch whill_dispatch dispatch_launch.py site:=%s use_mock:=false use_tls:=true\n' "$SITE"
            $WITH_RVIZ && printf 'rviz2\n'
        fi
        return 0
    fi
    load_environment
    if $PREFLIGHT_ONLY; then exec timeout 180 "$REPO_ROOT/scripts/m6r_preflight.sh"; fi
    command -v flock >/dev/null || fail 'flockがありません'
    mkdir -p "$REPO_ROOT/log"
    exec 9>"$REPO_ROOT/log/dispatch-demo.lock"
    flock -n 9 || fail '統合起動スクリプトが既に動いています'
    check_requirements
    LOG_DIR=$(mktemp -d "$REPO_ROOT/log/dispatch-demo-$(date +%Y%m%d-%H%M%S)-XXXXXX")
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    start_process bringup ros2 launch whill_safety m6r_bringup_launch.py "site:=$SITE"
    start_process nav2 ros2 launch whill_navigation nav_launch.py "site:=$SITE" map_variant:=v2 "speed:=$SPEED"
    initialize_navigation
    start_process dispatch ros2 launch whill_dispatch dispatch_launch.py "site:=$SITE" use_mock:=false use_tls:=true
    printf '\n起動コマンドを開始しました。Web: https://localhost:8000/\n'
    printf 'タブレット: https://<このPCのIP>:8000/\nログ: %s\n' "$LOG_DIR"
    printf 'RVizでmapと初期位置を設定後、別ターミナルで以下を実行してください:\n  %s/scripts/start_dispatch_demo.sh --domain %s --preflight\n' "$REPO_ROOT" "$DOMAIN_ID"
    printf '入力受信・収束・Web接続を確認してから配車してください。終了: Ctrl-C\n'
    while :; do check_children; sleep 1; done
}
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then main "$@"; fi
