#!/usr/bin/env bash
# 実機デモの環境を揃え、所有するlaunchだけをまとめて起動・終了する。
# SPDX-License-Identifier: BSD-3-Clause
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 既定は campus-m8（2026-11-13、#190）。旧 campus は --site campus で選ぶ fallback。
SITE=campus-m8
# 空なら site から決める（demo_variant_for）。
MAP_VARIANT=
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
  --site NAME       地図site（既定: campus-m8。旧地図は campus）
  --variant NAME    Nav2の map_variant（既定: campus-m8 は candidate、それ以外は v2）
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
--site campus で起動するときは、Webの背景地図を旧地図で作り直してから
whill_dispatchをビルドする（runbook。作り直さないと地図の同期検査で止まる）。
EOF
}
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
# デモで nav_launch に渡す map_variant。campus-m8 は手塗りの確定前で occupancy.yaml が
# 無く、候補の occupancy_candidate.yaml を使う。旧 campus と他の site は従来どおり v2。
# check_map_sync.py の DEMO_VARIANT_BY_SITE と同じ対応にする。
demo_variant_for() {
    case "$1" in
        campus-m8) printf 'candidate' ;;
        *) printf 'v2' ;;
    esac
}
parse_args() {
    while (($#)); do
        case "$1" in
            --site|--variant|--speed|--domain)
                (($# >= 2)) || fail "$1 に値が必要です"
                case "$1" in
                    --site) SITE=$2 ;;
                    --variant) MAP_VARIANT=$2 ;;
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
    [[ -n "$MAP_VARIANT" ]] || MAP_VARIANT=$(demo_variant_for "$SITE")
    [[ "$MAP_VARIANT" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]] || fail 'variant名が不正です'
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
    # launchごとに新しいsession（pgid=sid=pid）を作り、子ノードもそこへ入れる。端末の
    # Ctrl-C・HUPがlaunchへ直接届かず、止める順序をcleanupだけが決められる。
    # bashはjob control無しの非同期コマンドでSIGINT/SIGQUITを無視に設定し、それがexec後も
    # 残る。ros2 launchはその状態だとSIGINTで何もせず、SIGTERMで親だけ落ちて子が孤児化
    # する（#194で再現）。env --default-signalで既定に戻し、launch自身の終了処理を使う。
    # PIPEも戻すのは、cleanupで無視にした後に起動しても無視が継承されないようにするため。
    # 前提: このsubshellはprocess groupの先頭でない（set -mを使わない）。先頭だとsetsidが
    # forkし、$!とsession IDがずれる。
    setsid env --default-signal=INT,QUIT,PIPE "$@" >"$LOG_DIR/$name.log" 2>&1 9>&- &
    PIDS+=("$!")
    NAMES+=("$name")
    # session IDを出しておくと、スクリプトが消えた後でも pgrep -a -s で残りを特定できる。
    say '起動: %s（session %s, log: %s/%s.log）\n' "$name" "$!" "$LOG_DIR" "$name"
}
# 端末が閉じた後の書き込みはEIOで失敗する。set -eのcleanupがそこで中断し、launchを
# 止めないまま抜けた（#194）。表示の失敗は終了処理を妨げない。
# shellcheck disable=SC2059  # 書式は呼び出し側の固定文字列だけを渡す。
say() { printf "$@" 2>/dev/null || true; }
# sessionに残るプロセスを SCAN_PIDS / SCAN_LINES（"pid args"）へ入れる。
# $(pgrep ...) は使わない。コマンド置換のsubshellはcleanupのINT trapを持たず、二度目の
# Ctrl-Cで空のまま終わる。空を「残り無し」と読むと、早すぎるTERMや誤った完了表示になる。
# pgrepの終了コードは0=有り・1=無し。それ以外（中断など）は取り直し、続けば非0を返す。
SCAN_PIDS=()
SCAN_LINES=()
scan_sessions() {  # scan_sessions <sid[,sid...]>
    local rc=3 attempt line file="${LOG_DIR:-/tmp}/session-scan.txt"
    for attempt in 1 2 3 4 5; do
        # 「無し」の1でset -eに落とされないよう、終了コードは||で受ける。
        pgrep -a -s "$1" >"$file" 2>/dev/null && rc=0 || rc=$?
        if ((rc == 0 || rc == 1)); then
            SCAN_LINES=()
            mapfile -t SCAN_LINES <"$file" || return 3
            SCAN_PIDS=()
            for line in "${SCAN_LINES[@]}"; do SCAN_PIDS+=("${line%% *}"); done
            return 0
        fi
    done
    return "$rc"
}
# 数えられないときは「残っている」とみなす。止める側を続けるほうが安全なため。
session_alive() { scan_sessions "$1" || return 0; ((${#SCAN_PIDS[@]})); }
pgid_of() {  # pgid_of <pid>: PGIDへ入れる。/proc/<pid>/stat の ")" 以降の3番目。
    local stat fields
    read -r stat <"/proc/$1/stat" 2>/dev/null || return 1
    read -r -a fields <<<"${stat##*) }"
    PGID=${fields[2]}
}
signal_session() {  # signal_session <SIG> <sid>
    local pid
    kill "-$1" -- "-$2" 2>/dev/null || true
    scan_sessions "$2" || return 0
    for pid in "${SCAN_PIDS[@]}"; do
        # INTはgroup宛てで既に届いている。launchも子へ転送するので、二重に送ると
        # 終了処理中のノードを割り込ませる。groupを抜けたものにだけ個別に送る。
        if [[ "$1" == INT ]] && pgid_of "$pid" && [[ "$PGID" == "$2" ]]; then continue; fi
        kill "-$1" "$pid" 2>/dev/null || true
    done
}
stop_group() {  # stop_group <sid> [INTの猶予秒]
    local pid=$1 deadline=$((SECONDS + ${2:-8}))
    signal_session INT "$pid"
    while session_alive "$pid" && ((SECONDS < deadline)); do sleep 0.2; done
    if session_alive "$pid"; then
        signal_session TERM "$pid"
        deadline=$((SECONDS + 3))
        while session_alive "$pid" && ((SECONDS < deadline)); do sleep 0.2; done
        signal_session KILL "$pid"
    fi
    wait "$pid" 2>/dev/null || true
}
started_sessions() { printf -v SIDS '%s,' "${PIDS[@]}"; SIDS=${SIDS%,}; }
# 個別のstop_groupが時間切れや中断で漏らしたものを最後にもう一度探し、名前を出して止める。
sweep_leftovers() {
    local i
    ((${#PIDS[@]})) || return 0
    started_sessions
    if ! scan_sessions "$SIDS"; then
        say 'ERROR: 残りのプロセスを確認できません（pgrep -a -s %s）\n' "$SIDS" >&2
        return 1
    fi
    ((${#SCAN_LINES[@]})) || return 0
    say '残っているプロセスを停止します:\n'
    say '  %s\n' "${SCAN_LINES[@]}"
    # 一度INTを受けて残ったものなので、INTの猶予は短くする。
    for ((i=0; i<${#PIDS[@]}; i++)); do stop_group "${PIDS[i]}" 2; done
    if ! scan_sessions "$SIDS" || ((${#SCAN_LINES[@]})); then
        say 'ERROR: 停止できないプロセスがあります（pgrep -a -s %s）:\n' "$SIDS" >&2
        say '  %s\n' "${SCAN_LINES[@]}" >&2
        return 1
    fi
}
cleanup() {
    local result=$? i
    # 終了処理の途中で落ちるとbringupだけ残る。二度目のCtrl-Cや端末の終了で中断させない。
    # sleepなど前景の子はCtrl-Cで終わるが、ループは続く。INT/TERM/HUPを無視（trap ''）に
    # しないのは、無視が後に起こすtimeout配下の子へ継承され、timeoutのTERMが効かなくなるため。
    # PIPEは無視する。| tee の tee がCtrl-Cで先に終わると最初の表示でSIGPIPEになり、
    # stop_groupの前にbashが落ちる。ここで起こす子はファイルへ書くだけなので継承は害が無い。
    trap - EXIT
    trap 'say "\n終了処理中です。完了まで待ってください。\n"' INT
    trap ':' TERM HUP
    trap '' PIPE
    set +e
    if ((${#PIDS[@]})); then
        say '\nデモを終了します。車体の停止も確認してください。\n'
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
        started_sessions
        if sweep_leftovers; then
            say '終了しました。残りの確認: pgrep -a -s %s\n' "$SIDS"
        else
            result=1
            say '止め方: 上のPIDを確かめて kill -TERM <PID>。全体は pgrep -af -- '"'"'ros2 launch|--ros-args'"'"'\n' >&2
        fi
    fi
    exit "$result"
}
# 確認手順（#194）: 起動後にCtrl-C、または端末を閉じてから、起動時に表示したsessionで
#   pgrep -a -s <sid,sid,...>
# が何も出さないこと（sessionが分からなければ pgrep -af -- 'ros2 launch|--ros-args'。
# 別ターミナルのbag再生などのノードも出るので、PIDを確かめてから止める）。
# scripts/tests/test_dispatch_demo_launcher.py が無害なプロセスで同じ経路を試す。
install_traps() {
    # 端末を閉じた・SSHが切れた場合もHUPでcleanupへ入る。trapの無いHUPではbashがEXIT
    # trapを実行せずに終わり、launchがsystemd --userの下に残った（#194）。
    # PIPEはhandlerにする（無視ではない）。handlerはexec後の子へ継承されない。
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP
    trap 'exit 141' PIPE
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
# static.pcd は Git 管理外なので、checkout ごとに正本を複製する。無いときは起動前に止め、
# 複製のコマンドをその場で出す（launch のエラーまで進むと、bringup だけ落ちて分かりにくい）。
# 正本の場所と sha256 は docs/maps/<site>/metadata.yaml の dufomap_output / static_pcd_sha256。
# sha256 を照合するのは、取り違えた複製（作り直した別の static.pcd、旧地図の PCD など）で
# 走ると、#156 の受入結果がどの地図の結果か曖昧になるのを防ぐため。GPU の GLIM は非決定的で、
# 作り直すと同じ地図にならないので、点数やサイズではなく sha256 で比べる。
check_static_pcd() {
    local maps_root=$1 site=$2
    python3 - "$maps_root/$site" "$REPO_ROOT" <<'PY'
import hashlib, os, pathlib, sys
import yaml
site_dir, repo = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
pcd, meta_path = site_dir / 'static.pcd', site_dir / 'metadata.yaml'
meta = {}
if meta_path.is_file():
    try:
        meta = yaml.safe_load(meta_path.read_text()) or {}
    except yaml.YAMLError as e:
        sys.exit(f'metadata.yaml が読めません: {meta_path}: {e}')
expected = str(meta.get('static_pcd_sha256') or '').strip().lower()
source = meta.get('dufomap_output')
# WHILL_MAPS_ROOT で別の地図ディレクトリを使うと README が無いことがある。手順はリポジトリ側を指す。
readme = site_dir / 'README.md'
if not readme.is_file():
    readme = repo / 'docs' / 'maps' / site_dir.name / 'README.md'
if not pcd.is_file():
    lines = [f'static.pcd がありません: {pcd}',
             '  static.pcd は Git 管理外。正本を複製する（作り直さない）:']
    if source:
        lines.append(f'    cp {os.path.expanduser(source)} {pcd}')
        if expected:
            lines.append(f'    echo "{expected}  {pcd}" | sha256sum -c')
    else:
        lines.append('    （metadata.yaml に正本の場所 dufomap_output がありません）')
    lines.append(f'  手順: {readme}（runbook 1 節も参照）')
    sys.exit('\n'.join(lines))
if expected:
    h = hashlib.sha256()
    with pcd.open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    if h.hexdigest() != expected:
        lines = [f'static.pcd の sha256 が metadata.yaml と違います: {pcd}',
                 f'  期待: {expected}', f'  実際: {h.hexdigest()}',
                 '  別の地図の複製か、作り直したものです。この static.pcd では起動しません。']
        if source:
            lines.append(f'  正本から複製し直す: cp {os.path.expanduser(source)} {pcd}')
        lines.append(f'  手順: {readme}')
        sys.exit('\n'.join(lines))
PY
}
check_requirements() {
    local command device maps_root
    for command in ros2 setsid flock timeout python3 pgrep; do
        command -v "$command" >/dev/null || fail "必要なコマンドがありません: $command"
    done
    # 別名が無いのか、機器が無いのか、権限が無いのかで直し方が違う。理由と戻し方まで
    # 表示する検査を使う（#141）。ここで止まると preflight の check 0b まで進まない。
    "$REPO_ROOT/scripts/check_device_aliases.sh" || fail 'デバイスの別名が使えません（上の表示を参照）'
    maps_root="${WHILL_MAPS_ROOT:-$REPO_ROOT/docs/maps}"
    check_static_pcd "$maps_root" "$SITE" || fail 'static.pcd を使えません（上の表示を参照）'
    for device in "$maps_root/$SITE/occupancy_$MAP_VARIANT.yaml" "$maps_root/$SITE/waypoints.yaml" \
                  "$HOME/.whill_dispatch_tls/dispatch.crt" "$HOME/.whill_dispatch_tls/dispatch.key"; do
        [[ -r "$device" ]] || fail "必要なファイルがありません: $device"
    done
    # XMLのNICと実機が一致しないままloだけで起動するのを防ぐ。
    python3 - "${CYCLONEDDS_URI#file://}" "$maps_root/$SITE/occupancy_$MAP_VARIANT.yaml" <<'PY'
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
    # Webの背景地図はNav2の地図と別の成果物で、手で再生成している（#171）。ずれたままだと、
    # タップした地点とNav2のゴールが食い違う。配信されるのはinstall側のweb/なので、そちらを見る。
    local web_dir
    web_dir="$(ros2 pkg prefix whill_dispatch)/share/whill_dispatch/web" || fail 'whill_dispatchが見つかりません'
    python3 "$REPO_ROOT/scripts/check_map_sync.py" --maps-root "$maps_root" --site "$SITE" \
        --variant "$MAP_VARIANT" --web-dir "$web_dir" --quiet-notes || fail 'Webの背景地図がNav2の地図と揃っていません（上の表示を参照）'
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
            printf 'ros2 launch whill_navigation nav_launch.py site:=%s map_variant:=%s speed:=%s\n' "$SITE" "$MAP_VARIANT" "$SPEED"
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
    install_traps
    start_process bringup ros2 launch whill_safety m6r_bringup_launch.py "site:=$SITE"
    start_process nav2 ros2 launch whill_navigation nav_launch.py "site:=$SITE" "map_variant:=$MAP_VARIANT" "speed:=$SPEED"
    initialize_navigation
    start_process dispatch ros2 launch whill_dispatch dispatch_launch.py "site:=$SITE" use_mock:=false use_tls:=true
    printf '\n起動コマンドを開始しました。Web: https://localhost:8000/\n'
    printf 'タブレット: https://<このPCのIP>:8000/\nログ: %s\n' "$LOG_DIR"
    printf 'RVizでmapと初期位置を設定後、別ターミナルで以下を実行してください:\n  %s/scripts/start_dispatch_demo.sh --domain %s --preflight\n' "$REPO_ROOT" "$DOMAIN_ID"
    printf '入力受信・収束・Web接続を確認してから配車してください。終了: Ctrl-C\n'
    while :; do check_children; sleep 1; done
}
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then main "$@"; fi
