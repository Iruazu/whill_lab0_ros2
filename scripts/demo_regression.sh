#!/usr/bin/env bash
# マージ前のデモ回帰チェック（#142）。手順の説明は docs/regression-check.md。
# 実機は要らず、走行指令も出さない。問題があれば非0で終わる。
# SPDX-License-Identifier: BSD-3-Clause
set -o pipefail

usage() {
    cat <<'EOF_USAGE'
使用法: scripts/demo_regression.sh [--level static|replay] [--bag DIR] [--only failsafe|localizer]
  --level static  テスト、--dry-run、launch の --show-args（既定。1分未満）
  --level replay  上に加えて基準 bag を再生する（約12分）
  --bag DIR       基準 bag（既定: 2026-07-24 の走行 bag）
  --only NAME     再生をその検査だけにする
  --measure       基準と比べず、測定だけする（基準を決め直すとき）
EOF_USAGE
}

LEVEL=static
ONLY=()
MEASURE=()
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MAIN_ROOT=$(dirname "$(git -C "$ROOT" rev-parse --path-format=absolute --git-common-dir)")
# 置き場は docs/bags.md。
BAG="${WHILL_BAGS_ROOT:-$HOME/bags}/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26"
while (($#)); do
    case "$1" in
        --level) LEVEL=${2:-}; shift 2 ;;
        --bag) BAG=${2:-}; shift 2 ;;
        --only) ONLY=(--only "${2:-}"); shift 2 ;;
        --measure) MEASURE=(--no-compare); shift ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'ERROR: 不明なオプション: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
done
case "$LEVEL" in static|replay) ;; *) usage >&2; exit 2 ;; esac

RESULTS=()
FAILED=0
record() {
    RESULTS+=("$(printf '%-12s %-5s %s' "$1" "$2" "${3:-}")")
    [[ "$2" == OK ]] || FAILED=1
}

echo '==> テスト（scripts/run_tests.sh）'
if "$ROOT/scripts/run_tests.sh"; then record tests OK; else record tests FAIL; fi

# run_tests.sh が作った、このブランチのビルドを使う。
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$MAIN_ROOT/install/setup.bash"
if [[ -r "$ROOT/.test_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROOT/.test_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export WHILL_MAPS_ROOT="$MAIN_ROOT/docs/maps"
unset CYCLONEDDS_URI

echo '==> start_dispatch_demo.sh --dry-run'
if "$ROOT/scripts/start_dispatch_demo.sh" --dry-run >/dev/null; then
    record dry-run OK
else
    record dry-run FAIL
fi

echo '==> launch --show-args'
BAD=()
for pair in whill_safety:m6r_bringup_launch.py whill_navigation:nav_launch.py \
        whill_dispatch:dispatch_launch.py; do
    ros2 launch "${pair%%:*}" "${pair##*:}" --show-args >/dev/null 2>&1 || BAD+=("${pair##*:}")
done
if ((${#BAD[@]})); then record show-args FAIL "${BAD[*]}"; else record show-args OK '3 launch'; fi

if [[ "$LEVEL" == replay ]]; then
    OUT="$ROOT/.test_ws/regression/$(date +%Y%m%d-%H%M%S)"
    echo "==> 基準 bag の再生（結果: $OUT）"
    if [[ ! -r "$BAG/metadata.yaml" ]]; then
        record replay FAIL "bag が無い: $BAG（docs/bags.md を参照）"
    elif python3 "$ROOT/scripts/regression/replay_check.py" --bag "$BAG" \
            --baseline "$ROOT/scripts/regression/baseline.json" --out "$OUT" \
            "${ONLY[@]}" "${MEASURE[@]}"; then
        record replay OK "$OUT/result.json"
    else
        record replay FAIL "$OUT"
    fi
fi

echo
echo '==> 結果'
printf '  %s\n' "${RESULTS[@]}"
exit "$FAILED"
