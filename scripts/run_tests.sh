#!/usr/bin/env bash
# 自作パッケージと scripts/tests のテストを1コマンドで回す入口（#137）。
# 実機もノードの常駐も要らない。失敗があれば非0で終わる。
# SPDX-License-Identifier: BSD-3-Clause
set -o pipefail

usage() {
    cat <<'EOF'
使用法: scripts/run_tests.sh [オプション]
  --no-build      ビルドを省き、前回の .test_ws を使う
  --scripts-only  scripts/tests だけを回す（ビルドしない。数秒で終わる）
  -h, --help      この説明を表示

作業用の出力は <repo>/.test_ws/ に置く。何度でも実行できる。
デモが使う install/ は変えない。上流パッケージは main 側の install/ を土台にする。
EOF
}

BUILD=1
COLCON=1
while (($#)); do
    case "$1" in
        --no-build) BUILD=0 ;;
        --scripts-only) BUILD=0; COLCON=0 ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'ERROR: 不明なオプション: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS="$ROOT/.test_ws"
# worktree でも main 側を指す。上流パッケージのビルドは main 側にしか無い。
MAIN_ROOT=$(dirname "$(git -C "$ROOT" rev-parse --path-format=absolute --git-common-dir)")

# 走行中のスタックと ROS グラフが混ざらないようにする。
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
# ~/.local の anyio の pytest プラグインが、システムの pytest 6.2.5 を起動時に壊す
# （ModuleNotFoundError: _pytest.scope）。テストは外部プラグインを使わない。
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# setup.bash は未定義変数を参照するので、set -u の外で読む。
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

RESULTS=()
FAILED=0
record() {  # record <名前> <結果> [補足]
    RESULTS+=("$(printf '%-14s %-5s %s' "$1" "$2" "${3:-}")")
    [[ "$2" == OK || "$2" == SKIP ]] || FAILED=1
}

colcon_steps() {
    local underlay="$MAIN_ROOT/install/setup.bash"
    if [[ ! -r "$underlay" ]]; then
        record build FAIL "$underlay が無い。docs/development.md のビルドを先に行う"
        return
    fi
    # shellcheck disable=SC1090
    source "$underlay"

    # whill_perception は Patchwork++ を ../third_party から直接組み込む。
    # src/third_party は git 管理外で、worktree には無い。
    if [[ ! -e "$ROOT/src/third_party" ]]; then
        ln -s "$MAIN_ROOT/src/third_party" "$ROOT/src/third_party"
    fi

    local paths=("$ROOT"/src/whill_*)
    local packages=()
    local p
    for p in "${paths[@]}"; do packages+=("$(basename "$p")"); done
    # --paths: 上流を探索しない。--*-base: デモが使う build/ install/ を変えない。
    local bases=(--build-base "$WS/build" --install-base "$WS/install")
    # pytest の結果はビルド時に決まる build/<pkg>/test_results に出る。test 時だけ別の
    # --test-result-base を渡すと、ctest の粗い結果しか数えられなくなる。
    local results="$WS/build"

    if ((BUILD)); then
        if colcon --log-base "$WS/log" build --paths "${paths[@]}" "${bases[@]}" \
                --symlink-install --event-handlers console_direct-; then
            record build OK "${#packages[@]} packages"
        else
            record build FAIL "ログ: $WS/log/latest_build"
            return
        fi
    elif [[ ! -d "$WS/install" ]]; then
        record build FAIL '.test_ws が無い。--no-build を外して実行する'
        return
    else
        record build SKIP
    fi

    # 前回の結果が残ると、今回走らなかったテストを成功と数えてしまう。
    find "$results" -type d -name test_results -prune -exec rm -rf {} +
    colcon --log-base "$WS/log" test --paths "${paths[@]}" "${bases[@]}" \
        --event-handlers console_direct- \
        --return-code-on-test-failure
    local test_rc=$?
    local summary
    summary=$(colcon test-result --test-result-base "$results" --verbose 2>&1)
    local result_rc=$?
    printf '%s\n' "$summary"

    # テストを宣言したのに結果が無いパッケージは失敗にする。プラグインの破損で
    # pytest が起動しないと、失敗ではなく「0件」として素通りした前例がある。
    local missing=()
    for p in "${paths[@]}"; do
        if grep -qE '^[[:space:]]*ament_add_' "$p/CMakeLists.txt" \
                && ! find "$results/$(basename "$p")/test_results" -name '*.xml' 2>/dev/null | grep -q .; then
            missing+=("$(basename "$p")")
        fi
    done
    local line
    line=$(printf '%s\n' "$summary" | grep -E '^Summary:' | tail -1)
    if ((${#missing[@]})); then
        record colcon-test FAIL "結果が無い: ${missing[*]}"
    elif ((test_rc != 0 || result_rc != 0)); then
        record colcon-test FAIL "${line:-結果なし}"
    else
        record colcon-test OK "${line:-宣言されたテストなし}"
    fi
}

mkdir -p "$WS"
if ((COLCON)); then
    colcon_steps
else
    record build SKIP
    record colcon-test SKIP
fi

# pytest は0件のとき終了コード5を返すので、収集漏れも失敗になる。
echo '==> scripts/tests'
PYTEST_OUT=$(python3 -m pytest "$ROOT/scripts/tests" -q 2>&1)
PYTEST_RC=$?
printf '%s\n' "$PYTEST_OUT"
if ((PYTEST_RC == 0)); then
    record scripts-tests OK "$(printf '%s\n' "$PYTEST_OUT" | tail -1)"
else
    record scripts-tests FAIL "pytest 終了コード $PYTEST_RC"
fi

echo
echo '==> 結果'
printf '  %s\n' "${RESULTS[@]}"
exit "$FAILED"
