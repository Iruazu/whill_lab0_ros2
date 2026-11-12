#!/usr/bin/env bash
# udev の別名（/dev/whill、/dev/imu）が在り、規則どおりの機器を指すかを確かめる（#141）。
# 全部を確かめて、問題のある別名をすべて表示する。1つでも問題があれば非0で終わる。
# SPDX-License-Identifier: BSD-3-Clause
#
# 2026-10-30、ttyACM0（2b72:0003）が在るのに /dev/imu が無かった。IMU の launch の既定は
# /dev/imu なので、別名が無いと IMU ノードは起動に失敗する。driver の起動ログを見落とすと
# 走行前に気づけないため、ここで直接確かめる。
#
# VID:PID はこのファイルに書かず、udev/99-whill-stack.rules から読む。規則を直したときに
# 検査だけ古い値のまま残るのを防ぐ。照合は規則と同じく ATTRS（親を遡った最初の値）で行う。
set -u

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
RULES=${WHILL_UDEV_RULES:-$ROOT/udev/99-whill-stack.rules}
INSTALLED=${WHILL_UDEV_INSTALLED:-/etc/udev/rules.d/99-whill-stack.rules}
# 試験で偽の /dev を使うため。実機では変えない。
DEV_DIR=${WHILL_DEV_DIR:-/dev}

if [[ ! -r "$RULES" ]]; then
    printf '   規則ファイルがありません: %s\n' "$RULES"
    exit 1
fi

# 親を遡って最初に出る ATTRS{key} の値。tty の上の usb_device の値が最初に出る。
first_attr() {  # first_attr <attribute-walk の出力> <key>
    printf '%s\n' "$1" | sed -n "s/^ *ATTRS{$2}==\"\\([^\"]*\\)\".*/\\1/p" | head -n 1
}

# 規則に合う tty を探す。別名が消えたときに、機器が無いのか別名だけ無いのかを分ける。
find_matching_tty() {  # find_matching_tty <vid> <pid>
    local tty walk
    for tty in "$DEV_DIR"/ttyACM* "$DEV_DIR"/ttyUSB*; do
        [[ -e "$tty" ]] || continue
        walk=$(udevadm info --attribute-walk --name="$tty" 2>/dev/null) || continue
        if [[ "$(first_attr "$walk" idVendor)" == "$1" && "$(first_attr "$walk" idProduct)" == "$2" ]]; then
            printf '%s\n' "$tty"
            return 0
        fi
    done
    return 1
}

problems=0
checked=0
while IFS= read -r line; do
    [[ "$line" =~ ^[[:space:]]*# ]] && continue
    [[ "$line" =~ ATTRS\{idVendor\}==\"([0-9a-fA-F]+)\" ]] || continue
    vid=${BASH_REMATCH[1]}
    [[ "$line" =~ ATTRS\{idProduct\}==\"([0-9a-fA-F]+)\" ]] || continue
    pid=${BASH_REMATCH[1]}
    [[ "$line" =~ SYMLINK\+=\"([^\"]+)\" ]] || continue
    alias=${BASH_REMATCH[1]}
    checked=$((checked + 1))
    path="$DEV_DIR/$alias"

    if [[ -e "$path" ]]; then
        target=$(readlink -f "$path")
        walk=$(udevadm info --attribute-walk --name="$target" 2>/dev/null)
        got="$(first_attr "$walk" idVendor):$(first_attr "$walk" idProduct)"
        if [[ "$got" != "$vid:$pid" ]]; then
            printf '   /dev/%-6s -> %s  規則は %s:%s、実物は %s\n' "$alias" "$target" "$vid" "$pid" "$got"
            problems=1
        elif [[ ! -r "$path" || ! -w "$path" ]]; then
            printf '   /dev/%-6s -> %s  読み書きできません（dialout に入っているか確認: scripts/grant_serial_access.sh）\n' \
                "$alias" "$target"
            problems=1
        else
            printf '   /dev/%-6s -> %s  ok (%s:%s)\n' "$alias" "$target" "$vid" "$pid"
        fi
        continue
    fi

    problems=1
    if tty=$(find_matching_tty "$vid" "$pid"); then
        name="/dev/$(basename "$tty")"
        printf '   /dev/%-6s 無し。ただし %s が規則（%s:%s）に合っている\n' "$alias" "$name" "$vid" "$pid"
        # 戻す前に udev の記録を残す。S: に別名があるのにリンクが無いのか、そもそも
        # 規則が適用されていないのかで、原因の候補が変わる（#141 は未解明）。
        printf '   原因調査のため、先に記録を残す:\n'
        printf '     udevadm info --query=all --name=%s > ~/udev-%s-%s.txt\n' "$name" "$alias" "$(date +%Y%m%d-%H%M%S)"
        printf '   戻す:\n'
        printf '     sudo udevadm trigger --action=change --name-match=%s\n' "$name"
        printf '     ls -l /dev/%s\n' "$alias"
    else
        printf '   /dev/%-6s 無し。規則（%s:%s）に合う ttyACM / ttyUSB も無い。USB の接続と電源を確認\n' \
            "$alias" "$vid" "$pid"
    fi
done <"$RULES"

if ((checked == 0)); then
    printf '   規則ファイルに SYMLINK+= の規則がありません: %s\n' "$RULES"
    exit 1
fi

# 入っている規則がリポジトリと違えば、別名の問題の原因になり得る。コメントの差は無視する。
if ((problems)); then
    rules_body() { grep -vE '^[[:space:]]*(#|$)' "$1"; }
    if [[ ! -r "$INSTALLED" ]]; then
        printf '   規則が入っていません: %s（scripts/install_udev_rules.sh）\n' "$INSTALLED"
    elif ! diff -q <(rules_body "$RULES") <(rules_body "$INSTALLED") >/dev/null; then
        printf '   入っている規則がリポジトリと違います: %s（scripts/install_udev_rules.sh）\n' "$INSTALLED"
    fi
fi
exit "$problems"
