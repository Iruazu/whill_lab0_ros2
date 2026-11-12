#!/usr/bin/env bash
# udev の別名（/dev/whill、/dev/imu）が在り、規則どおりの機器を指すかを確かめる（#141）。
# 全部を確かめて、問題のある別名をすべて表示する。1つでも問題があれば非0で終わる。
# SPDX-License-Identifier: BSD-3-Clause
#
# 2026-10-30、ttyACM0 が在るのに /dev/imu が無かった。IMU の launch の既定は /dev/imu
# なので、別名が無いと IMU ノードは起動に失敗する。driver の起動ログを見落とすと走行前に
# 気づけないため、ここで直接確かめる。
#
# VID:PID はこのファイルに書かず、udev/99-whill-stack.rules から読む。規則を直したときに
# 検査だけ古い値のまま残るのを防ぐ。udev の ATTRS は「親を遡ったどれか1つの機器で、
# 全部の ATTRS が揃って合う」ときに成り立つ。ここでも idVendor と idProduct を同じ機器の
# 組として見る（tty の上の usb_device が両方を持つ）。
set -u

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
RULES=${WHILL_UDEV_RULES:-$ROOT/udev/99-whill-stack.rules}
INSTALLED=${WHILL_UDEV_INSTALLED:-/etc/udev/rules.d/99-whill-stack.rules}
# 試験で偽の /dev を使うため。実機では変えない。
DEV_DIR=${WHILL_DEV_DIR:-/dev}
# 偽の /dev には文字デバイスを作れない（root が要る）。種類の検査は本物の /dev に限る。
REAL_DEV=0
[[ "$DEV_DIR" == /dev ]] && REAL_DEV=1

if [[ ! -r "$RULES" ]]; then
    printf '   規則ファイルがありません: %s\n' "$RULES"
    exit 1
fi
# udevadm が無いと全部が「不一致」や「未接続」に化けるので、先に別の理由として落とす。
if ! command -v udevadm >/dev/null; then
    printf '   udevadm がありません。udev の属性を確かめられません\n'
    exit 1
fi

# attribute-walk の出力から、idVendor と idProduct を両方持つ最初の機器の組を返す。
# 機器の区切りは「looking at」の行。片方だけ持つ機器の値を混ぜない。
usb_ids() {  # usb_ids <attribute-walk の出力>  → "vid:pid"（無ければ空）
    printf '%s\n' "$1" | awk '
        /looking at/ { if (v != "" && p != "") exit; v = ""; p = ""; next }
        match($0, /ATTRS\{idVendor\}=="[^"]*"/)  { v = substr($0, RSTART + 18, RLENGTH - 19) }
        match($0, /ATTRS\{idProduct\}=="[^"]*"/) { p = substr($0, RSTART + 19, RLENGTH - 20) }
        END { if (v != "" && p != "") print v ":" p }'
}

# 規則に合う tty を探す。別名が消えたときに、機器が無いのか別名だけ無いのかを分ける。
find_matching_tty() {  # find_matching_tty <vid> <pid>
    local tty walk
    for tty in "$DEV_DIR"/ttyACM* "$DEV_DIR"/ttyUSB*; do
        [[ -e "$tty" ]] || continue
        walk=$(udevadm info --attribute-walk --name="$tty" 2>/dev/null) || continue
        if [[ "$(usb_ids "$walk")" == "$1:$2" ]]; then
            printf '%s\n' "$tty"
            return 0
        fi
    done
    return 1
}

print_recovery() {  # print_recovery <alias> </dev/ttyXXX>
    # 戻す前に記録を残す。S: に別名があるのにリンクが無いのか、そもそも規則が適用されて
    # いないのかで、原因の候補が変わる（#141 は未解明）。
    printf '   原因調査のため、戻す前に記録を残す:\n'
    printf '     udevadm info --query=all --name=%s > ~/udev-%s-%s.txt\n' "$2" "$1" "$(date +%Y%m%d-%H%M%S)"
    printf '     ls -l /run/udev/links/ | grep -i %s\n' "$1"
    printf '     journalctl -k --since "-2h" | grep -iE "usb|tty|acm|suspend|resume"\n'
    printf '     systemctl is-active ModemManager\n'
    printf '   戻す:\n'
    printf '     sudo udevadm trigger --action=change --name-match=%s\n' "$2"
    printf '     ls -l /dev/%s\n' "$1"
}

problems=0
checked=0
# 最終行に改行が無い規則ファイルでも、最後の規則を読み落とさない。
while IFS= read -r line || [[ -n "$line" ]]; do
    [[ "$line" =~ ^[[:space:]]*# ]] && continue
    [[ "$line" =~ ATTRS\{idVendor\}==\"([0-9a-fA-F]+)\" ]] || continue
    vid=${BASH_REMATCH[1]}
    [[ "$line" =~ ATTRS\{idProduct\}==\"([0-9a-fA-F]+)\" ]] || continue
    pid=${BASH_REMATCH[1]}
    [[ "$line" =~ SYMLINK\+=\"([^\"]+)\" ]] || continue
    alias=${BASH_REMATCH[1]}
    checked=$((checked + 1))
    path="$DEV_DIR/$alias"

    if [[ -L "$path" && ! -e "$path" ]]; then
        # 別名が無い場合と分ける。udev がリンクを消し損ねたのか、tty が消えたのかの手がかり。
        printf '   /dev/%-6s リンクはあるが指す先が無い: %s\n' "$alias" "$(readlink "$path")"
        problems=1
        if tty=$(find_matching_tty "$vid" "$pid"); then
            print_recovery "$alias" "/dev/$(basename "$tty")"
        fi
        continue
    fi

    if [[ -e "$path" ]]; then
        target=$(readlink -f "$path")
        if ! walk=$(udevadm info --attribute-walk --name="$target" 2>/dev/null); then
            printf '   /dev/%-6s -> %s  udevadm で属性を読めません\n' "$alias" "$target"
            problems=1
            continue
        fi
        got=$(usb_ids "$walk")
        if [[ "$got" != "$vid:$pid" ]]; then
            printf '   /dev/%-6s -> %s  規則は %s:%s、実物は %s\n' "$alias" "$target" "$vid" "$pid" "${got:-USB の VID:PID なし}"
            problems=1
        elif ((REAL_DEV)) && [[ ! -c "$target" ]]; then
            printf '   /dev/%-6s -> %s  文字デバイスではありません\n' "$alias" "$target"
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
        print_recovery "$alias" "$name"
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
