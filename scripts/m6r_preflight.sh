#!/usr/bin/env bash
# m6r_preflight.sh — blocking pre-drive gate for M6-R integration demo.
#
# 2026-07-16 incident: Layer D failed to arm because /scan subscription
# defaulted to RELIABLE while p2ls publishes BEST_EFFORT. The chair drove
# into a person during V2 verification. Non-blocking pre-drive checks
# (rely on operator to eyeball a log line) let this slip through.
#
# This script FAILS LOUD (exit 1) if failsafe_node is not up, if the
# whill driver has no live serial (/whill/odom silent), has DEAD
# INPUT errors within its 10-s watchdog window, or if a live-fire hand
# test shows /cmd_vel_safety not publishing at the expected 20 Hz.
#
# Usage:
#   # After bringup + nav_launch have been running for ~15 s:
#   scripts/m6r_preflight.sh
#   # Only proceed to send navigation goals after this exits 0.

set -u

echo "=== M6-R preflight gate ==="
echo

# ---- 0. environment sanity -----------------------------------------
# CYCLONEDDS_URI が typo / 存在しないファイルを指すと cyclone は participant
# を作れず、以降の全 ros2 コマンドが即エラー終了する。stderr を捨てる後続
# check ではこれが「Layer D not publishing」に化ける (2026-07-19 field で
# whill_labo_ros2 の typo により実際に発生、原因特定に ~40 分を消費)。
# ここで環境自体を先に検証して、env 壊れは env 壊れとして落とす。
echo -n "0. environment (CYCLONEDDS_URI / DDS participant) ... "
if [ -n "${CYCLONEDDS_URI:-}" ]; then
    _xml="${CYCLONEDDS_URI#file://}"
    if [ ! -r "$_xml" ]; then
        echo "FAIL"
        echo "   CYCLONEDDS_URI points to a missing file: $_xml"
        echo "   Fix the export (typo?) and re-run. Do NOT drive."
        exit 1
    fi
fi
if ! timeout 10 ros2 topic list >/dev/null 2>&1; then
    echo "FAIL"
    echo "   'ros2 topic list' failed — DDS participant cannot start in this"
    echo "   terminal (check RMW_IMPLEMENTATION / CYCLONEDDS_URI / sourcing)."
    exit 1
fi
echo "PASS"

# ---- 1. controller_server: use_collision_detection: true -----------
echo -n "1. use_collision_detection ... "
val=$(ros2 param get /controller_server FollowPath.use_collision_detection 2>&1)
if ! echo "$val" | grep -q "Boolean value is: True"; then
    echo "FAIL"
    echo "   Got: $val"
    exit 1
fi
echo "PASS"

# ---- 2. failsafe_node running --------------------------------------
echo -n "2. failsafe_node alive ... "
if ! ros2 node list 2>/dev/null | grep -qx "/failsafe_node"; then
    echo "FAIL (not in ros2 node list)"
    exit 1
fi
echo "PASS"

# ---- 3. whill driver serial alive ----------------------------------
# 2026-07-22 field: the WHILL enumerated as ttyUSB1 while the driver's
# port_name default was /dev/ttyUSB0 — the driver came up with no live
# serial, /whill/odom stayed silent, and every Nav2 motion primitive
# died on "Failed to make progress" (the chair never moved). This gate
# catches a dead driver before the first goal instead. The launch-side
# fix (port_name -> /dev/whill udev symlink) removes the enumeration
# dependence, but a gate stays: it also catches unplugged cable / dead
# WHILL power, which no port override can fix. /whill/odom is ~2.5 Hz,
# so 5 s is generous. echo, not `topic hz` — see the check-5 rationale.
echo -n "3. whill driver serial (/whill/odom) ... "
if ! timeout 5 ros2 topic echo /whill/odom --once >/dev/null 2>&1; then
    echo "FAIL — no /whill/odom within 5 s"
    echo "   The whill driver has no live serial (or the WHILL is off)."
    echo "   Check:  ls -la /dev/whill /dev/ttyUSB*"
    echo "   then replug the WHILL USB / power-cycle the chair and restart"
    echo "   the bringup terminal. Do NOT drive."
    exit 1
fi
echo "PASS"

# ---- 4. Wait past dead-input watchdog window -----------------------
# failsafe_node's STARTUP_DEAD_INPUT_TIMEOUT_S = 10 s. If the
# subscriptions did not arm within that budget, an ERROR line shows
# up on /rosout with the substring "DEAD INPUT".
echo "4. dead-input watchdog: waiting 12 s to catch any ERROR ..."
sleep 12
if ros2 topic echo --once --qos-durability transient_local /rosout 2>/dev/null \
        | grep -q "DEAD INPUT"; then
    echo "   FAIL — /rosout carries DEAD INPUT from failsafe_node"
    echo "   Fix: check publisher QoS on the reported topic(s), or check"
    echo "        that the upstream (localizer / patchworkpp / p2ls) is"
    echo "        running. Do NOT drive."
    exit 1
fi
echo "   PASS (no DEAD INPUT reported)"

# ---- 5. Live-fire Layer D test -------------------------------------
echo "5. Layer D live fire test."
echo "   >>> Have a person walk to ~1.5 m directly ahead of the chair"
echo "   >>> and stand still. No rush — waiting up to 30 s for detection."
# /cmd_vel_safety は遮断中しか publish されない。固定窓での計測は check 4
# の 12 s 無言待ちの間に人が持ち場を離れる/戻り遅れると偽陽性 FAIL する
# (2026-07-19 field で 3 連発を実測。failsafe ログの ENGAGED は毎回窓の後)。
# そのため「最初の 1 msg = engagement 成立」を latch として最大 30 s 待ち、
# 成立後に継続性を別窓で測る 2 段構えにする。
# なお ros2 topic hz は本環境で publisher 健在でも受信ゼロになるため使わない
# (echo は同条件で受信できることを実測済)。
if ! timeout 30 ros2 topic echo /cmd_vel_safety --once >/dev/null 2>&1; then
    echo "   FAIL — Layer D did not engage within 30 s. Do NOT drive."
    echo "   (person must be inside the forward ±30°, 1.0-2.0 m band;"
    echo "    check the failsafe log for 'ENGAGED' to see if detection fired)"
    exit 1
fi
echo "   engaged — hold position ~6 more seconds ..."
count=$(timeout 6 ros2 topic echo /cmd_vel_safety --field linear.x 2>/dev/null \
        | grep -c -- '---')
if [ "${count:-0}" -lt 60 ]; then
    echo "   FAIL — only ${count} safety msgs in 6 s (need >= 60 = 20 Hz x 3 s)"
    echo "   Layer D engagement did not hold. Do NOT drive."
    exit 1
fi
echo "   PASS: Layer D engaged and held (${count} msgs / 6 s)"
echo
echo "=== preflight PASS — safe to drive ==="
