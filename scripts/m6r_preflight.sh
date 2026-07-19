#!/usr/bin/env bash
# m6r_preflight.sh — blocking pre-drive gate for M6-R integration demo.
#
# 2026-07-16 incident: Layer D failed to arm because /scan subscription
# defaulted to RELIABLE while p2ls publishes BEST_EFFORT. The chair drove
# into a person during V2 verification. Non-blocking pre-drive checks
# (rely on operator to eyeball a log line) let this slip through.
#
# This script FAILS LOUD (exit 1) if failsafe_node is not up, has DEAD
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

# ---- 3. Wait past dead-input watchdog window -----------------------
# failsafe_node's STARTUP_DEAD_INPUT_TIMEOUT_S = 10 s. If the
# subscriptions did not arm within that budget, an ERROR line shows
# up on /rosout with the substring "DEAD INPUT".
echo "3. dead-input watchdog: waiting 12 s to catch any ERROR ..."
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

# ---- 4. Live-fire Layer D test -------------------------------------
echo "4. Layer D live fire test."
echo "   >>> NOW: have a person stand ~1-1.5 m directly ahead of the chair"
echo "   >>> and STAY there until this check prints PASS/FAIL (~13 s)."
# 立ち位置につく猶予。/cmd_vel_safety は遮断中しか publish されないため、
# 計測窓が始まる前に人が検出帯 (前方 ±30°, 1.0-2.0 m) に入っている必要が
# ある。2026-07-19 field でここのタイミング不一致による偽陽性 FAIL を実測。
sleep 3
# ros2 topic hz は本環境 (cyclonedds runtime.xml + 常駐 launch 構成) で
# publisher が健在でも受信ゼロになる (2026-07-19 field で確定。echo は同
# 条件で受信できる) ため、echo のメッセージ数カウントで判定する。
# 合格条件は「窓 10 s 内に 20 Hz × 3 s 連続遮断ぶん (45 msg) 以上」。
# レートの平均値判定だと立ち位置調整に食われた時間で薄まり不安定なため、
# 絶対数で見る。
count=$(timeout 10 ros2 topic echo /cmd_vel_safety --field linear.x 2>/dev/null \
        | grep -c -- '---')
if [ "${count:-0}" -eq 0 ]; then
    echo "   FAIL — /cmd_vel_safety not publishing at all"
    echo "   Layer D did not engage. Do NOT drive."
    echo "   (person must be inside the 1.0-2.0 m forward band during the window;"
    echo "    check the failsafe log for 'ENGAGED' to confirm detection itself)"
    exit 1
fi
if [ "${count}" -lt 45 ]; then
    echo "   FAIL — only ${count} safety msgs in 10 s (need >= 45 = 20 Hz x 3 s)"
    echo "   Layer D partial engagement. Do NOT drive."
    exit 1
fi
echo "   PASS: ${count} safety msgs in 10 s (Layer D engaged and held)"
echo
echo "=== preflight PASS — safe to drive ==="
