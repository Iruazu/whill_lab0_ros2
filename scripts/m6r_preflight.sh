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
echo "4. Layer D live fire — obstruct the chair front now ..."
echo "   (hold a hand ~1.5 m ahead of the chair for the next 5 s)"
sleep 5
rate=$(timeout 3 ros2 topic hz /cmd_vel_safety 2>&1 \
        | grep -oP 'average rate: \K[0-9.]+' | tail -1)
if [ -z "${rate:-}" ]; then
    echo "   FAIL — /cmd_vel_safety not publishing at all"
    echo "   Layer D did not engage. Do NOT drive."
    exit 1
fi
if [ "$(awk -v r="$rate" 'BEGIN{print (r < 15)}')" = "1" ]; then
    echo "   FAIL — /cmd_vel_safety at ${rate} Hz (expected >= 15 Hz)"
    echo "   Layer D partial engagement. Do NOT drive."
    exit 1
fi
echo "   PASS: /cmd_vel_safety at ${rate} Hz"
echo
echo "=== preflight PASS — safe to drive ==="
