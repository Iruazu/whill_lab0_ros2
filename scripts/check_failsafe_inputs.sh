#!/usr/bin/env bash
# failsafe が監視する入力が届いているかを、failsafe と同じ QoS で確かめる（#168）。
# 全部を確かめて、届かなかった入力をすべて表示する。1つでも届かなければ非0で終わる。
# SPDX-License-Identifier: BSD-3-Clause
#
# /rosout の DEAD INPUT は使わない。起動10秒後に1回しか出ないので、RViz で初期位置を
# 与える前に Layer B が必ず載る。今の入力の状態も表さない。
# ros2 topic hz も使わない。BEST_EFFORT で届いていても受信ゼロになることがある。
set -u

TIMEOUT_S=${PREFLIGHT_INPUT_TIMEOUT_S:-8}
# トピック、型、Layer、QoS。型と QoS は failsafe_node.py の create_subscription と揃える。
# 揃えないと、過去の事故のような不一致を見逃す（型は 2026-07-14、QoS は 2026-07-16）。
# 型を省くと、echo は発見の前に即座に失敗し、時間いっぱい待たない。
INPUTS=(
    '/alignment_status|diagnostic_msgs/msg/DiagnosticArray|B|--qos-reliability reliable --qos-durability volatile'
    '/pcl_pose|geometry_msgs/msg/PoseWithCovarianceStamped|B|--qos-reliability reliable --qos-durability volatile'
    '/velodyne_points_no_ground|sensor_msgs/msg/PointCloud2|C|--qos-profile sensor_data'
    '/scan|sensor_msgs/msg/LaserScan|D|--qos-profile sensor_data'
)

missing=0
for input in "${INPUTS[@]}"; do
    IFS='|' read -r topic type layer qos <<<"$input"
    # shellcheck disable=SC2086
    if timeout "$TIMEOUT_S" ros2 topic echo --once --no-arr $qos "$topic" "$type" >/dev/null 2>&1; then
        printf '   %-28s Layer %s  ok\n' "$topic" "$layer"
    else
        printf '   %-28s Layer %s  %s 秒以内に届かない\n' "$topic" "$layer" "$TIMEOUT_S"
        missing=1
    fi
done
exit "$missing"
