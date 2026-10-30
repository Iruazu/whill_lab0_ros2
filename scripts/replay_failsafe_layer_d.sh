#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Replay a bag's /scan through two failsafe_node instances (Layer D sector =
# demo default, rectangle = avoidance:=true params) and count Layer D
# engagements from /failsafe/state.
#
# Why two nodes in one replay: both see the identical message timing, so the
# sector/rectangle difference is purely geometric. Their outputs are
# remapped (/fs_<shape>/...) so nothing reaches /cmd_vel_safety, and only
# /scan (+ /clock) is played: no cmd_vel topic from the bag is republished.
#
# Safety: run on a private ROS_DOMAIN_ID with no chair driver in it. The
# script refuses to start if any node is already visible in the domain.
#
# Usage: scripts/replay_failsafe_layer_d.sh <bag_dir> [rate] [out_dir]
set -euo pipefail

BAG=${1:?usage: $0 <bag_dir> [rate] [out_dir]}
RATE=${2:-2.0}
OUT=${3:-$(mktemp -d /tmp/fs_replay_XXXX)}
mkdir -p "$OUT"
: "${ROS_DOMAIN_ID:?set a private ROS_DOMAIN_ID (no chair driver in it)}"

if [ -n "$(ros2 node list 2>/dev/null)" ]; then
  echo "domain ${ROS_DOMAIN_ID} is not empty:" >&2
  ros2 node list >&2
  exit 1
fi

pids=()
# `ros2 run` does not forward SIGTERM to the node it spawns, so also kill the
# node processes by their unique __node remap (found left over on 10-30).
cleanup() {
  kill "${pids[@]}" 2>/dev/null || true
  pkill -INT -f -- '__node:=fs_(sector|rectangle)' 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

# Same params safety_launch.py passes for avoidance:=false / true (Layer D
# part only; extra_perception_topics are omitted because the bag has none
# and they do not affect Layer D).
for shape in sector rectangle; do
  extra=()
  if [ "$shape" = rectangle ]; then
    extra=(-p layer_d_shape:=rectangle -p layer_d_rect_half_width_m:=0.5)
  fi
  ros2 run whill_safety failsafe_node --ros-args \
    -r __node:=fs_${shape} \
    -r /failsafe/state:=/fs_${shape}/state \
    -r /cmd_vel_safety:=/fs_${shape}/cmd_vel_safety \
    -p use_sim_time:=true "${extra[@]}" >"$OUT/fs_${shape}.log" 2>&1 &
  pids+=($!)
done

# Count rising edges of D:forward_blocked on each state topic.
python3 - "$OUT" <<'EOF' &
import sys, json, signal, rclpy
from rclpy.node import Node
from std_msgs.msg import String
out = sys.argv[1]
rclpy.init()
n = Node('fs_replay_counter', parameter_overrides=[
    rclpy.parameter.Parameter('use_sim_time', value=True)])
res = {s: {'engagements': 0, 'times': [], 'prev': False} for s in ('sector', 'rectangle')}
def cb(shape):
    def f(msg):
        d = 'D:forward_blocked' in msg.data
        r = res[shape]
        if d and not r['prev']:
            r['engagements'] += 1
            r['times'].append(round(n.get_clock().now().nanoseconds * 1e-9, 2))
        r['prev'] = d
    return f
for s in res:
    n.create_subscription(String, f'/fs_{s}/state', cb(s), 50)
def dump(*_):
    for r in res.values():
        r.pop('prev', None)
    with open(f'{out}/counts.json', 'w') as fh:
        json.dump(res, fh, indent=1)
    raise SystemExit(0)
signal.signal(signal.SIGTERM, dump)
signal.signal(signal.SIGINT, dump)
rclpy.spin(n)
EOF
pids+=($!)
counter=$!

sleep 3
ros2 bag play "$BAG" --clock 200 --rate "$RATE" --topics /scan >"$OUT/play.log" 2>&1
sleep 2
kill -TERM "$counter"; wait "$counter" 2>/dev/null || true
cat "$OUT/counts.json"
grep -h "ready\|ENGAGED" "$OUT"/fs_*.log | head -4
echo "logs: $OUT"
