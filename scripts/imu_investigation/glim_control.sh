#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# #147: run GLIM (GPU) on one bag with exactly one config change per condition and
# keep each run's config, log and exit code. The input bags are only read, and the
# ROS graph is isolated so this cannot talk to a running robot.
# Usage: glim_control.sh <bag> <base_config_dir> <out_dir> [module.json:section.key=json_value ...]
set -eo pipefail
BAG=$(readlink -f "$1"); BASE=$(readlink -f "$2"); OUT=$3; shift 3
[[ ! -e "$OUT" ]] || { echo "$OUT exists; refusing to overwrite" >&2; exit 2; }
mkdir -p "$OUT"; OUT=$(readlink -f "$OUT")
# -L: saved GLIM configs may be symlinks to the mutable upstream tree; freeze real files.
cp -rL "$BASE" "$OUT/config"
python3 -I - "$OUT/config" "$@" <<'PY'
import json, sys
from pathlib import Path
root = Path(sys.argv[1])
for spec in sys.argv[2:]:
    name, rest = spec.split(':', 1)
    key, val = rest.split('=', 1)
    sec, k = key.split('.', 1)
    p = root / name
    d = json.loads(p.read_text())
    if k not in d[sec]:
        sys.exit(f'{name}: {key} not present; refusing to add a key silently')
    d[sec][k] = json.loads(val)
    p.write_text(json.dumps(d, indent=2))
PY
source /opt/ros/humble/setup.bash
source "$HOME/whill_lab0_ros2/install/setup.bash"
# GLIM_DOMAIN_ID: use_sim_time makes GLIM subscribe /clock, so two runs (or
# another session's replay) on one domain could feed each other's clock.
export ROS_DOMAIN_ID=${GLIM_DOMAIN_ID:-135} ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
# stdin: KeyboardHandler touches the TTY before opening the bag (a TTY run hung, #135).
rc=0
# GLIM_TIMEOUT: 300 s fits the 300 s control segments of #147. The full 07-10
# map bag (2162 s, #153) took 692 s on this GPU in July, so it needs a longer
# limit; the timeout only guards against a hang, it is not a time budget.
timeout --signal=INT --kill-after=10s "${GLIM_TIMEOUT:-300s}" ros2 run glim_ros glim_rosbag "$BAG" \
  --ros-args -p use_sim_time:=true -p config_path:="$OUT/config/" \
  -p dump_path:="$OUT/glim/" -p auto_quit:=true </dev/null >"$OUT/glim.log" 2>&1 || rc=$?
echo "$rc" >"$OUT/exit-code.txt"
grep -q '\[info\] saved' "$OUT/glim.log" || { echo "$OUT: no save completion (rc=$rc)" >&2; exit 1; }
exit "$rc"
