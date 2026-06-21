#!/usr/bin/env bash
# Run GLIM against a recorded rosbag for M5R-3 (Issue #48) comparison.
# Wraps `ros2 run glim_ros glim_rosbag` with time + VRAM measurement so the
# numbers land in a fixed file layout that ADR-0003 can consume verbatim.
#
# Why a wrapper instead of a one-liner in the protocol doc:
#   * Wall time and peak VRAM are part of the comparison axes (see
#     docs/ja/plans/2026-06-21-m5r-execution.md §6 acceptance B1/B4 and
#     §M5R-3). Doing them by hand each run loses reproducibility — one
#     evaluator's `nvidia-smi` poll cadence differs from another's. The
#     wrapper fixes the cadence at 0.5 s and dumps a single peak number.
#   * GLIM's CLI accepts `config_path` and `dump_path` as ROS params, not
#     positionals. A wrapper makes the path conventions visible and
#     ensures the output directory is created with the manifest.yaml
#     before the run starts, so a crash mid-run still leaves enough
#     trace to debug.
#   * The Velodyne-vs-Ouster config switch (see select_glim_config below)
#     is fragile to get right by hand. Centralising it here keeps the
#     comparison symmetric: both GLIM runs apply the same config-select
#     logic regardless of which bag the evaluator points at.
#
# Idempotent: re-running with the same <out-dir> aborts unless --force is
# passed. The deliberate "abort by default" is so an accidental re-run does
# not silently overwrite a manifest that ADR-0003 already references.
#
# Usage:
#   ./scripts/m5r3_run_glim.sh <bag-dir> <out-dir> [--force]
#
# Prereqs (verified by this script):
#   * M5R-1 (Issue #45) install completed — `ros2 pkg list | grep glim_ros`
#   * CUDA 12.4 active — nvcc reports 12.4
#   * <bag-dir>/metadata.yaml exists (rosbag2 standard layout)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NVCC="/usr/local/cuda-12.4/bin/nvcc"

# --- argument parsing --------------------------------------------------------

usage() {
  cat >&2 <<EOF
Usage: ${0##*/} <bag-dir> <out-dir> [--force]

  <bag-dir>  rosbag2 directory (contains metadata.yaml + *.db3 or *.mcap)
  <out-dir>  directory to write GLIM outputs into (created if missing)
  --force    overwrite existing <out-dir> contents

Companion document: docs/ja/m5r3-comparison-protocol.md
EOF
  exit 2
}

if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage
fi

BAG_DIR="$(cd "$1" && pwd)"
OUT_DIR="$2"
FORCE=0
if [[ $# -eq 3 ]]; then
  if [[ "$3" != "--force" ]]; then
    usage
  fi
  FORCE=1
fi

# Resolve OUT_DIR to absolute so the manifest does not contain "./" relative
# paths that break when ADR-0003 is read from a different cwd.
mkdir -p "${OUT_DIR}"
OUT_DIR="$(cd "${OUT_DIR}" && pwd)"

# --- preflight ---------------------------------------------------------------

check_bag() {
  # rosbag2 always emits metadata.yaml in the bag directory. Its absence
  # almost always means the evaluator pointed at the parent directory by
  # mistake; failing early here saves a confusing GLIM crash later.
  if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
    echo "ERROR: ${BAG_DIR}/metadata.yaml not found. Is this a rosbag2 directory?" >&2
    exit 1
  fi
}

check_cuda_124() {
  # Mirror install_glim.sh::require_cuda_124. The 12.4 pin is the whole
  # reason install_glim.sh source-builds gtsam_points; if a different nvcc
  # is on PATH now, the GLIM binary's CUDA runtime can ABI-mismatch the
  # gtsam_points runtime that was linked at install time.
  if [[ ! -x "${NVCC}" ]]; then
    echo "ERROR: ${NVCC} not found. Re-run scripts/install_cuda.sh first." >&2
    exit 1
  fi
  if ! "${NVCC}" --version | grep -q 'release 12.4'; then
    echo "ERROR: ${NVCC} did not report release 12.4." >&2
    "${NVCC}" --version >&2 || true
    exit 1
  fi
}

check_glim_installed() {
  # The setup.bash trace variables (AMENT_TRACE_SETUP_FILES, COLCON_TRACE)
  # are unset in clean shells; sourcing under `set -u` aborts. Mirror the
  # `set +u; source; set -u` pattern from install_glim.sh::verify.
  if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
    echo "ERROR: ${REPO_ROOT}/install/setup.bash not found. Run M5R-1 install first." >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/install/setup.bash"
  set -u
  if ! ros2 pkg list 2>/dev/null | grep -q '^glim_ros$'; then
    echo "ERROR: glim_ros not visible to ros2 pkg list. Re-run scripts/install_glim.sh." >&2
    exit 1
  fi
}

check_out_dir_empty() {
  # The known GLIM outputs we look for as "already-run evidence":
  #   * traj_lidar.txt — written at end of run, hard signal of completed run
  #   * dump.pcd / map.pcd — the static cloud the comparison needs
  #   * manifest.yaml — our own marker from a previous wrapper invocation
  # Any of these existing means the previous wrapper output is still here
  # and a re-run would silently overwrite. Abort unless --force.
  local existing=()
  for marker in traj_lidar.txt dump.pcd map.pcd manifest.yaml; do
    if [[ -e "${OUT_DIR}/${marker}" ]]; then
      existing+=("${marker}")
    fi
  done
  if [[ ${#existing[@]} -gt 0 && "${FORCE}" -eq 0 ]]; then
    echo "ERROR: ${OUT_DIR} already contains: ${existing[*]}" >&2
    echo "       Re-run with --force to overwrite, or pick a fresh out-dir." >&2
    exit 1
  fi
}

# --- config selection (Velodyne vs Ouster sample) ----------------------------

select_glim_config() {
  # GLIM ships per-LiDAR config bundles. The Ouster OS1-128 sample bag
  # (the only one M5R-1 was able to smoke-test against) uses /points
  # whereas this repo's M4-R bringup publishes /velodyne_points. Picking
  # the right config matters because GLIM keys some preprocessing off the
  # ring layout — running the Ouster config on a Velodyne bag silently
  # degrades feature extraction in ways that would skew the M5R-3
  # comparison. The detection is done by topic name rather than file
  # contents so we avoid touching the bag's binary readers here.
  # The configs ship under the `glim` package, not `glim_ros` — the latter
  # is just the ROS 2 wrapper. M5R-1's install_glim.sh next-steps hint
  # incorrectly pointed at glim_ros/share/glim_ros/config (that path does
  # not exist); the actual location is glim/share/glim/config. We follow
  # the install tree here, and protocol doc + install_glim.sh are
  # corrected separately.
  local share
  share="$(ros2 pkg prefix glim)/share/glim/config"
  if [[ ! -d "${share}" ]]; then
    echo "ERROR: glim config dir not found under ${share}." >&2
    exit 1
  fi

  # Upstream ships a single flat config/ directory keyed off config.json,
  # which references config_sensors.json / config_preprocess.json / etc by
  # relative name. There is no per-LiDAR subdir — sensor and topic
  # selection is done by editing config_ros.json (topic names) and
  # config_sensors.json (T_lidar_imu, ring_field, ...). The upstream
  # defaults are Ouster topics (/os_cloud_node/imu, /os_cloud_node/points)
  # and an Ouster-tuned T_lidar_imu, neither of which matches our
  # Velodyne bag. The first real Phase B run on 2026-06-21 confirmed
  # this: the run started, subscribed to /os_cloud_node/* (which the bag
  # does not publish), got no data, and exited with SIGPIPE / rc=141.
  #
  # We work around by copying the upstream config dir into <OUT_DIR>/
  # config/ and patching just the topic strings. This keeps the install
  # tree clean, lets the per-run config travel with the run output
  # (reproducibility), and is the standard "custom-sensor" GLIM flow per
  # upstream docs. Sensor-side tuning (T_lidar_imu, ring_field) is left
  # to the evaluator on the assumption that bad output prompts that
  # edit; capturing the override in the per-run config means whatever
  # the evaluator settles on is recorded next to the trajectory.
  if grep -q '/velodyne_points' "${BAG_DIR}/metadata.yaml"; then
    local local_cfg="${OUT_DIR}/config"
    rm -rf "${local_cfg}"
    cp -r "${share}" "${local_cfg}"
    # Patch topics in config_ros.json with sed (the upstream JSON has
    # // comments, so a JSON parser like jq won't work directly).
    sed -i 's|^\(\s*"imu_topic":\s*\)"[^"]*"|\1"/imu/data_raw"|' \
      "${local_cfg}/config_ros.json"
    sed -i 's|^\(\s*"points_topic":\s*\)"[^"]*"|\1"/velodyne_points"|' \
      "${local_cfg}/config_ros.json"
    GLIM_CONFIG="${local_cfg}/"
    echo "NOTE: bag carries /velodyne_points; using per-run config copy at" >&2
    echo "      ${local_cfg}/ with topics rewritten to /velodyne_points + /imu/data_raw." >&2
    echo "      If trajectory still looks broken (missing points / preprocess" >&2
    echo "      warnings), the next thing to try is editing" >&2
    echo "      ${local_cfg}/config_sensors.json (ring_field=ring for VLP-16," >&2
    echo "      T_lidar_imu from M4R-2's measured extrinsic). Record any edit" >&2
    echo "      in the ADR-0003 Alternatives row." >&2
  else
    GLIM_CONFIG="${share}/"
  fi
}

# --- VRAM sampling -----------------------------------------------------------

start_vram_logger() {
  # 0.5 s cadence is a compromise: dense enough to catch the keyframe-emit
  # VRAM spike (GLIM allocates per-keyframe iVox tiles in bursts), sparse
  # enough that the logger itself does not steal noticeable PCIe bandwidth.
  # If multiple GPUs are present, --id 0 keeps the wrapper deterministic;
  # the M5R-3 host has one discrete GPU so this is fine for now.
  : > "${OUT_DIR}/vram.log"
  (
    while true; do
      nvidia-smi --id=0 --query-gpu=memory.used \
        --format=csv,noheader,nounits 2>/dev/null \
        | awk -v now="$(date +%s.%N)" '{print now, $1}' >> "${OUT_DIR}/vram.log" || true
      sleep 0.5
    done
  ) &
  VRAM_PID=$!
}

stop_vram_logger() {
  # Idempotent: clear VRAM_PID before kill/wait so a second invocation
  # (trap firing after the success path's explicit call) becomes a no-op
  # instead of hitting an already-reaped pid.
  local pid="${VRAM_PID:-}"
  VRAM_PID=""
  if [[ -n "${pid}" ]]; then
    kill "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

peak_vram_mib() {
  # vram.log lines are "epoch.fractional mib". Empty log (e.g. glim_rosbag
  # crashed before the first sample) returns 0 so the manifest field is
  # always numeric.
  if [[ ! -s "${OUT_DIR}/vram.log" ]]; then
    echo 0
    return
  fi
  awk 'BEGIN{m=0} {if ($2+0 > m) m=$2+0} END{print m}' "${OUT_DIR}/vram.log"
}

# --- main run ----------------------------------------------------------------

run_glim() {
  local started_at
  started_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  local started_epoch
  started_epoch="$(date +%s)"

  # Write the manifest BEFORE the run so a crash mid-run still leaves a
  # discoverable record of what was attempted. The fields here are the
  # ones ADR-0003 will quote verbatim (see docs/decisions/0003-...md
  # Context node).
  local git_commit
  git_commit="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
  local glim_prefix
  glim_prefix="$(ros2 pkg prefix glim_ros)"
  local cuda_ver
  cuda_ver="$("${NVCC}" --version | grep -oE 'release [0-9.]+' | awk '{print $2}')"

  cat > "${OUT_DIR}/manifest.yaml" <<EOF
# Generated by scripts/m5r3_run_glim.sh — do not hand-edit fields above the
# "results" stanza. Add notes under "notes:" at the bottom for ADR-0003.
slam_method: glim
bag: ${BAG_DIR}
bag_metadata: ${BAG_DIR}/metadata.yaml
out_dir: ${OUT_DIR}
glim_install_prefix: ${glim_prefix}
glim_config_path: ${GLIM_CONFIG}
cuda_version: ${cuda_ver}
git_commit: ${git_commit}
started_at: ${started_at}
EOF

  # `time -p` writes posix-format real/user/sys to stderr, so we redirect
  # the entire pipeline's stderr to run.log and then post-process. Using
  # /usr/bin/time would give us %M (max RSS) too, but that is host RAM not
  # VRAM — the VRAM logger captures GPU memory separately.
  # The bag dir is the first positional argument to glim_rosbag; M5R-1's
  # smoke test against the Ouster OS1-128 sample bag used that form
  # (config_path/dump_path passed as `--ros-args -p` only).
  start_vram_logger
  trap 'stop_vram_logger' EXIT

  set +e
  {
    echo "==> GLIM start ${started_at}"
    /usr/bin/time -p ros2 run glim_ros glim_rosbag \
      "${BAG_DIR}" \
      --ros-args \
        -p config_path:="${GLIM_CONFIG}" \
        -p dump_path:="${OUT_DIR}/"
    echo "==> GLIM end $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  } 2>&1 | tee "${OUT_DIR}/run.log"
  local rc=${PIPESTATUS[0]}
  set -e

  stop_vram_logger
  trap - EXIT

  local ended_at
  ended_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  local duration=$(( $(date +%s) - started_epoch ))
  local peak
  peak="$(peak_vram_mib)"

  cat >> "${OUT_DIR}/manifest.yaml" <<EOF
ended_at: ${ended_at}
duration_sec: ${duration}
max_vram_mib: ${peak}
exit_code: ${rc}

# results: filled by ADR-0003 author after running m5r3_loop_error.py and
# CloudCompare wall-point picking. Leave the keys here so the schema is
# obvious to the next reader.
results:
  loop_error_trajectory_m: TBD       # from m5r3_loop_error.py end-to-start
  loop_error_wall_3pt_m: TBD         # B1 official, from CloudCompare
  notes: |
    TBD: Iridescence visual cues (loop-closure trigger frame index, key
    frame density, manual relocalization need, etc.).
EOF

  if [[ "${rc}" -ne 0 ]]; then
    echo "WARNING: glim_rosbag exited with ${rc}. See ${OUT_DIR}/run.log." >&2
  fi
}

# --- next-steps hint ---------------------------------------------------------

print_next_steps() {
  cat >&2 <<EOF

GLIM run complete. Outputs under: ${OUT_DIR}

Next:
  python3 ${REPO_ROOT}/scripts/m5r3_loop_error.py ${OUT_DIR}/traj_lidar.txt

For the formal B1 criterion (start/end wall 3-point mean), open the
generated PCD (${OUT_DIR}/dump.pcd or map.pcd, depending on glim_ros
release) in CloudCompare and follow docs/ja/m5r3-comparison-protocol.md
§"ループ誤差計測".

Transcribe the manifest.yaml + traj_lidar.txt loop-error result into the
Alternatives table of docs/ja/decisions/0003-mapping-slam-choice.md.
EOF
}

main() {
  echo "[1/5] preflight: bag layout"
  check_bag
  echo "[2/5] preflight: CUDA 12.4"
  check_cuda_124
  echo "[3/5] preflight: glim_ros installed"
  check_glim_installed
  echo "[4/5] preflight: out-dir state"
  check_out_dir_empty
  select_glim_config
  echo "      using GLIM config: ${GLIM_CONFIG}"
  echo "[5/5] running glim_rosbag (VRAM sampling at 0.5 s)"
  run_glim
  print_next_steps
}

main "$@"
