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
  # Capture before grep -q to avoid the same SIGPIPE race that bit
  # check_glim_installed (see comment there). nvcc output is ~5 lines so
  # the race rarely fires, but keep the pattern consistent across
  # preflight checks.
  local nvcc_ver
  nvcc_ver="$("${NVCC}" --version)"
  if ! echo "${nvcc_ver}" | grep -q 'release 12.4'; then
    echo "ERROR: ${NVCC} did not report release 12.4." >&2
    echo "${nvcc_ver}" >&2
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
  # Capture ros2 pkg list output to a variable BEFORE grepping so the
  # SIGPIPE race below does not bite us:
  #   `ros2 pkg list | grep -q '^glim_ros$'` matches glim_ros early in the
  #   alphabetically-ordered output (the 'g' band), and grep -q closes
  #   stdin the moment it matches. ros2 still has the rest of the package
  #   list to write, gets SIGPIPE, exits 141. Under `set -o pipefail` the
  #   pipe's exit becomes 141 even though grep itself returned 0. The
  #   `if !` then reads "non-zero = failure" and enters the error branch.
  #   Symptom: this preflight intermittently fails on a healthy install,
  #   which is what tripped the 2026-06-24 verification session.
  # Storing the full output in a variable lets ros2 finish cleanly.
  local pkg_list
  pkg_list=$(ros2 pkg list 2>/dev/null)
  if ! echo "${pkg_list}" | grep -q '^glim_ros$'; then
    # Print enough state to diagnose the common causes: (a) install/glim_ros
    # missing on disk (real broken install) vs (b) AMENT_PREFIX_PATH not
    # extended by the just-sourced setup.bash (transient shell state).
    {
      echo "ERROR: glim_ros not visible to ros2 pkg list."
      echo ""
      echo "Diagnostic:"
      if [[ -d "${REPO_ROOT}/install/glim_ros" ]]; then
        echo "  install/glim_ros/ : EXISTS"
      else
        echo "  install/glim_ros/ : MISSING — run scripts/install_glim.sh"
      fi
      # Same SIGPIPE-avoidance pattern as the main check: store split
      # paths in a variable, grep that. Otherwise a long AMENT_PREFIX_PATH
      # could abort the diagnostic mid-output under `set -e`.
      local ament_paths
      ament_paths="$(echo "${AMENT_PREFIX_PATH:-}" | tr ':' '\n')"
      if echo "${ament_paths}" | grep -q '/glim_ros$'; then
        echo "  AMENT_PREFIX_PATH : contains glim_ros (env looks correct)"
        echo "  ros2 pkg list head:"
        # Use the captured pkg_list (same SIGPIPE-avoidance reason as
        # above); using `ros2 pkg list | head -3` would re-introduce the
        # bug.
        echo "${pkg_list}" | head -3 | sed 's/^/    /'
        echo ""
        echo "  This is unusual — re-source manually in this shell:"
        echo "    source /opt/ros/humble/setup.bash"
        echo "    source ${REPO_ROOT}/install/setup.bash"
      else
        echo "  AMENT_PREFIX_PATH : MISSING glim_ros — install/setup.bash"
        echo "                      did not extend the path. Try opening a"
        echo "                      fresh shell and sourcing manually."
      fi
    } >&2
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
  # config/ and patching topic strings + sensor-side tunables. This keeps
  # the install tree clean, lets the per-run config travel with the run
  # output (reproducibility), and is the standard "custom-sensor" GLIM
  # flow per upstream docs.
  if grep -q '/velodyne_points' "${BAG_DIR}/metadata.yaml"; then
    local local_cfg="${OUT_DIR}/config"
    rm -rf "${local_cfg}"
    cp -r "${share}" "${local_cfg}"
    # Patch topics in config_ros.json with sed (the upstream JSON has
    # // comments, so a JSON parser like jq won't work directly).
    # Auto-detect which IMU topic the bag carries. Bags recorded after
    # Issue #56 (whill_sensors_bringup/imu_sign_corrector) publish
    # /imu/data_rep145 in REP-145 specific-force convention; older bags
    # (or the m5r3_fix_imu_bag.py rewrite) carry /imu/data_raw.
    if grep -q '/imu/data_rep145' "${BAG_DIR}/metadata.yaml"; then
      imu_topic='/imu/data_rep145'
    elif grep -q '/imu/data_raw' "${BAG_DIR}/metadata.yaml"; then
      imu_topic='/imu/data_raw'
    else
      echo "ERROR: bag has neither /imu/data_rep145 nor /imu/data_raw" >&2
      exit 1
    fi
    sed -i "s|^\(\s*\"imu_topic\":\s*\)\"[^\"]*\"|\1\"${imu_topic}\"|" \
      "${local_cfg}/config_ros.json"
    sed -i 's|^\(\s*"points_topic":\s*\)"[^"]*"|\1"/velodyne_points"|' \
      "${local_cfg}/config_ros.json"
    # Patch sensor-side config with our calibrated Velodyne+RT-IMU values.
    # - T_lidar_imu: GLIM convention is p_lidar = T_lidar_imu * p_imu in
    #   TUM format [x, y, z, qx, qy, qz, qw]. Upstream default (Ouster OS0
    #   near-zero translation) makes GLIM assume LiDAR is co-located with
    #   the IMU, which on our rig introduces the "IMU prediction is not
    #   good. Possibly T_lidar_imu is not accurate" warning that filled
    #   the entire run.log of 2026-06-21 run #1.
    #
    # The numbers below are SE3-inverse of M4R-2's measured extrinsic
    # (extrinsic_T = LiDAR origin in IMU frame [0.104136, 0.411548,
    # 0.323704], extrinsic_R = LiDAR->IMU rotation in
    # docs/ja/m3-extrinsics-from-noetic.md) with quaternion computed in
    # TUM (qx,qy,qz,qw) order. Roundtrip error < 1e-6.
    #
    # 2026-07-08 note: direct visual inspection revealed the IMU is
    # actually rotated relative to base_link (IMU +y points forward,
    # +x points right = 90 deg yaw) and tilted ~8 deg (rear-low,
    # front-high). The base_link->imu_link static TF was updated to
    # reflect this. HOWEVER, attempts to update T_lidar_imu here to
    # match (yaw=-90, roll=+8 composed into the LiDAR-IMU relative
    # rotation) caused GLIM to diverge in the first 6 seconds — even
    # in clean state, reproducibly. The most likely explanation is
    # that the imu_sign_corrector (PR #56) REP-145 sign convention was
    # calibrated assuming axis-aligned IMU, and GLIM's noetic-derived
    # T_lidar_imu implicitly assumes the same convention. Changing
    # T_lidar_imu to the physically-correct orientation breaks that
    # internal consistency and the optimizer cannot converge from
    # frame 1. Keep the noetic value until (a) a new bag is recorded
    # under the corrected static TF chain, (b) imu_sign_corrector is
    # revisited, or (c) FAST-LIO SAM / another SLAM less sensitive to
    # this convention is tried.
    # - ring_field: Velodyne ROS2 driver writes laser ID into "ring".
    #   Upstream "" (auto-detect) downgrades preprocessing quality on
    #   Velodyne bags per koide3/glim README "Custom sensor" section.
    python3 - "${local_cfg}/config_sensors.json" <<'PY'
import re, sys
path = sys.argv[1]
with open(path, 'r') as f:
    txt = f.read()
new_tli = (
    '"T_lidar_imu": [\n'
    '      -0.050000,\n'
    '      -0.400000,\n'
    '      -0.350000,\n'
    '      0.017399,\n'
    '      -0.078447,\n'
    '      0.001369,\n'
    '      0.996765\n'
    '    ]'
)
txt, n_tli = re.subn(r'"T_lidar_imu":\s*\[[^\]]*\]', new_tli, txt, count=1)
txt, n_rf = re.subn(r'("ring_field":\s*)"[^"]*"', r'\1"ring"', txt, count=1)
if n_tli != 1 or n_rf != 1:
    sys.stderr.write(
        f"ERROR: config_sensors.json patch failed (T_lidar_imu={n_tli}, ring_field={n_rf}).\n"
        f"  This usually means upstream GLIM changed the JSON shape of T_lidar_imu\n"
        f"  or ring_field. Inspect {path} by hand; the regexes here assume a flat\n"
        f"  float array for T_lidar_imu and a quoted string for ring_field.\n"
    )
    sys.exit(1)
with open(path, 'w') as f:
    f.write(txt)
PY
    GLIM_CONFIG="${local_cfg}/"
    echo "NOTE: bag carries /velodyne_points; using per-run config copy at" >&2
    echo "      ${local_cfg}/ with topics rewritten (${imu_topic} + " >&2
    echo "      /velodyne_points) and sensor config patched (T_lidar_imu" >&2
    echo "      from M4R-2 measured extrinsic, ring_field=ring for VLP-16)." >&2
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

  # Run the ros2 invocation as its own pipe stage so PIPESTATUS[0] reflects
  # GLIM's exit code. Wrapping start/end echoes + ros2 in a brace group
  # would make the brace group's exit be the trailing echo (always 0),
  # which silently overwrites a non-zero GLIM exit in manifest.yaml — and
  # ADR-0003 would then ingest a "successful" run that actually died.
  echo "==> GLIM start ${started_at}" | tee "${OUT_DIR}/run.log"
  set +e
  # auto_quit:=true forces glim_rosbag to dump + exit at end of bag
  # rather than entering rclcpp::spin() to wait for SIGINT. Without it
  # the wrapper hangs (and Ctrl+C mid-optimisation loses traj_lidar.txt
  # — see Issue #63 and the dead 2026-06-24 run that triggered this fix).
  /usr/bin/time -p ros2 run glim_ros glim_rosbag \
    "${BAG_DIR}" \
    --ros-args \
      -p config_path:="${GLIM_CONFIG}" \
      -p dump_path:="${OUT_DIR}/" \
      -p auto_quit:=true 2>&1 | tee -a "${OUT_DIR}/run.log"
  local rc=${PIPESTATUS[0]}
  set -e
  echo "==> GLIM end $(date -u +%Y-%m-%dT%H:%M:%SZ)" | tee -a "${OUT_DIR}/run.log"

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
