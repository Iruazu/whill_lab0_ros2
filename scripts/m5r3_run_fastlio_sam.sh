#!/usr/bin/env bash
# Run FAST-LIO SAM against a recorded rosbag for M5R-3 (Issue #48) comparison.
# Companion to scripts/m5r3_run_glim.sh — same wrapper shape, same manifest
# schema, so the two runs land in ADR-0003 as a directly comparable pair.
#
# Why the shape is matched with m5r3_run_glim.sh: the M5R-3 decision rests on
# numbers from both SLAMs on the *same* bag (docs/ja/plans/2026-06-21-m5r-
# execution.md §6 B4, ADR-0003 Context). If the two wrappers measure or
# record different things, the comparison is invalidated. Keep the schema
# in lockstep — when m5r3_run_glim.sh adds a field, mirror it here.
#
# Two areas where this diverges from the GLIM wrapper, and why:
#   1. GTSAM coexistence: M5R-1 installed GTSAM 4.3a0 under /usr/local/lib;
#      M5R-2's clone helper installed GTSAM 4.1.1 via PPA under /usr/lib.
#      `ldconfig -p` shows which one the dynamic linker reaches first.
#      We dump that table to gtsam_env.log so ADR-0003 can record the
#      coexistence state at run time (the linker resolution can change
#      between hosts and between re-runs after apt activity).
#   2. CPU vs GPU memory: FAST-LIO SAM upstream does not declare GPU
#      compute usage, but its CMakeLists pulls in some optional CUDA
#      paths (gtsam_unstable). We log both nvidia-smi VRAM and
#      /proc/<pid>/status VmRSS so neither resource ceiling is missed.
#
# What this script does NOT do:
#   * It does not run `colcon build`. The upstream README still lists
#     "Full ROS2 adaptation" as TODO, and the build is the M5R-3
#     evaluator's open question — see docs/ja/m5r-fastlio-sam-eval.md.
#     This wrapper assumes the build is already done and refuses to run
#     otherwise. The reason: silently building from a wrapper would hide
#     build errors that ADR-0003 wants to record verbatim.
#
# Idempotent: re-running with the same <out-dir> aborts unless --force.
#
# Usage:
#   ./scripts/m5r3_run_fastlio_sam.sh <bag-dir> <out-dir> [--force]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY="${REPO_ROOT}/src/third_party"
FASTLIO_SAM_DIR="${THIRD_PARTY}/FAST_LIO_SAM"

# Upstream package name verified by scripts/clone_fastlio_sam_for_eval.sh
# at clone time. If a future upstream rename changes this, both that
# script and this one need updating in lockstep.
PKG_NAME="fast_lio_sam"

# --- argument parsing --------------------------------------------------------

usage() {
  cat >&2 <<EOF
Usage: ${0##*/} <bag-dir> <out-dir> [--force]

  <bag-dir>  rosbag2 directory (contains metadata.yaml + *.db3 or *.mcap)
  <out-dir>  directory to write outputs into (created if missing)
  --force    overwrite existing <out-dir> contents

Prereqs: run scripts/clone_fastlio_sam_for_eval.sh first, then
\`colcon build --packages-up-to ${PKG_NAME} --symlink-install\` from the
repo root. See docs/ja/m5r-fastlio-sam-eval.md §2.
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

mkdir -p "${OUT_DIR}"
OUT_DIR="$(cd "${OUT_DIR}" && pwd)"

# --- preflight ---------------------------------------------------------------

check_bag() {
  if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
    echo "ERROR: ${BAG_DIR}/metadata.yaml not found. Is this a rosbag2 directory?" >&2
    exit 1
  fi
}

check_upstream_cloned() {
  # clone_fastlio_sam_for_eval.sh is the only sanctioned path into the
  # tree; anything else risks landing the upstream without the license
  # caveat being acknowledged (see docs/ja/m5r-fastlio-sam-eval.md).
  # Failing here with an explicit pointer is friendlier than letting the
  # `ros2 pkg list` check below fail with a vague "package not found".
  if [[ ! -d "${FASTLIO_SAM_DIR}" ]]; then
    cat >&2 <<EOF
ERROR: ${FASTLIO_SAM_DIR} not found.

The FAST-LIO SAM upstream has not been cloned. Run:

  FASTLIO_SAM_LICENSE_ACK=yes ./scripts/clone_fastlio_sam_for_eval.sh

See docs/ja/m5r-fastlio-sam-eval.md for the license caveat that the
env-var gate acknowledges.
EOF
    exit 1
  fi
}

check_built() {
  # Mirror m5r3_run_glim.sh::check_glim_installed for the setup.bash
  # source guard. The `set +u; source; set -u` pattern is required
  # because ROS 2 setup.bash references AMENT_TRACE_SETUP_FILES /
  # COLCON_TRACE which are not defined in clean shells.
  if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
    echo "ERROR: ${REPO_ROOT}/install/setup.bash not found. Build the workspace first." >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/install/setup.bash"
  set -u

  if ! ros2 pkg list 2>/dev/null | grep -q "^${PKG_NAME}$"; then
    cat >&2 <<EOF
ERROR: ros2 pkg list does not show '${PKG_NAME}'.

This usually means one of:
  (a) The colcon build has not been run since the clone. Try:
        cd ${REPO_ROOT}
        colcon build --packages-up-to ${PKG_NAME} --symlink-install
  (b) The upstream ROS 2 adaptation is incomplete and the build
      failed. The upstream README marks "Full ROS2 adaptation" as TODO,
      so this is the documented failure mode.

If (b), capture the colcon error log into
docs/m5r-bench-data/<run>/fastlio-sam-out/build-failure.log and write
the symptom into ADR-0003 Alternatives ("FAST-LIO SAM rejected because
the upstream ROS 2 adaptation does not build at <commit>"). The license
restriction in docs/ja/m5r-fastlio-sam-eval.md forbids us from patching
the upstream tree directly.

Exit code 2 is used so the comparison protocol can distinguish "not
built" (exit 2) from "build OK but run failed" (other non-zero).
EOF
    exit 2
  fi
}

check_out_dir_empty() {
  # Markers mirror m5r3_run_glim.sh. traj.txt is the upstream's
  # trajectory output filename (per upstream README); if upstream renames
  # it we want a manifest.yaml from a previous wrapper invocation to
  # still serve as "do not overwrite" evidence.
  local existing=()
  for marker in traj.txt traj_lidar.txt map.pcd manifest.yaml; do
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

# --- GTSAM coexistence inspection -------------------------------------------

dump_gtsam_env() {
  # The two GTSAMs in play and where they come from:
  #   * /usr/local/lib/libgtsam.so.4.3a0 — installed by install_glim.sh
  #     (M5R-1) for GLIM.
  #   * /usr/lib/x86_64-linux-gnu/libgtsam.so.4.1.1 — installed by
  #     clone_fastlio_sam_for_eval.sh (M5R-2) via the borglab PPA for
  #     FAST-LIO SAM.
  #
  # `ldconfig -p` shows the loader's view, which is what actually decides
  # which library a running process binds. If both versions are listed
  # and FAST-LIO SAM was linked against 4.1, behaviour is undefined when
  # both are reachable — usually a SONAME resolution to 4.3 followed by
  # a symbol mismatch crash. Dumping the table to gtsam_env.log lets
  # ADR-0003 Alternatives record the exact state under which numbers
  # were captured.
  ldconfig -p 2>/dev/null | grep -E 'libgtsam(_unstable)?\.so' \
    > "${OUT_DIR}/gtsam_env.log" || true

  local seen_43=0
  local seen_41=0
  if grep -q '\.so\.4\.3' "${OUT_DIR}/gtsam_env.log" 2>/dev/null; then
    seen_43=1
  fi
  if grep -q '\.so\.4\.1' "${OUT_DIR}/gtsam_env.log" 2>/dev/null; then
    seen_41=1
  fi

  if [[ "${seen_43}" -eq 1 && "${seen_41}" -eq 1 ]]; then
    cat >&2 <<EOF
WARNING: both GTSAM 4.3 (GLIM) and GTSAM 4.1 (FAST-LIO SAM) are
         visible to ldconfig. If FAST-LIO SAM was linked against 4.1
         it may end up dynamically resolving to 4.3 with undefined
         results.

         If this run produces obviously wrong trajectories or
         segfaults early, isolate by re-running this script in a
         shell where /usr/local/lib does not precede /usr/lib:

           LD_LIBRARY_PATH=/usr/lib:\$LD_LIBRARY_PATH \\
             ./scripts/m5r3_run_fastlio_sam.sh <bag> <out>

         Either outcome (works with default linker, needs the override)
         is a fact ADR-0003 should record in its Alternatives section.
EOF
  fi
}

# --- resource sampling -------------------------------------------------------

start_vram_logger() {
  # FAST-LIO SAM may or may not use the GPU. We log VRAM unconditionally
  # so the comparison row reports both numbers; a flat-zero VRAM log is
  # itself a finding (i.e. "FAST-LIO SAM is CPU-only on this host").
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

start_rss_logger() {
  # FAST-LIO SAM is heavier on host RAM than GPU. We log VmRSS of the
  # spawned ros2 process tree at 1 s cadence. Using pgrep against the
  # bag path is fragile if the same bag is replayed in parallel, but for
  # the single-run wrapper case it is good enough; the log timestamp
  # disambiguates if the evaluator overlaps two runs.
  : > "${OUT_DIR}/rss.log"
  (
    while true; do
      # Sum VmRSS across any process whose cmdline mentions the bag dir
      # so we capture both the launch process and the node processes.
      # /proc/*/status is read-only; failures (pid disappeared mid-read)
      # are swallowed so the logger never aborts the parent script.
      # `local` is invalid in a subshell. The outer `( ... ) &` block is a
      # subshell, not a function body, so plain assignment is the only
      # form that works without tripping `set -u` and killing the logger
      # on the first iteration (see #48 code-review MF-1).
      total=0
      for pid in $(pgrep -f "${BAG_DIR}" 2>/dev/null); do
        if [[ -r "/proc/${pid}/status" ]]; then
          rss="$(awk '/^VmRSS:/ {print $2}' "/proc/${pid}/status" 2>/dev/null || true)"
          if [[ -n "${rss}" ]]; then
            total=$(( total + rss ))
          fi
        fi
      done
      echo "$(date +%s.%N) ${total}" >> "${OUT_DIR}/rss.log"
      sleep 1
    done
  ) &
  RSS_PID=$!
}

stop_loggers() {
  # Idempotent: clear the PID variables before kill/wait so a second
  # invocation (e.g. trap firing after an explicit call on the success
  # path) becomes a no-op instead of hitting an already-reaped pid.
  local vp="${VRAM_PID:-}"
  local rp="${RSS_PID:-}"
  VRAM_PID=""
  RSS_PID=""
  for pid in "${vp}" "${rp}"; do
    if [[ -n "${pid}" ]]; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}

peak_vram_mib() {
  if [[ ! -s "${OUT_DIR}/vram.log" ]]; then
    echo 0
    return
  fi
  awk 'BEGIN{m=0} {if ($2+0 > m) m=$2+0} END{print m}' "${OUT_DIR}/vram.log"
}

peak_rss_kib() {
  if [[ ! -s "${OUT_DIR}/rss.log" ]]; then
    echo 0
    return
  fi
  awk 'BEGIN{m=0} {if ($2+0 > m) m=$2+0} END{print m}' "${OUT_DIR}/rss.log"
}

# --- main run ----------------------------------------------------------------

run_fastlio_sam() {
  local started_at
  started_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  local started_epoch
  started_epoch="$(date +%s)"

  local git_commit
  git_commit="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
  local upstream_commit
  upstream_commit="$(git -C "${FASTLIO_SAM_DIR}" rev-parse HEAD 2>/dev/null || echo unknown)"
  local pkg_prefix
  pkg_prefix="$(ros2 pkg prefix "${PKG_NAME}")"

  cat > "${OUT_DIR}/manifest.yaml" <<EOF
# Generated by scripts/m5r3_run_fastlio_sam.sh — do not hand-edit fields
# above the "results" stanza. Add notes under "notes:" at the bottom for
# ADR-0003.
slam_method: fast_lio_sam
bag: ${BAG_DIR}
bag_metadata: ${BAG_DIR}/metadata.yaml
out_dir: ${OUT_DIR}
fastlio_sam_install_prefix: ${pkg_prefix}
fastlio_sam_upstream_commit: ${upstream_commit}
git_commit: ${git_commit}
started_at: ${started_at}
EOF

  dump_gtsam_env
  start_vram_logger
  start_rss_logger
  trap 'stop_loggers' EXIT

  # Upstream's launch invocation is unstable across releases (the README
  # shows both `ros2 launch fast_lio_sam mapping_velodyne.launch.py` and
  # a `build.sh humble` indirection). We delegate to the upstream's own
  # launch file so a future upstream change in node names does not
  # require a wrapper edit. The bag is replayed in a second process
  # because the upstream launch does NOT take a bag positional argument;
  # this means the SLAM node and `ros2 bag play` race on startup. The
  # `--delay` mitigates that.
  # LiDAR: this wrapper hardcodes mapping_velodyne.launch.py because the
  # M5R-3 comparison uses Velodyne VLP-16 bags (the lab platform). FAST-
  # LIO SAM ships separate launch files per LiDAR (mapping_robosense,
  # mapping_unilidar); if M5R-3 ever evaluates an Ouster bag in parallel
  # with GLIM's Ouster config, swap the launch name here or add a
  # bag-introspection branch mirroring select_glim_config() in
  # m5r3_run_glim.sh.
  set +e
  {
    echo "==> FAST-LIO SAM start ${started_at}"
    echo "==> launching SLAM node in background"
    ros2 launch "${PKG_NAME}" mapping_velodyne.launch.py >> "${OUT_DIR}/slam.log" 2>&1 &
    local slam_pid=$!
    sleep 5  # let the SLAM node finish startup before replay begins
    echo "==> replaying bag (pid ${slam_pid} should be alive)"
    /usr/bin/time -p ros2 bag play "${BAG_DIR}" --delay 2
    local play_rc=$?
    echo "==> bag replay finished (rc=${play_rc}); waiting for SLAM node to flush"
    # Give the SLAM node a moment to write its trajectory / PCD before
    # we tear it down. Empirically the upstream emits these on shutdown.
    sleep 5
    kill -INT "${slam_pid}" 2>/dev/null || true
    wait "${slam_pid}" 2>/dev/null || true
    echo "==> FAST-LIO SAM end $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    exit "${play_rc}"
  } 2>&1 | tee "${OUT_DIR}/run.log"
  local rc=${PIPESTATUS[0]}
  set -e

  stop_loggers
  trap - EXIT

  local ended_at
  ended_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  local duration=$(( $(date +%s) - started_epoch ))
  local peak_vram
  peak_vram="$(peak_vram_mib)"
  local peak_rss
  peak_rss="$(peak_rss_kib)"

  cat >> "${OUT_DIR}/manifest.yaml" <<EOF
ended_at: ${ended_at}
duration_sec: ${duration}
max_vram_mib: ${peak_vram}
max_rss_kib: ${peak_rss}
exit_code: ${rc}

# results: filled by ADR-0003 author after running m5r3_loop_error.py and
# CloudCompare wall-point picking. Schema matches the GLIM manifest so
# the two are line-by-line comparable.
results:
  loop_error_trajectory_m: TBD       # from m5r3_loop_error.py end-to-start
  loop_error_wall_3pt_m: TBD         # B1 official, from CloudCompare
  notes: |
    TBD: GTSAM coexistence resolution observed (see gtsam_env.log),
    manual relocalization need, keyframe density, loop-closure trigger
    behaviour, upstream README "Full ROS2 adaptation" TODO impact.
EOF

  if [[ "${rc}" -ne 0 ]]; then
    echo "WARNING: ros2 bag play exited with ${rc}. See ${OUT_DIR}/run.log + slam.log." >&2
  fi
}

# --- next-steps hint ---------------------------------------------------------

print_next_steps() {
  cat >&2 <<EOF

FAST-LIO SAM run complete. Outputs under: ${OUT_DIR}

Next:
  # Upstream's trajectory filename can differ between releases. Inspect
  # the dump dir first:
  ls ${OUT_DIR}
  # Then point the loop-error script at it (likely traj.txt):
  python3 ${REPO_ROOT}/scripts/m5r3_loop_error.py ${OUT_DIR}/traj.txt

For the formal B1 criterion (start/end wall 3-point mean), open the
generated PCD in CloudCompare and follow docs/ja/m5r3-comparison-protocol.md
§"ループ誤差計測".

Transcribe the manifest.yaml + traj loop-error result into the
Alternatives table of docs/ja/decisions/0003-mapping-slam-choice.md.
Also copy the gtsam_env.log content into the same row.
EOF
}

main() {
  echo "[1/6] preflight: bag layout"
  check_bag
  echo "[2/6] preflight: upstream cloned"
  check_upstream_cloned
  echo "[3/6] preflight: package built"
  check_built
  echo "[4/6] preflight: out-dir state"
  check_out_dir_empty
  echo "[5/6] inspecting GTSAM coexistence"
  # dump_gtsam_env is called inside run_fastlio_sam so it lands after the
  # manifest is created; here we only note the step in the [n/6] counter.
  echo "[6/6] running fast_lio_sam (VRAM + RSS sampling)"
  run_fastlio_sam
  print_next_steps
}

main "$@"
