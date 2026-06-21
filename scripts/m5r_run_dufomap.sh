#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# End-to-end driver for M5R-4 (Issue #49) dynamic-object removal:
# GLIM keyframe dump  ->  per-keyframe PCD with VIEWPOINT  ->  DUFOMap
# ->  single static PCD.
#
# This wrapper exists so a reviewer of the M5-R pipeline can re-run the
# dynamic-removal stage with one command instead of remembering the
# (Python) converter + (Python) DUFOMap-runner sequence.  It also
# centralises the "is dufomap installed?" pre-flight so the failure
# message is consistent regardless of which sub-script the user invokes.
#
# Setup (one-time, on the development host):
#
#   pip install dufomap
#
# Verified on Ubuntu 22.04 + Python 3.10 (the lab host configuration
# baked in by docs/ja/m5r-cuda-setup.md).  DUFOMap pulls a native wheel
# with bundled UFO library; no separate UFO install is needed.
#
# Inputs:
#   <glim-out-dir>  GLIM dump_path produced by scripts/m5r3_run_glim.sh.
#                   Must contain NNNNNN/ keyframe subdirectories.
#   <output-dir>    Directory created for staging + final static PCD.
#
# Outputs:
#   <output-dir>/staging/pcd/*.pcd  per-keyframe PCDs (converter intermediates)
#   <output-dir>/static.pcd         final static cloud, input to M5R-6
#
# Idempotency: refuses to overwrite an existing <output-dir>/static.pcd
# unless --force is passed.  Each sub-script propagates --force, so a
# re-run regenerates the staging too.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat >&2 <<EOF
Usage: ${0##*/} <glim-out-dir> <output-dir> [--force]

  <glim-out-dir>  GLIM output directory (with NNNNNN/ keyframe subdirs)
  <output-dir>    Directory to write staging/ and static.pcd into
  --force         Overwrite existing staging + static.pcd
  -h, --help      Show this message

Companion docs:
  docs/ja/m5r-pipeline.md  (parameters, troubleshooting)
  docs/ja/decisions/0004-dynamic-removal-choice.md  (why DUFOMap)
EOF
  exit 2
}

# Handle --help before arg-count check so `script.sh -h` works even
# without the two positional args.
case "${1:-}" in
  -h|--help) usage ;;
esac
if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage
fi

GLIM_OUT_DIR="$(cd "$1" && pwd)"
OUTPUT_DIR="$2"
FORCE=0
if [[ $# -eq 3 ]]; then
  if [[ "$3" != "--force" ]]; then
    usage
  fi
  FORCE=1
fi

mkdir -p "${OUTPUT_DIR}"
OUTPUT_DIR="$(cd "${OUTPUT_DIR}" && pwd)"

STAGING_DIR="${OUTPUT_DIR}/staging"
STATIC_PCD="${OUTPUT_DIR}/static.pcd"

# --- preflight ---------------------------------------------------------------

check_dufomap_installed() {
  # The import is the only reliable check — there is no `dufomap`
  # console script we can probe for. We deliberately run python3 (not
  # any virtualenv-activated interpreter) because the sub-scripts will
  # do the same; if the user has dufomap in a venv they need to source
  # it before invoking us.
  if ! python3 -c 'import dufomap' >/dev/null 2>&1; then
    cat >&2 <<EOF
ERROR: dufomap is not importable from python3.

  Install with: pip install dufomap

  See docs/ja/m5r-pipeline.md "DUFOMap セットアップ" for the verified
  Ubuntu 22.04 + Python 3.10 setup recipe.
EOF
    exit 1
  fi
}

check_glim_keyframes() {
  # Match directories only — a six-digit filename would otherwise satisfy
  # compgen's pattern but get rejected later by Python's is_dir() check,
  # producing a confusing "shell preflight passed, Python failed" error.
  local found=0
  for path in "${GLIM_OUT_DIR}"/[0-9][0-9][0-9][0-9][0-9][0-9]; do
    if [[ -d "${path}" ]]; then
      found=1
      break
    fi
  done
  if [[ ${found} -eq 0 ]]; then
    echo "ERROR: ${GLIM_OUT_DIR} contains no NNNNNN/ keyframe directories." >&2
    echo "       Did you point at the GLIM output (dump_path), not a parent?" >&2
    exit 1
  fi
}

check_static_pcd_absent() {
  if [[ -e "${STATIC_PCD}" && ${FORCE} -eq 0 ]]; then
    echo "ERROR: ${STATIC_PCD} already exists. Re-run with --force to overwrite." >&2
    exit 1
  fi
}

check_dufomap_installed
check_glim_keyframes
check_static_pcd_absent

# --- stage 1: GLIM keyframes -> per-keyframe PCD -----------------------------

# We always pass --force to the converter when the orchestrator itself
# was invoked with --force, so the staging tree is regenerated cleanly.
# When not in --force mode the orchestrator already verified the final
# static.pcd was absent, but staging may still exist from a partial
# previous run; we let the converter abort in that case to keep the
# "no silent overwrite" invariant.
CONVERTER_FORCE_FLAG=()
if [[ ${FORCE} -eq 1 ]]; then
  CONVERTER_FORCE_FLAG=(--force)
fi

echo "==> Converting GLIM keyframes to per-scan PCDs in ${STAGING_DIR}"
python3 "${SCRIPT_DIR}/m5r_glim_to_pcd.py" \
  --glim-out "${GLIM_OUT_DIR}" \
  --out-dir "${STAGING_DIR}" \
  "${CONVERTER_FORCE_FLAG[@]}"

# --- stage 2: DUFOMap on the staged PCDs -------------------------------------

DUFOMAP_FORCE_FLAG=()
if [[ ${FORCE} -eq 1 ]]; then
  DUFOMAP_FORCE_FLAG=(--force)
fi

echo "==> Running DUFOMap; output -> ${STATIC_PCD}"
python3 "${SCRIPT_DIR}/m5r_run_dufomap_core.py" \
  --data-dir "${STAGING_DIR}" \
  --output "${STATIC_PCD}" \
  "${DUFOMAP_FORCE_FLAG[@]}"

echo
echo "static map: ${STATIC_PCD}"
