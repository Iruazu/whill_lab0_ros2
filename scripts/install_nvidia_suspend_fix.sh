#!/usr/bin/env bash
# Persist the NVIDIA suspend/resume CUDA fix on the development host.
# See Issue #76 for background and full diagnosis notes.
#
# Symptom this addresses:
#   After systemd suspend/resume, any new CUDA process on this host aborts
#   within ~0.6 s with:
#     warning: cudaErrorUnknown : unknown error
#     warning: frame doesn't have points on GPU
#     error:   GPU points/covs not allocated!!
#     [ros2run]: Aborted
#   `nvidia-smi` still responds correctly, which masks the state — GLIM /
#   any GPU-using ROS 2 node dies at library-load without an obvious cause.
#
# Root cause on this host (Alienware x15 R2, RTX 3080 Ti, driver 595.71.05,
# kernel 6.8): `NVreg_PreserveVideoMemoryAllocations=2` is already set by
# the driver package (nvidia-graphics-drivers-kms.conf), but the CUDA UVM
# kernel module (nvidia_uvm) does not always come back into a usable state
# after resume. The reliable fix is a two-step:
#   1. Enable NVreg_UseKernelSuspendNotifiers=1 so the driver installs its
#      pm_notifier callback and quiesces UVM before userspace freeze.
#   2. On resume, reload nvidia_uvm to force a clean UVM state machine.
#      Any userspace CUDA process from before suspend is already dead (they
#      were frozen and killed by our resume path), so an unload/reload is
#      safe as far as running processes go.
#
# Idempotent — re-running writes the same content each time. Not shipping
# host driver settings via cfengine/ansible on this repo; a single script
# is enough because there is exactly one physical host.
#
# Usage:
#   sudo ./scripts/install_nvidia_suspend_fix.sh            # install
#   sudo ./scripts/install_nvidia_suspend_fix.sh --uninstall # remove
#   ./scripts/install_nvidia_suspend_fix.sh --verify        # read-only check
#
# Reboot required after install for NVreg_UseKernelSuspendNotifiers=1 to
# take effect (nvidia.ko re-reads modprobe.d only on module load, and
# nvidia.ko is loaded at boot). The systemd sleep hook itself takes effect
# immediately (systemd re-scans /lib/systemd/system-sleep on every wake).

set -euo pipefail

MODPROBE_CONF="/etc/modprobe.d/whill-nvidia-uvm.conf"
SLEEP_HOOK="/lib/systemd/system-sleep/whill-nvidia-uvm-reload"

# Content pinned as heredocs so the install target files match what git
# tracks in this script. Do not edit the deployed files by hand — re-run
# this installer instead.
read -r -d '' MODPROBE_BODY <<'EOF' || true
# Managed by scripts/install_nvidia_suspend_fix.sh (Issue #76).
# Do not hand-edit; re-run the installer to change.
#
# Purpose: enable the driver's kernel PM notifier so nvidia_uvm gets a
# clean pre-freeze / post-thaw sequence around systemd suspend/resume.
# Combined with the driver-package default of
# PreserveVideoMemoryAllocations=2, this is upstream's recommended
# configuration for CUDA workloads on kernel 4.20+.
options nvidia NVreg_UseKernelSuspendNotifiers=1
EOF

read -r -d '' SLEEP_HOOK_BODY <<'EOF' || true
#!/bin/sh
# Managed by scripts/install_nvidia_suspend_fix.sh (Issue #76).
# Do not hand-edit; re-run the installer to change.
#
# systemd sleep hook: reload nvidia_uvm on wake so the first CUDA process
# after resume gets a fresh UVM state machine. Without this, the process
# aborts with cudaErrorUnknown at library load even though nvidia-smi
# reports the GPU as healthy.
#
# systemd invokes this script with two positional arguments:
#   $1 = pre | post
#   $2 = suspend | hibernate | hybrid-sleep | suspend-then-hibernate
# We only act on post-wake events. Pre-sleep is left to the driver's
# own nvidia-suspend.service.

case "$1" in
  post)
    case "$2" in
      suspend|hibernate|hybrid-sleep|suspend-then-hibernate)
        # rmmod may fail if UVM has leaked references (rare, but possible
        # if a CUDA process re-attached during the wake race). Do not
        # abort the resume path — log to journal and continue; the next
        # wake will retry. modprobe on top of a still-loaded module is a
        # no-op, so the caller's next CUDA context creation may still
        # work in that degraded case.
        if /sbin/modprobe -r nvidia_uvm 2>/dev/null; then
          /sbin/modprobe nvidia_uvm || true
        else
          echo "whill-nvidia-uvm-reload: rmmod nvidia_uvm failed;" \
               "reload skipped" >&2
        fi
        ;;
    esac
    ;;
esac
EOF

require_root() {
  if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    echo "error: this action needs root (re-run with sudo)" >&2
    exit 1
  fi
}

do_verify() {
  # Read-only: safe to run without sudo. Reports current state.
  echo "== installed files =="
  for f in "${MODPROBE_CONF}" "${SLEEP_HOOK}"; do
    if [[ -f "${f}" ]]; then
      printf "  present   %s\n" "${f}"
    else
      printf "  MISSING   %s\n" "${f}"
    fi
  done
  echo
  echo "== live driver params =="
  if [[ -r /proc/driver/nvidia/params ]]; then
    grep -E 'PreserveVideoMemoryAllocations|UseKernelSuspendNotifiers' \
      /proc/driver/nvidia/params | sed 's/^/  /'
    echo
    echo "  UseKernelSuspendNotifiers=1 is the goal after reboot."
  else
    echo "  /proc/driver/nvidia/params not readable (nvidia.ko unloaded?)"
  fi
  echo
  echo "== systemd sleep hook =="
  if [[ -x "${SLEEP_HOOK}" ]]; then
    echo "  ${SLEEP_HOOK} is executable"
  else
    echo "  ${SLEEP_HOOK} missing or not executable"
  fi
}

install_file() {
  local dst="$1" mode="$2" body="$3"
  local tmp
  tmp="$(mktemp)"
  printf '%s\n' "${body}" >"${tmp}"
  # cmp exits non-zero on mismatch or missing target; both paths trigger
  # the mv so `set -e` above does not abort on the first run.
  if [[ -f "${dst}" ]] && cmp -s "${tmp}" "${dst}"; then
    echo "  unchanged ${dst}"
    rm -f "${tmp}"
    return
  fi
  install -m "${mode}" "${tmp}" "${dst}"
  rm -f "${tmp}"
  echo "  written   ${dst}"
}

do_install() {
  require_root
  echo "== installing =="
  install_file "${MODPROBE_CONF}" 0644 "${MODPROBE_BODY}"
  install_file "${SLEEP_HOOK}"    0755 "${SLEEP_HOOK_BODY}"
  echo
  echo "== next steps =="
  echo "  1. Reboot to activate NVreg_UseKernelSuspendNotifiers=1."
  echo "     (The sleep hook itself is already live.)"
  echo "  2. After reboot, verify with:"
  echo "     ${BASH_SOURCE[0]} --verify"
  echo "  3. Manual test: \`systemctl suspend\`, resume, then re-run"
  echo "     scripts/m5r3_run_glim.sh and confirm run.log has 0 lines"
  echo "     matching 'cudaErrorUnknown'."
}

do_uninstall() {
  require_root
  echo "== uninstalling =="
  for f in "${MODPROBE_CONF}" "${SLEEP_HOOK}"; do
    if [[ -f "${f}" ]]; then
      rm -f "${f}"
      echo "  removed   ${f}"
    else
      echo "  absent    ${f}"
    fi
  done
  echo
  echo "NVreg_UseKernelSuspendNotifiers=1 stays in effect until reboot."
}

usage() {
  cat >&2 <<EOF
Usage: ${0##*/} [--install|--uninstall|--verify]

  (no arg)     alias for --install
  --install    write files (needs root)
  --uninstall  remove files (needs root)
  --verify     read-only report of current state

See Issue #76 for background.
EOF
  exit 2
}

case "${1:-}" in
  ""|--install) do_install ;;
  --uninstall)  do_uninstall ;;
  --verify)     do_verify ;;
  -h|--help)    usage ;;
  *)            usage ;;
esac
