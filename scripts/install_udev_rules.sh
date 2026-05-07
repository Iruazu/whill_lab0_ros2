#!/usr/bin/env bash
# Install the WHILL-stack udev rules so that the WHILL USB-serial cable and
# the RT 9-axis IMU appear at stable paths /dev/whill and /dev/imu, regardless
# of which USB port they were plugged into and in what order.
#
# Idempotent — safe to re-run. Replaces /etc/udev/rules.d/99-whill-stack.rules
# with the version tracked in this repo and reloads udev.
#
# After installation:
#   ls -la /dev/whill /dev/imu
# should show symlinks owned root:dialout 0660. The current user must be a
# member of `dialout` to read/write without sudo (see grant_serial_access.sh).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_RULE="${REPO_ROOT}/udev/99-whill-stack.rules"
DST_RULE="/etc/udev/rules.d/99-whill-stack.rules"

main() {
  if [[ ! -f "${SRC_RULE}" ]]; then
    echo "ERROR: ${SRC_RULE} not found." >&2
    exit 1
  fi

  if [[ -f "${DST_RULE}" ]] && cmp -s "${SRC_RULE}" "${DST_RULE}"; then
    echo "${DST_RULE} already up to date — reloading udev anyway."
  else
    echo "Installing ${DST_RULE}"
    sudo install -m 644 -o root -g root "${SRC_RULE}" "${DST_RULE}"
  fi

  sudo udevadm control --reload
  sudo udevadm trigger --action=change --subsystem-match=tty

  echo
  echo "Resulting symlinks (only present when the matching device is plugged in):"
  ls -la /dev/whill /dev/imu 2>/dev/null || echo "  (no matching device currently connected — symlinks will appear on plug-in)"
}

main "$@"
