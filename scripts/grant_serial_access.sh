#!/usr/bin/env bash
# Add the current user to the `dialout` group so that `/dev/ttyUSB*` is
# accessible without sudo. Required for the WHILL Model CR2 USB serial port.
#
# Idempotent — safe to re-run.
#
# IMPORTANT: group membership only takes effect in *new* login sessions.
# After running this script, log out and back in (or `newgrp dialout`).

set -euo pipefail

main() {
  local user="${SUDO_USER:-${USER}}"
  if id -nG "${user}" | tr ' ' '\n' | grep -qx dialout; then
    echo "${user} already in dialout group — no change."
    return 0
  fi
  sudo usermod -aG dialout "${user}"
  echo "Added ${user} to dialout group."
  echo "Log out and back in (or run: newgrp dialout) for the change to take effect."
}

main "$@"
