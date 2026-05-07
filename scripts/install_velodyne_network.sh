#!/usr/bin/env bash
# Install a netplan config that puts the host-side USB-Ethernet adapter
# (the one cabled to the Velodyne VLP-16) on the 192.168.1.0/24 subnet
# at .100. The LiDAR's factory default IP is 192.168.1.201 — adjust the
# template if your unit was reprogrammed to a different subnet.
#
# Idempotent — safe to re-run; overwrites
# /etc/netplan/01-velodyne-static.yaml with the rendered template.
#
# Usage:
#   ./scripts/install_velodyne_network.sh <iface>
#
# where <iface> is the predictable name of the USB-Ethernet adapter
# (e.g. enxAABBCCDDEEFF). Find candidates with:
#   ip -br link show | grep -E '^(enx|eth)'

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE="${REPO_ROOT}/network/01-velodyne-static.yaml.template"
DST="/etc/netplan/01-velodyne-static.yaml"

main() {
  local iface="${1:-}"
  if [[ -z "${iface}" ]]; then
    echo "Usage: $(basename "$0") <iface>" >&2
    echo >&2
    echo "Candidate USB-Ethernet / Ethernet interfaces on this host:" >&2
    ip -br link show | grep -E '^(enx|eth|enp)' >&2 || echo "  (none found)" >&2
    exit 1
  fi

  if ! ip link show "${iface}" > /dev/null 2>&1; then
    echo "ERROR: interface '${iface}' not found." >&2
    exit 1
  fi

  if [[ ! -f "${TEMPLATE}" ]]; then
    echo "ERROR: ${TEMPLATE} not found." >&2
    exit 1
  fi

  local tmpfile
  tmpfile="$(mktemp)"
  trap 'rm -f "${tmpfile}"' EXIT
  sed "s/\${IFACE}/${iface}/g" "${TEMPLATE}" > "${tmpfile}"

  echo "Rendered config:"
  cat "${tmpfile}"
  echo

  echo "Installing -> ${DST}"
  sudo install -m 600 -o root -g root "${tmpfile}" "${DST}"

  echo "Applying netplan..."
  sudo netplan apply

  echo
  echo "Done. Verify with:"
  echo "  ip -4 addr show ${iface}"
  echo "  ping -c 3 192.168.1.201   # Velodyne VLP-16 (factory default)"
}

main "$@"
