#!/usr/bin/env bash
# Install ROS 2 humble Desktop on Ubuntu 22.04 (jammy).
#
# Idempotent — safe to re-run. Uses the official ros2-apt-source procedure
# documented at https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
#
# Usage:
#   ./install_ros2_humble.sh
#
# If the host sits behind an HTTP proxy, set HTTP_PROXY / HTTPS_PROXY in your
# shell *before* invoking this script. apt also needs the proxy in
# /etc/apt/apt.conf.d/95proxies (see configure_proxy.sh).

set -euo pipefail

require_jammy() {
  source /etc/os-release
  if [[ "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}" != "jammy" ]]; then
    echo "ERROR: this script targets Ubuntu 22.04 (jammy). Detected: ${VERSION_CODENAME}" >&2
    exit 1
  fi
}

setup_locale() {
  sudo apt-get install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
}

enable_universe() {
  sudo apt-get install -y software-properties-common
  sudo add-apt-repository -y universe
}

install_ros2_apt_source() {
  sudo apt-get install -y curl gnupg2 lsb-release
  local version
  version=$(curl -sS https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' \
    | awk -F'"' '{print $4}')
  if [[ -z "${version}" ]]; then
    echo "ERROR: could not resolve latest ros-apt-source release" >&2
    exit 1
  fi
  local codename
  codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
  local url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.${codename}_all.deb"
  curl -sSL -o /tmp/ros2-apt-source.deb "${url}"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
}

install_ros2_desktop() {
  sudo DEBIAN_FRONTEND=noninteractive apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-desktop ros-dev-tools
}

main() {
  require_jammy
  echo "[1/4] locale"
  setup_locale
  echo "[2/4] universe repo"
  enable_universe
  echo "[3/4] ROS 2 apt source"
  install_ros2_apt_source
  echo "[4/4] ros-humble-desktop + ros-dev-tools"
  install_ros2_desktop
  echo "Done. Run: source /opt/ros/humble/setup.bash"
}

main "$@"
