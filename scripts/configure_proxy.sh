#!/usr/bin/env bash
# Configure apt and SSH for use behind the Utsunomiya University HTTP proxy.
#
# Idempotent — safe to re-run. Run as a normal user; this script invokes sudo
# only for the apt config file.
#
# Usage:
#   ./configure_proxy.sh
#
# Effects:
#   * /etc/apt/apt.conf.d/95proxies — apt uses the proxy for http/https/ftp
#   * ~/.ssh/config — SSH to GitHub is tunneled over HTTPS through the proxy
#
# Shell-level env vars (HTTP_PROXY etc.) are NOT touched by this script —
# add them to ~/.bashrc or ~/.profile yourself.

set -euo pipefail

PROXY_HOST="proxy.cc.utsunomiya-u.ac.jp"
PROXY_PORT="8080"
PROXY_URL="http://${PROXY_HOST}:${PROXY_PORT}"

configure_apt() {
  local conf=/etc/apt/apt.conf.d/95proxies
  sudo tee "${conf}" > /dev/null <<EOF
Acquire::http::Proxy  "${PROXY_URL}";
Acquire::https::Proxy "${PROXY_URL}";
Acquire::ftp::Proxy   "${PROXY_URL}";
EOF
  echo "wrote ${conf}"
}

configure_ssh() {
  local conf=~/.ssh/config
  mkdir -p ~/.ssh
  chmod 700 ~/.ssh
  if [[ -f "${conf}" ]] && grep -q "Host github.com" "${conf}"; then
    echo "SSH config already has a github.com block — leaving as-is"
    return 0
  fi
  cat >> "${conf}" <<EOF

Host github.com
    Hostname ssh.github.com
    Port 443
    User git
    ProxyCommand nc -X connect -x ${PROXY_HOST}:${PROXY_PORT} %h %p
    ServerAliveInterval 60
EOF
  chmod 600 "${conf}"
  echo "appended github.com block to ${conf}"
}

main() {
  configure_apt
  configure_ssh
  echo "Done. Verify with:  ssh -T git@github.com"
}

main "$@"
