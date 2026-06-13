#!/usr/bin/env bash
# Install NVIDIA CUDA Toolkit 12.4 and cuDNN 8 on Ubuntu 22.04 (jammy) as the
# build-time prerequisite for GLIM (M5-R first-candidate map-building SLAM).
#
# Idempotent — safe to re-run. Each step short-circuits when the target package
# is already installed; no global state outside /etc/apt and /usr/local/cuda-*
# is modified.
#
# Scope notes:
#   * Ubuntu 22.04 only. CUDA 12.4 has prebuilt repos for jammy; for noble
#     (24.04) the repo URL and pinning scheme differ, and our lab host is
#     pinned to 22.04 by the ROS 2 humble requirement (see CLAUDE.md §9).
#     The mismatch is loud, not silent, so we exit early in require_jammy.
#   * Requires a working NVIDIA driver (>= 525). We do not install or replace
#     the driver here — the dkms / Secure Boot interaction is fragile and
#     belongs to the OS install step, not to a repeatable tooling script.
#     nvidia-driver-595 is already installed on the lab host and is forward-
#     compatible with the full CUDA 12.x runtime line.
#   * PATH / LD_LIBRARY_PATH are *not* auto-appended to any rc file. Multiple
#     CUDA versions can co-exist under /usr/local/, and silently pinning one
#     of them into the user's shell startup has bitten us before
#     (configure_proxy.sh follows the same convention). We print the lines
#     for the user to paste in.
#
# Usage:
#   ./install_cuda.sh
#
# If the host sits behind an HTTP proxy, set HTTP_PROXY / HTTPS_PROXY in your
# shell *before* invoking this script, and ensure /etc/apt/apt.conf.d/95proxies
# is in place (see configure_proxy.sh).

set -euo pipefail

CUDA_KEYRING_DEB="cuda-keyring_1.1-1_all.deb"
CUDA_KEYRING_URL="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/${CUDA_KEYRING_DEB}"
CUDA_TOOLKIT_PKG="cuda-toolkit-12-4"
CUDA_PREFIX="/usr/local/cuda-12.4"

require_jammy() {
  source /etc/os-release
  if [[ "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}" != "jammy" ]]; then
    echo "ERROR: this script targets Ubuntu 22.04 (jammy). Detected: ${VERSION_CODENAME:-unknown}" >&2
    exit 1
  fi
}

require_driver() {
  # Driver-before-toolkit is a hard ordering: the toolkit runtime libraries
  # link against the driver's userspace stub, and `nvidia-smi` is the quickest
  # smoke test that the kernel module loaded. We accept any driver here; the
  # docs spell out the minimum (>= 525 for cuDNN 8 / CUDA 12.4).
  if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "ERROR: nvidia-smi not found. Install the NVIDIA driver first" >&2
    echo "       (e.g. sudo apt-get install nvidia-driver-595) and reboot." >&2
    exit 1
  fi
  echo "Detected NVIDIA driver:"
  nvidia-smi --query-gpu=driver_version,name --format=csv,noheader | sed 's/^/  /'
}

setup_repo() {
  # cuda-keyring is NVIDIA's modern entry point: installing this .deb drops the
  # signed-by keyring under /usr/share/keyrings/ and a matching sources.list
  # fragment under /etc/apt/sources.list.d/cuda-ubuntu2204-x86_64.list. Re-running
  # `dpkg -i` would work but is noisy; we skip the download entirely if the
  # package is already registered.
  if dpkg -l cuda-keyring 2>/dev/null | grep -q '^ii'; then
    echo "cuda-keyring already installed — skipping repo setup."
    return 0
  fi
  echo "Fetching ${CUDA_KEYRING_DEB} from developer.download.nvidia.com"
  local tmp
  tmp="$(mktemp -d)"
  trap 'rm -rf "${tmp}"' RETURN
  curl -fsSL -o "${tmp}/${CUDA_KEYRING_DEB}" "${CUDA_KEYRING_URL}"
  sudo dpkg -i "${tmp}/${CUDA_KEYRING_DEB}"
  sudo apt-get update
}

install_toolkit() {
  # We pin the minor version (cuda-toolkit-12-4) instead of cuda-toolkit. The
  # umbrella metapackage tracks "latest 12.x", which makes the GLIM build
  # reproducibility worse — we want bit-for-bit identical nvcc across the
  # team. Note the apt name uses hyphens (12-4), not dots.
  if dpkg -l "${CUDA_TOOLKIT_PKG}" 2>/dev/null | grep -q '^ii'; then
    echo "${CUDA_TOOLKIT_PKG} already installed — skipping."
    return 0
  fi
  # Refresh apt cache here, not in setup_repo. setup_repo short-circuits when
  # cuda-keyring is already present, but the apt cache for the NVIDIA repo can
  # still be stale (e.g. previous run installed the keyring then aborted before
  # `apt-get update`). Without this refresh, `apt-get install cuda-toolkit-12-4`
  # would fail with "Unable to locate package" and the cause would be opaque.
  echo "Refreshing apt cache before installing ${CUDA_TOOLKIT_PKG}"
  sudo apt-get update
  echo "Installing ${CUDA_TOOLKIT_PKG}"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${CUDA_TOOLKIT_PKG}"
}

install_cudnn() {
  # libcudnn8 / libcudnn8-dev come through the same NVIDIA apt repo as the
  # toolkit (now that cuDNN ships there since 8.9). We stay on the 8.x line
  # because GLIM and the rest of the CUDA 12.x ecosystem we depend on have
  # been validated against it; cuDNN 9 is API-compatible but newer than any
  # of our pinned upstream releases were tested on.
  local need_install=0
  for pkg in libcudnn8 libcudnn8-dev; do
    if ! dpkg -l "${pkg}" 2>/dev/null | grep -q '^ii'; then
      need_install=1
      break
    fi
  done
  if [[ "${need_install}" -eq 0 ]]; then
    echo "libcudnn8 / libcudnn8-dev already installed — skipping."
    return 0
  fi
  echo "Installing libcudnn8 + libcudnn8-dev"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y libcudnn8 libcudnn8-dev
}

verify() {
  # The toolkit binaries live under ${CUDA_PREFIX}/bin/. We don't rely on PATH
  # here because main() doesn't touch the user's shell — we want this verify
  # step to work even on a freshly-installed system where nothing is exported.
  local nvcc="${CUDA_PREFIX}/bin/nvcc"
  if [[ ! -x "${nvcc}" ]]; then
    echo "ERROR: ${nvcc} not found after install. Check apt logs." >&2
    exit 1
  fi
  if ! "${nvcc}" --version | grep -q 'release 12.4'; then
    echo "ERROR: ${nvcc} did not report release 12.4. Output:" >&2
    "${nvcc}" --version >&2 || true
    exit 1
  fi
  echo "nvcc reports CUDA 12.4."

  # cuDNN ships the version macros in cudnn_version.h. We accept the header
  # at either the historical location (/usr/include) or the multi-arch one
  # (/usr/include/x86_64-linux-gnu) depending on packaging vintage.
  local cudnn_hdr=""
  for candidate in /usr/include/cudnn_version.h /usr/include/x86_64-linux-gnu/cudnn_version.h; do
    if [[ -f "${candidate}" ]]; then
      cudnn_hdr="${candidate}"
      break
    fi
  done
  if [[ -z "${cudnn_hdr}" ]]; then
    echo "ERROR: cudnn_version.h not found. libcudnn8-dev did not install correctly." >&2
    exit 1
  fi
  local major
  major="$(awk '/#define CUDNN_MAJOR/ {print $3}' "${cudnn_hdr}")"
  if [[ -z "${major}" || "${major}" -lt 8 ]]; then
    echo "ERROR: cuDNN major version is '${major}', expected >= 8 (from ${cudnn_hdr})." >&2
    exit 1
  fi
  echo "cuDNN reports MAJOR=${major} (from ${cudnn_hdr})."
}

print_path_hint() {
  # We print to stderr so that scripts wrapping this one (e.g. CI) can ignore
  # the hint while users on a terminal still see it. Same convention as
  # configure_proxy.sh's final advice block.
  cat >&2 <<EOF

CUDA Toolkit 12.4 is installed under ${CUDA_PREFIX}/.
To use nvcc and CUDA libraries, add the following to your shell rc (~/.bashrc):

  export PATH=${CUDA_PREFIX}/bin\${PATH:+:\${PATH}}
  export LD_LIBRARY_PATH=${CUDA_PREFIX}/lib64\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}

(this script does not write to your rc file; do it yourself to avoid surprises.)
EOF
}

main() {
  require_jammy
  echo "[1/5] driver presence"
  require_driver
  echo "[2/5] NVIDIA apt repo"
  setup_repo
  echo "[3/5] ${CUDA_TOOLKIT_PKG}"
  install_toolkit
  echo "[4/5] cuDNN 8"
  install_cudnn
  echo "[5/5] verify"
  verify
  print_path_hint
}

main "$@"
