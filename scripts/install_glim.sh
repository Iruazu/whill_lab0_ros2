#!/usr/bin/env bash
# Install GLIM (M5-R first-candidate offline map-building SLAM) from source on
# the development host. The lab host's CUDA pin (12.4) does not match the
# binaries available from the upstream apt PPA (koide3/ppa publishes 12.2,
# 12.6 and 13.1 only), so source build is mandatory; we cannot fall back to
# the PPA without diverging from docs/ja/m5r-cuda-setup.md.
#
# Idempotent — safe to re-run. Each step short-circuits when the upstream
# source tree is already cloned at the pinned ref, or when the installed
# artefact is already present under /usr/local. No global state outside
# /usr/local/{include,lib} and the workspace's src/third_party/ is modified.
#
# Scope notes:
#   * Ubuntu 22.04 + CUDA 12.4 only. CUDA 12.4 is pinned by
#     docs/ja/m5r-cuda-setup.md so that the GLIM build is bit-for-bit
#     reproducible across the team; bumping CUDA is a separate change that
#     must update both this script and that document.
#   * The script does NOT install the NVIDIA driver or the CUDA Toolkit.
#     Those are the responsibility of scripts/install_cuda.sh and the
#     companion document; this script exits early if nvcc is not where we
#     expect it.
#   * Upstream commit pins (GTSAM_REF etc.) are tag-based where the upstream
#     publishes tags, and tracking-branch where they do not. The trade-off
#     is documented per-ref below. When upstream cuts a new tag we can use,
#     bump the variable here in a single edit.
#   * Iridescence is the visualiser. It is optional for headless builds but
#     strongly recommended for development; pass "skip-iridescence" as the
#     first positional argument to skip it (CI / headless).
#   * GLIM and glim_ros2 live under src/third_party/ (vcs-import territory,
#     gitignored). This script clones them directly rather than going via
#     vcs import because they are not yet listed in whill_lab.repos — we
#     keep this script as the single source of truth until M5R-3 decides
#     whether GLIM stays in the runtime tree at all.
#
# Usage:
#   ./install_glim.sh                  # full install incl. Iridescence
#   ./install_glim.sh skip-iridescence # headless / CI: skip the visualiser
#
# If the host sits behind an HTTP proxy, set HTTP_PROXY / HTTPS_PROXY in your
# shell *before* invoking this script (matches install_cuda.sh convention).

set -euo pipefail

# --- pinned versions ---------------------------------------------------------
# GTSAM: upstream cut "4.3a0" as the release that the current GLIM HEAD
# requires (the requirement was bumped on 2025-06-15, breaking the 4.2a9
# build that earlier docs referenced). The tag is annotated, so vcs/git
# resolve it to a fixed commit.
GTSAM_REPO="https://github.com/borglab/gtsam.git"
GTSAM_REF="4.3a0"

# gtsam_points has no semver tags upstream — koide3 publishes releases as
# unnamed master commits. We track master here and document the trade-off:
# if reproducibility matters more than freshness, pin to a commit SHA in
# the variable below (e.g. GTSAM_POINTS_REF="abcd1234"). The PPA does the
# same thing internally.
GTSAM_POINTS_REPO="https://github.com/koide3/gtsam_points.git"
GTSAM_POINTS_REF="master"

# Iridescence — same story as gtsam_points (no tags). master.
IRIDESCENCE_REPO="https://github.com/koide3/iridescence.git"
IRIDESCENCE_REF="master"

# GLIM and glim_ros2 — likewise no tags. master.
GLIM_REPO="https://github.com/koide3/glim.git"
GLIM_REF="master"
GLIM_ROS2_REPO="https://github.com/koide3/glim_ros2.git"
GLIM_ROS2_REF="master"

# --- paths -------------------------------------------------------------------
# The repo root is determined relative to this script's location so the
# script works whether invoked from the repo root or via an absolute path.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY="${REPO_ROOT}/src/third_party"

# Out-of-tree sources for the dependencies that are NOT ROS 2 packages
# (GTSAM, gtsam_points, Iridescence). Keeping them under ~/.cache rather
# than the repo avoids polluting `src/` with non-ament directories that
# colcon would otherwise try to inspect.
BUILD_CACHE="${HOME}/.cache/whill_lab0_ros2/glim"

CUDA_PREFIX="/usr/local/cuda-12.4"
NVCC="${CUDA_PREFIX}/bin/nvcc"

# `nproc` minus one keeps the host responsive while compiling; on the 14C/20T
# Alienware host this still gives ~13 parallel cc1plus instances.
JOBS="$(($(nproc) - 1))"
if [[ "${JOBS}" -lt 1 ]]; then JOBS=1; fi

SKIP_IRIDESCENCE=0
if [[ "${1:-}" == "skip-iridescence" ]]; then
  SKIP_IRIDESCENCE=1
fi

# --- helpers -----------------------------------------------------------------

require_jammy() {
  source /etc/os-release
  if [[ "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}" != "jammy" ]]; then
    echo "ERROR: this script targets Ubuntu 22.04 (jammy). Detected: ${VERSION_CODENAME:-unknown}" >&2
    exit 1
  fi
}

require_cuda_124() {
  # The Toolkit is the precondition, not a thing this script installs. Failing
  # loudly here keeps the failure mode close to its cause — without this guard
  # the gtsam_points cmake would later fail with a confusing "no CUDA compiler
  # found" message that doesn't point at scripts/install_cuda.sh.
  if [[ ! -x "${NVCC}" ]]; then
    echo "ERROR: ${NVCC} not found. Run scripts/install_cuda.sh first." >&2
    echo "       See docs/ja/m5r-cuda-setup.md for the prerequisite." >&2
    exit 1
  fi
  if ! "${NVCC}" --version | grep -q 'release 12.4'; then
    echo "ERROR: ${NVCC} did not report release 12.4. Output:" >&2
    "${NVCC}" --version >&2 || true
    exit 1
  fi
  echo "Detected nvcc at ${NVCC} (CUDA 12.4)."
}

require_ros2_humble() {
  # glim_ros2 needs a sourced humble environment for colcon to find ament_cmake
  # and the rclcpp headers. We don't auto-source — that masks misconfigured
  # shells — but we check that the env is already sourced.
  if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "ERROR: ROS_DISTRO not set. Source /opt/ros/humble/setup.bash first." >&2
    exit 1
  fi
  if [[ "${ROS_DISTRO}" != "humble" ]]; then
    echo "ERROR: ROS_DISTRO is '${ROS_DISTRO}', expected 'humble'." >&2
    exit 1
  fi
  echo "Detected ROS_DISTRO=${ROS_DISTRO}."
}

# Clone or fast-forward a repo to a specific ref. Idempotent and tolerant of
# the case where the user has already cloned it manually — we only fetch +
# checkout, we never reset --hard.
#
# Why the branch-vs-tag/SHA split below: a plain `git checkout <ref>` is a
# silent no-op when the local branch of that name is already current, even
# if `origin/<ref>` has advanced after the fetch. That breaks idempotency
# in exactly the case we care about — re-running the script after an
# upstream push to master/main. For branch refs we therefore force the
# local branch to track `origin/<ref>` explicitly. For tag or SHA pins
# the original detached-HEAD checkout is already correct (and is what we
# want when GTSAM_POINTS_REF etc. are bumped to commit SHAs for repro).
clone_or_update() {
  local repo="$1"
  local ref="$2"
  local dest="$3"
  if [[ ! -d "${dest}/.git" ]]; then
    echo "Cloning ${repo} -> ${dest}"
    git clone --recurse-submodules "${repo}" "${dest}"
  else
    echo "Updating ${dest} (fetch only)"
    git -C "${dest}" fetch --tags --recurse-submodules origin
  fi
  echo "Checking out ${ref} in ${dest}"
  if git -C "${dest}" ls-remote --exit-code origin "refs/heads/${ref}" >/dev/null 2>&1; then
    # ref is a branch name — force the local branch to point at origin's
    # tip so a re-run actually advances HEAD when upstream has moved.
    git -C "${dest}" checkout -B "${ref}" "origin/${ref}"
  else
    # ref is a tag or SHA — detached HEAD is intentional and correct.
    git -C "${dest}" checkout "${ref}"
  fi
  git -C "${dest}" submodule update --init --recursive
}

# --- step 1: apt deps --------------------------------------------------------

install_apt_deps() {
  # The dependency list is the union of:
  #   * GTSAM build deps (libboost-all-dev, libmetis-dev, libtbb-dev)
  #   * gtsam_points build deps (libomp-dev, fmt/spdlog)
  #   * Iridescence build deps (libglm-dev, libglfw3-dev, libpng-dev,
  #     libjpeg-dev, libeigen3-dev)
  #   * GLIM itself (libomp-dev for OpenMP, libgtest-dev for upstream's
  #     test suite — we don't run it but the cmake gates on it).
  # Listing all of them up front lets a single apt invocation resolve the
  # dep graph; running them per-step would amplify apt overhead and
  # produce confusing dpkg state on failure.
  local pkgs=(
    build-essential
    cmake
    git
    libboost-all-dev
    libeigen3-dev
    libfmt-dev
    libglfw3-dev
    libglm-dev
    libgtest-dev
    libjpeg-dev
    libmetis-dev
    libomp-dev
    libpng-dev
    libspdlog-dev
    libtbb-dev
  )
  local missing=()
  for pkg in "${pkgs[@]}"; do
    if ! dpkg -l "${pkg}" 2>/dev/null | grep -q '^ii'; then
      missing+=("${pkg}")
    fi
  done
  if [[ "${#missing[@]}" -eq 0 ]]; then
    echo "All apt build deps already installed — skipping."
    return 0
  fi
  echo "Installing apt build deps: ${missing[*]}"
  sudo DEBIAN_FRONTEND=noninteractive apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing[@]}"
}

# --- step 2: GTSAM 4.3a0 -----------------------------------------------------

install_gtsam() {
  # GTSAM 4.3a0 was published 2025-06-15 to match a GLIM API change; older
  # GLIM docs referenced 4.2a9 which no longer compiles against current
  # HEAD. The "a0" pre-release suffix is intentional — borglab/gtsam ships
  # 4.3 as 4.3aN incrementally.
  #
  # CMake flags rationale:
  #   * GTSAM_BUILD_WITH_MARCH_NATIVE=OFF — defaults to ON, which embeds
  #     the build host's microarch into the binary. We want the artefact
  #     to be portable to other lab machines (e.g. the future car-borne
  #     mini-PC), so we disable it.
  #   * GTSAM_USE_SYSTEM_EIGEN=ON — we already have libeigen3-dev pinned
  #     to Ubuntu's version; letting GTSAM ship its own would create two
  #     Eigen ABIs in the same process and break gtsam_points linking.
  #   * GTSAM_WITH_TBB=OFF — TBB integration occasionally interacts
  #     badly with the GLIM threading layer; upstream gtsam_points
  #     recommends OFF.
  #   * GTSAM_BUILD_EXAMPLES_ALWAYS=OFF / GTSAM_BUILD_TESTS=OFF — we are
  #     not validating GTSAM itself here; skipping these halves the build
  #     time on the Alienware host.
  #   * GTSAM_BUILD_UNSTABLE=OFF — the "unstable" sublibrary is consumed by
  #     neither gtsam_points nor GLIM. Leaving it ON drags in an extra
  #     boost::serialization dependency and noticeably extends the build,
  #     for headers we never link against.
  if [[ -f /usr/local/lib/libgtsam.so ]] && [[ -f /usr/local/include/gtsam/config.h ]]; then
    local installed
    installed="$(grep -E '^#define GTSAM_VERSION_STRING' /usr/local/include/gtsam/config.h 2>/dev/null | awk -F'"' '{print $2}')"
    if [[ "${installed}" == "4.3a0" ]] || [[ "${installed}" == "4.3."* ]]; then
      echo "GTSAM ${installed} already installed at /usr/local — skipping."
      return 0
    fi
    echo "Found stale GTSAM ${installed} at /usr/local; rebuilding to ${GTSAM_REF}."
  fi
  local src="${BUILD_CACHE}/gtsam"
  mkdir -p "${BUILD_CACHE}"
  clone_or_update "${GTSAM_REPO}" "${GTSAM_REF}" "${src}"
  mkdir -p "${src}/build"
  (cd "${src}/build" && cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF)
  cmake --build "${src}/build" --parallel "${JOBS}"
  sudo cmake --install "${src}/build"
  sudo ldconfig
}

# --- step 3: gtsam_points (CUDA 12.4) ---------------------------------------

install_gtsam_points() {
  # gtsam_points is the GPU-accelerated point-cloud factor graph extension
  # that GLIM uses for ICP/GICP factors. We force CMAKE_CUDA_COMPILER to
  # the 12.4-pinned nvcc rather than relying on PATH — if the user has
  # multiple CUDA versions installed, cmake will silently pick the most
  # recent one in PATH, which can ABI-mismatch the runtime libraries that
  # docs/ja/m5r-cuda-setup.md pinned. Spelling out nvcc removes that drift.
  #
  # BUILD_WITH_CUDA=ON is the whole reason we're not using the PPA: the
  # PPA's CUDA-enabled binaries are linked against 12.2/12.6/13.1, none of
  # which we are pinned to.
  if [[ -f /usr/local/lib/libgtsam_points.so ]]; then
    echo "gtsam_points already installed at /usr/local — skipping."
    echo "  (delete /usr/local/lib/libgtsam_points.so to force a rebuild.)"
    return 0
  fi
  local src="${BUILD_CACHE}/gtsam_points"
  mkdir -p "${BUILD_CACHE}"
  clone_or_update "${GTSAM_POINTS_REPO}" "${GTSAM_POINTS_REF}" "${src}"
  mkdir -p "${src}/build"
  (cd "${src}/build" && cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_COMPILER="${NVCC}" \
    -DBUILD_WITH_CUDA=ON)
  cmake --build "${src}/build" --parallel "${JOBS}"
  sudo cmake --install "${src}/build"
  sudo ldconfig
}

# --- step 4: Iridescence (optional, default ON) ------------------------------

install_iridescence() {
  if [[ "${SKIP_IRIDESCENCE}" -eq 1 ]]; then
    echo "skip-iridescence requested — skipping visualiser build."
    return 0
  fi
  # Iridescence is the OpenGL visualiser that GLIM uses for trajectory /
  # cloud display. Headless CI runs do not need it (set skip-iridescence),
  # but for development on the dev host the trajectory viewer is the main
  # debugging affordance, so default to ON.
  if [[ -f /usr/local/lib/libiridescence.so ]]; then
    echo "Iridescence already installed at /usr/local — skipping."
    return 0
  fi
  local src="${BUILD_CACHE}/iridescence"
  mkdir -p "${BUILD_CACHE}"
  clone_or_update "${IRIDESCENCE_REPO}" "${IRIDESCENCE_REF}" "${src}"
  mkdir -p "${src}/build"
  (cd "${src}/build" && cmake .. \
    -DCMAKE_BUILD_TYPE=Release)
  cmake --build "${src}/build" --parallel "${JOBS}"
  sudo cmake --install "${src}/build"
  sudo ldconfig
}

# --- step 5: glim + glim_ros2 (under src/third_party/) ----------------------

install_glim() {
  # GLIM core and the ROS 2 wrapper are colcon packages, so they live under
  # src/third_party/ rather than ~/.cache. This is the same convention as
  # FAST_LIO and the rest of whill_lab.repos — it lets `colcon build` see
  # the packages without any extra plumbing. They are not yet *in*
  # whill_lab.repos because M5R-3 may decide GLIM is map-build-only and
  # should not ship to the runtime tree; until that ADR is settled, the
  # clone is driven from this script.
  mkdir -p "${THIRD_PARTY}"
  clone_or_update "${GLIM_REPO}" "${GLIM_REF}" "${THIRD_PARTY}/glim"
  clone_or_update "${GLIM_ROS2_REPO}" "${GLIM_ROS2_REF}" "${THIRD_PARTY}/glim_ros2"

  # Build only the GLIM packages — building the whole workspace here would
  # bring in FAST-LIO / Nav2 / sensor drivers and balloon the time budget.
  # The user can re-run a full `colcon build` afterwards.
  #
  # CMAKE_CUDA_COMPILER is forwarded for the same reason as install_gtsam_points
  # (§3): GLIM itself compiles CUDA TUs, and if a stray nvcc sits earlier on
  # PATH cmake will pick it silently. Mixing nvcc versions between
  # gtsam_points and glim produces ABI-incompatible CUDA runtime symbols
  # that fail at link time. Pin both to the 12.4 toolchain explicitly.
  echo "Building glim and glim_ros2 (this can take 10+ minutes)..."
  (cd "${REPO_ROOT}" && colcon build \
    --packages-select glim glim_ros2 \
    --symlink-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CUDA_COMPILER="${NVCC}")
}

# --- verify ------------------------------------------------------------------

verify() {
  # We check that the colcon environment can see glim_ros2 after sourcing
  # install/setup.bash. Failure here usually means a build error was
  # silently swallowed earlier; do not skip this step.
  if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
    echo "ERROR: ${REPO_ROOT}/install/setup.bash not found after build." >&2
    exit 1
  fi
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/install/setup.bash"
  if ! ros2 pkg list 2>/dev/null | grep -q '^glim_ros2$'; then
    echo "ERROR: glim_ros2 not visible to ros2 pkg list after build." >&2
    exit 1
  fi
  echo "glim_ros2 is visible to ros2 pkg list."
}

# --- next-steps hint ---------------------------------------------------------

print_next_steps() {
  # stderr, same convention as install_cuda.sh's print_path_hint — keeps
  # CI-style wrapping pipelines clean while still informing the user.
  cat >&2 <<EOF

GLIM is installed. To smoke-test with the upstream sample bag:

  # 1. Download the Ouster OS1-128 sample bag (~1 GB):
  #    https://staff.aist.go.jp/k.koide/projects/glim/os1_128_01_downsampled.tar.gz
  #    (bag fetch and mirror notes: docs/ja/m5r-glim-setup.md §4)

  # 2. In one terminal, run GLIM with the OS1-128 config. Note the bag
  #    directory is a positional argument to glim_rosbag — without it the
  #    node has nothing to replay.
  cd ${REPO_ROOT}
  source install/setup.bash
  ros2 run glim_ros2 glim_rosbag \\
    /tmp/glim_sample/os1_128_01_downsampled \\
    --ros-args -p config_path:=\$(ros2 pkg prefix glim_ros2)/share/glim_ros2/config/ \\
    -p dump_path:=/tmp/dump/

  # 3. The trajectory will be written to /tmp/dump/traj_lidar.txt.
  #    See docs/ja/m5r-glim-setup.md §5 for the full procedure.
EOF
}

main() {
  require_jammy
  echo "[1/7] CUDA 12.4 prerequisite"
  require_cuda_124
  echo "[2/7] ROS 2 humble env"
  require_ros2_humble
  echo "[3/7] apt build deps"
  install_apt_deps
  echo "[4/7] GTSAM ${GTSAM_REF}"
  install_gtsam
  echo "[5/7] gtsam_points (CUDA 12.4)"
  install_gtsam_points
  echo "[6/7] Iridescence"
  install_iridescence
  echo "[7/7] glim + glim_ros2 (colcon)"
  install_glim
  verify
  print_next_steps
}

main "$@"
