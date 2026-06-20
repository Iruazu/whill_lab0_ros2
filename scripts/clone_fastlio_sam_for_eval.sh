#!/usr/bin/env bash
# Clone FAST-LIO SAM (RightTr fork) into src/third_party/ for offline,
# evaluation-only use under M5R-3 (Issue #48, "GLIM vs FAST-LIO SAM on real
# bags"). Companion to docs/ja/m5r-fastlio-sam-eval.md.
#
# Why this is *not* `vcs import` via whill_lab.repos:
#   * Upstream has no LICENSE file (verified 2026-06-20 via WebFetch and
#     `find -iname LICENSE*`). package.xml claims <license>BSD</license>,
#     but there is no LICENSE text in the repository, so the claim has no
#     enforceable backing. Under copyright law this is effectively
#     "all rights reserved", which is stricter than permissive BSD/MIT.
#   * The upstream is derived from HKU-MaRS FAST-LIO (GPL-2.0), so the
#     copyleft can propagate to derivatives. The platform-pivot plan
#     §3.4 restricts GPL-family code to "offline map-building tool in a
#     separated process" — admissible for map building, NOT for the
#     operational stack.
#   * Putting this upstream in whill_lab.repos would turn every clone of
#     this repo into an implicit re-distribution of license-uncertain
#     code (since vcs import runs automatically). That is too broad.
#   * Clone-on-demand from a script puts the responsibility on the
#     individual evaluator and gates the operation behind the explicit
#     env-var acknowledgement below.
#
# What this script does NOT do:
#   * It does not run `colcon build`. The upstream README still lists
#     "Full ROS2 adaptation" and "ROS2 adaptation Test" as TODO, so the
#     build outcome is an open question, and that question is M5R-3
#     evaluator's call rather than this Issue's.
#   * It does not modify any operational package or link FAST-LIO SAM
#     into one. That is explicitly forbidden by the platform-pivot plan.
#
# Idempotent — safe to re-run. apt install short-circuits on already-present
# packages; clone short-circuits to fetch+checkout when the destination is
# already a working tree.
#
# Usage:
#   FASTLIO_SAM_LICENSE_ACK=yes ./scripts/clone_fastlio_sam_for_eval.sh
#
# Without FASTLIO_SAM_LICENSE_ACK=yes the script exits 1 after printing
# the license caveat. The variable is a misuse guard, not security — the
# point is that the evaluator must take an explicit action to acknowledge
# the license situation before the upstream lands in their tree.

# Match install_glim.sh's `set -euo pipefail` policy. -u and -o pipefail
# are insurance against later edits introducing pipes or new variable
# references — without them, a typo'd $VAR on the LHS of a pipe would
# silently fail and the script would keep going. This script does not
# source /opt/ros/humble/setup.bash (we only read $ROS_DISTRO), so the
# AMENT_TRACE_SETUP_FILES / COLCON_TRACE trap that install_glim.sh
# works around with `set +u; ...; set -u` does not apply here. If a
# future edit ever needs to source a ROS setup file, wrap it the same
# way install_glim.sh does.
set -euo pipefail

UPSTREAM_REPO="https://github.com/RightTr/FAST-LIO-SAM.git"
UPSTREAM_REF="master"
# GTSAM PPA: upstream README for FAST-LIO SAM lists this exact PPA as the
# Ubuntu 22.04 prerequisite (GTSAM 4.1 line). We do NOT switch to the
# GTSAM 4.3a0 source build that install_glim.sh provides — FAST-LIO SAM
# is pinned against the 4.1 API and a 4.3 build typically fails the
# find_package version check. See docs/ja/m5r-fastlio-sam-eval.md §3
# ("GTSAM の競合に関する警告") for the coexistence discussion.
GTSAM_PPA="ppa:borglab/gtsam-release-4.1"
GTSAM_APT_PKGS=(libgtsam-dev libgtsam-unstable-dev)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
THIRD_PARTY="${REPO_ROOT}/src/third_party"
CLONE_DEST="${THIRD_PARTY}/FAST_LIO_SAM"

# --- helpers -----------------------------------------------------------------

require_jammy() {
  # FAST-LIO SAM upstream documents Ubuntu 22.04 as its target. The GTSAM
  # PPA also publishes for jammy specifically; on noble (24.04) the PPA
  # entry would silently be a no-op or pull the wrong release. Fail loud.
  source /etc/os-release
  if [[ "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}" != "jammy" ]]; then
    echo "ERROR: this script targets Ubuntu 22.04 (jammy). Detected: ${VERSION_CODENAME:-unknown}" >&2
    exit 1
  fi
}

require_humble() {
  # We don't auto-source /opt/ros/humble/setup.bash — masking the user's
  # missing source line would make the later `colcon build` failure
  # opaque. Check the env that the user is expected to have set, mirror
  # the convention in install_glim.sh.
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

acknowledge_license_caveat() {
  # The env-var gate exists because:
  #   * Cloning into src/third_party/ leaves a copy of upstream code on
  #     the host. Given the upstream has no LICENSE file, that copy is in
  #     legal grey territory and the evaluator should be making that
  #     decision consciously, not as a side effect of running an
  #     installer.
  #   * Without the gate, someone glancing at scripts/ and running every
  #     file would end up with the clone unintentionally. The gate forces
  #     them to read this caveat first.
  if [[ "${FASTLIO_SAM_LICENSE_ACK:-}" != "yes" ]]; then
    cat >&2 <<EOF
ERROR: FASTLIO_SAM_LICENSE_ACK is not set to "yes".

FAST-LIO SAM upstream (${UPSTREAM_REPO}) has NO LICENSE file and no
license declaration in its README. Its package.xml claims BSD, but
without LICENSE text the claim has no enforceable backing.

The upstream is derived from HKU-MaRS FAST-LIO (GPL-2.0), so GPL
copyleft can propagate. Per docs/ja/plans/2026-06-11-platform-pivot.md
§3.4, this means FAST-LIO SAM may be used as an offline map-building
tool only; it must NOT be linked from the operational stack and must
NOT be added to whill_lab.repos.

If you accept this scope and are evaluating FAST-LIO SAM for M5R-3
(Issue #48), re-run as:

  FASTLIO_SAM_LICENSE_ACK=yes ./scripts/clone_fastlio_sam_for_eval.sh

See docs/ja/m5r-fastlio-sam-eval.md for the full discussion.
EOF
    exit 1
  fi
  echo "License caveat acknowledged via FASTLIO_SAM_LICENSE_ACK=yes."
}

install_gtsam_ppa() {
  # We install the GTSAM 4.1 line via the borglab PPA rather than a
  # source build because (a) FAST-LIO SAM was authored against this
  # exact line so it has the best chance of building unmodified, and (b)
  # we already have a 4.3a0 source build for GLIM and adding a second
  # source build of GTSAM at a different version would just compound the
  # coexistence problem.
  #
  # Idempotency: only run add-apt-repository if the PPA list file is
  # absent, otherwise apt would re-prompt. The fragment file name is the
  # standard add-apt-repository output for PPAs on jammy.
  local apt_list_glob="/etc/apt/sources.list.d/borglab-ubuntu-gtsam-release-4_1-*.list"
  # shellcheck disable=SC2086  # glob expansion is intended
  if ls ${apt_list_glob} >/dev/null 2>&1; then
    echo "GTSAM PPA already present in apt sources — skipping add-apt-repository."
  else
    echo "Adding GTSAM PPA: ${GTSAM_PPA}"
    sudo DEBIAN_FRONTEND=noninteractive add-apt-repository -y "${GTSAM_PPA}"
    sudo DEBIAN_FRONTEND=noninteractive apt-get update
  fi

  local missing=()
  for pkg in "${GTSAM_APT_PKGS[@]}"; do
    if ! dpkg -l "${pkg}" 2>/dev/null | grep -q '^ii'; then
      missing+=("${pkg}")
    fi
  done
  if [[ "${#missing[@]}" -eq 0 ]]; then
    echo "GTSAM apt packages already installed — skipping."
    return 0
  fi
  # Even when the PPA list file is already present (the add-apt-repository
  # branch above was skipped), the apt cache itself may be stale — e.g. on
  # a host where someone added the PPA days ago and never updated since,
  # `apt-get install libgtsam-dev` would 404 on the cached pool URL. Mirror
  # install_glim.sh::install_apt_deps and refresh the cache right before
  # install. Cheap and idempotent.
  echo "Refreshing apt cache before installing: ${missing[*]}"
  sudo DEBIAN_FRONTEND=noninteractive apt-get update
  echo "Installing GTSAM apt packages: ${missing[*]}"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing[@]}"
}

clone_or_update() {
  # Same idempotency pattern as install_glim.sh::clone_or_update. The
  # branch-vs-tag/SHA split exists because a bare `git checkout <ref>` is
  # a silent no-op on an up-to-date local branch even if origin/<ref>
  # has advanced after the fetch; without the explicit `checkout -B`
  # against origin we would not actually re-track upstream master on
  # re-runs. For tags/SHAs detached HEAD is correct.
  mkdir -p "${THIRD_PARTY}"
  if [[ ! -d "${CLONE_DEST}/.git" ]]; then
    echo "Cloning ${UPSTREAM_REPO} -> ${CLONE_DEST}"
    git clone --recurse-submodules "${UPSTREAM_REPO}" "${CLONE_DEST}"
  else
    echo "Updating ${CLONE_DEST} (fetch only)"
    git -C "${CLONE_DEST}" fetch --tags --recurse-submodules origin
  fi
  echo "Checking out ${UPSTREAM_REF} in ${CLONE_DEST}"
  if git -C "${CLONE_DEST}" ls-remote --exit-code origin "refs/heads/${UPSTREAM_REF}" >/dev/null 2>&1; then
    git -C "${CLONE_DEST}" checkout -B "${UPSTREAM_REF}" "origin/${UPSTREAM_REF}"
  else
    git -C "${CLONE_DEST}" checkout "${UPSTREAM_REF}"
  fi
  git -C "${CLONE_DEST}" submodule update --init --recursive
}

verify() {
  # The package.xml <name> check is deliberately specific: it confirms we
  # cloned the expected upstream tree (and not, say, a renamed fork) and
  # that the package name M5R-3 will pass to `colcon build
  # --packages-up-to fast_lio_sam` actually resolves. A missing or
  # different name here is a hard fail rather than a warning, because
  # downstream evaluation steps would silently break.
  local pkg_xml="${CLONE_DEST}/package.xml"
  if [[ ! -f "${pkg_xml}" ]]; then
    echo "ERROR: ${pkg_xml} not found after clone." >&2
    exit 1
  fi
  if ! grep -qE '<name>\s*fast_lio_sam\s*</name>' "${pkg_xml}"; then
    echo "ERROR: ${pkg_xml} does not declare <name>fast_lio_sam</name>." >&2
    echo "       Upstream may have renamed the package; update this script." >&2
    exit 1
  fi
  echo "Verified package name fast_lio_sam in ${pkg_xml}."
}

print_next_steps() {
  # stderr, matching install_glim.sh / install_cuda.sh convention. Build
  # and bag-evaluation belong to M5R-3 (#48); we explicitly do NOT call
  # colcon here because (a) build success itself is the open question
  # M5R-3 needs to answer, and (b) this Issue's success criterion is
  # "clone + GTSAM coexistence are documented and reproducible", not
  # "FAST-LIO SAM compiles".
  cat >&2 <<EOF

FAST-LIO SAM is cloned under ${CLONE_DEST}.

This script intentionally stops short of the colcon build. M5R-3 (Issue
#48) is the evaluation phase that decides whether the upstream master is
currently buildable on ROS 2 humble. See
docs/ja/m5r-fastlio-sam-eval.md §"Setup procedure" for the next steps,
in particular:

  * §2 "Build (when M5R-3 evaluator gets there)" — the colcon command
    to try, and the deferred handling if it fails.
  * §3 "GTSAM の競合に関する警告" — how to deal with the coexistence of
    /usr/local GTSAM 4.3a0 (from install_glim.sh) and the PPA's
    GTSAM 4.1.1 just installed here.

Reminder of scope (CLAUDE.md / platform-pivot §3.4):
  * Do NOT link FAST-LIO SAM from operational packages.
  * Do NOT add FAST_LIO_SAM to whill_lab.repos until ADR-0003 says so.
  * Edits inside ${CLONE_DEST} are forbidden by the third_party rule.
EOF
}

main() {
  # Order rationale: the license caveat runs FIRST, before any environment
  # check, so the evaluator sees *what* this script is about to fetch before
  # being told *whether their OS qualifies*. Reversing the order (jammy check
  # first) would silently block non-jammy hosts without the user ever
  # learning the license situation — a poor UX, and not what the env-var
  # gate is for. require_jammy / require_humble are grouped together as the
  # "environment preflight" step so the [N/4] counter still reflects the
  # logical phases (license / env / deps / fetch).
  echo "[1/4] license acknowledgement"
  acknowledge_license_caveat
  echo "[2/4] environment preflight (Ubuntu 22.04 jammy + ROS 2 humble)"
  require_jammy
  require_humble
  echo "[3/4] GTSAM 4.1 (borglab PPA)"
  install_gtsam_ppa
  echo "[4/4] clone src/third_party/FAST_LIO_SAM"
  clone_or_update
  verify
  print_next_steps
}

main "$@"
