# M5-R prerequisite: source-build setup for GLIM

Language: [日本語](../ja/m5r-glim-setup.md) | [English](m5r-glim-setup.md)

## Goal

Install [GLIM](https://github.com/koide3/glim), the first-candidate
map-building SLAM for M5-R (the offline map-production pipeline), from
source on the development host (Alienware x15 R2), and smoke-test it
against the upstream sample bag through to trajectory output. This is the
prerequisite for the upcoming M5R-3 empirical comparison of GLIM versus
FAST-LIO SAM on real bags.

Why source-build:

- The upstream apt PPA (`koide3/ppa`) does not ship a CUDA 12.4 build
  (only 12.2 / 12.6 / 13.1). This repository pins CUDA at 12.4 in
  [`m5r-cuda-setup.md`](m5r-cuda-setup.md), so adopting the PPA would
  silently drift the CUDA version.
- The trade-off is "easy but unpinnable PPA" versus "heavy but
  reproducible source build". Because M5R-3 needs a bit-for-bit
  reproducible build to evaluate fairly, we pick the side that gives us
  pinning.
- GLIM, gtsam_points and Iridescence upstreams are all MIT/BSD licensed,
  which is consistent with this repository's
  [§3.4 license policy](plans/2026-06-11-platform-pivot.md) of keeping
  the operational stack permissive.

For the selection rationale and how it traces back to the project
requirements, see
[`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
§3.3 and §9.

## Host environment

| | |
|--|--|
| Host | Alienware x15 R2 (`systemlab-Alienware-x15-R2`) |
| OS | Ubuntu 22.04.5 LTS (jammy) |
| GPU | NVIDIA GeForce RTX 3080 Laptop GPU (16 GB VRAM, Ampere CC 8.6) |
| NVIDIA Driver | 595.71.05 |
| CUDA Toolkit | 12.4 (`/usr/local/cuda-12.4`, installed via [`m5r-cuda-setup.md`](m5r-cuda-setup.md) §2) |
| cuDNN | 8.x (same source as above) |
| ROS 2 | humble Desktop (`/opt/ros/humble`) |

When the CUDA Toolkit is not installed (which was the state at the time
Issue #45 was filed), the build cannot proceed. Get the `vectorAdd`
sample in [`m5r-cuda-setup.md`](m5r-cuda-setup.md) §4 to print
`Result = PASS` first.

## License alignment

| Component | Upstream repo | License | Alignment with policy |
|---|---|---|---|
| GLIM | [`koide3/glim`](https://github.com/koide3/glim) | MIT | permissive; embeddable in the operational stack |
| glim_ros (repo is [`koide3/glim_ros2`](https://github.com/koide3/glim_ros2)) | upstream | MIT | same. **Note**: the repo URL is `glim_ros2`, but `package.xml` declares the name as `glim_ros` (upstream naming inconsistency). Always use `glim_ros` in `colcon build --packages-select`, `ros2 run`, `ros2 pkg list`. |
| gtsam_points | [`koide3/gtsam_points`](https://github.com/koide3/gtsam_points) | MIT | same |
| GTSAM | [`borglab/gtsam`](https://github.com/borglab/gtsam) | BSD | permissive; embeddable |
| Iridescence | [`koide3/iridescence`](https://github.com/koide3/iridescence) | MIT | visualisation only, outside the operational core |

All of these are permissive, which leaves the door open for GLIM to
re-enter the operational stack in M6-R or later. For this document the
treatment is map-building-only. For the GPL boundary against FAST-LIO and
friends, see [§3.4 of the platform-pivot plan](plans/2026-06-11-platform-pivot.md).

## Setup procedure

### 0. Verify CUDA 12.4 is present

```bash
/usr/local/cuda-12.4/bin/nvcc --version
```

Confirm that the output contains `release 12.4`. If it does not, the CUDA
Toolkit is not installed; run `scripts/install_cuda.sh` first as
described in [`m5r-cuda-setup.md`](m5r-cuda-setup.md) §2.

At the time Issue #45 was filed, `nvidia-driver-595` was running but the
Toolkit itself had disappeared. This step is where you redo the install.

### 1. Run install_glim.sh

From the repository root:

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash    # exports ROS_DISTRO=humble into the env
./scripts/install_glim.sh
```

To skip the Iridescence visualiser (CI / headless host):

```bash
./scripts/install_glim.sh skip-iridescence
```

The script runs the following in order (see the header comment in the
script for the rationale of each step):

1. Verify Ubuntu 22.04, CUDA 12.4 nvcc, and `ROS_DISTRO=humble` (exit 1
   otherwise).
2. Install apt build deps
   (`libomp-dev libboost-all-dev libmetis-dev libfmt-dev libspdlog-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev libtbb-dev`,
   etc.; skipped if already present).
3. Build GTSAM `4.3a0` from source under
   `~/.cache/whill_lab0_ros2/glim/gtsam` and install to `/usr/local`.
4. Build gtsam_points (master) with CUDA 12.4 explicitly
   (`CMAKE_CUDA_COMPILER=/usr/local/cuda-12.4/bin/nvcc`,
   `BUILD_WITH_CUDA=ON`).
5. Build Iridescence (master) for visualisation (skipped on
   `skip-iridescence`).
6. Clone `src/third_party/glim` and `src/third_party/glim_ros2`, then run
   `colcon build --packages-select glim glim_ros --symlink-install`.
7. Source `install/setup.bash` and verify that `ros2 pkg list` lists
   `glim_ros`.

Build time estimate: 30–45 minutes end-to-end on the Alienware x15 R2
(i9-12900H, 14C/20T). GTSAM alone consumes 10–15 minutes.

### 2. Upstream version pinning

The pins are controlled by the variables at the top of the script:

| Variable | Value | Reason |
|---|---|---|
| `GTSAM_REF` | `4.3a0` | GLIM bumped this requirement on 2025-06-15; 4.2a9 no longer builds |
| `GTSAM_POINTS_REF` | `master` | Upstream does not cut tags. Replace with a commit SHA when bit-level reproducibility is needed |
| `IRIDESCENCE_REF` | `master` | same |
| `GLIM_REF` | `master` | same |
| `GLIM_ROS2_REF` | `master` | same |

Tracking `master` for everything except GTSAM is an upstream-driven
choice (koide3's projects do not publish release tags), not a preference
on our side. When the team needs to share "the same bits", open an issue
to bump the variables to SHAs.

### 3. Optional environment exports

GTSAM and gtsam_points install under `/usr/local/lib`. Ubuntu 22.04's
`ldconfig` already scans `/usr/local/lib` by default, so no extra
`LD_LIBRARY_PATH` is usually needed. Only if `glim_rosbag` complains that
it cannot find `libgtsam.so.4.3`, append:

```bash
export LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

The CUDA PATH export is assumed to already be in place from
[`m5r-cuda-setup.md`](m5r-cuda-setup.md) §3.

### 4. Fetch the sample bag

GLIM upstream distributes a verification bag captured with an Ouster
OS1-128.

```bash
mkdir -p /tmp/glim_sample && cd /tmp/glim_sample
# primary: zenodo (cited in the GLIM official quickstart, ROS 2 version, 426 MB)
curl -L --fail -o os1_128_01_downsampled.tar.gz \
  'https://zenodo.org/record/7233945/files/os1_128_01_downsampled.tar.gz?download=1'
tar -xzf os1_128_01_downsampled.tar.gz
# After extraction, /tmp/glim_sample/os1_128_01_downsampled/ contains
# metadata.yaml and the .db3 file.
```

The bag is about 426 MB (ROS 2 version). If the lab decides to mirror
this on the internal NAS, update this section. If zenodo is unreachable,
the official fallback mirror is
`https://staff.aist.go.jp/k.koide/projects/glim/datasets/os1_128_01_downsampled.tar.gz`
(note the `/datasets/` path segment — leaving it out yields a 404).

The sample bag is GLIM-verification-only and is not subject to the
`docs/maps/<site>/` convention (that convention is in M5R-7 scope). Keep
the storage path under `/tmp/`.

#### Sample-bag download status (2026-06-20, Issue #45 / PR #52 landing)

Neither mirror was usable from the lab host at install time. Logged for
posterity:

- **AIST mirror**: `HEAD` returns `Content-Length: 78524908` and the
  local download matches that exact size, but `gzip -t` fails with
  `unexpected end of file`. The gzip header advertises an original size
  of 2.5 GB (`modulo 2^32`), so AIST's stored archive is itself
  truncated (`Last-Modified: 2026-06-09`, stable).
- **zenodo**: 30-second probe averaged 36 KB/s, putting the full 426 MB
  download at ~3.4 hours. Feasible with `systemd-inhibit`, but the AC
  here is a smoke test and blocking the PR for three hours is poor
  cost-benefit.

Decision: **Issue #45 AC #4 (sample-bag smoke test) is rolled into
M5R-3 (#48, "GLIM vs FAST-LIO SAM real-bag comparison")**, where the
real indoor-loop bag exercises the same code path with stronger
verification. PR #52 lands with this caveat recorded. If a future
contributor needs to reproduce the smoke test, the remediation options
are: use the university's faster LAN/VPN to widen zenodo throughput,
switch to a GLIM github-releases mirror once upstream provides one, or
host a mirror on the lab NAS and replace the URL in this section.

### 5. Smoke test (through to trajectory output)

Run GLIM in rosbag input mode:

```bash
cd ~/whill_lab0_ros2
source install/setup.bash
mkdir -p /tmp/dump
ros2 run glim_ros glim_rosbag \
  /tmp/glim_sample/os1_128_01_downsampled \
  --ros-args \
    -p config_path:=$(ros2 pkg prefix glim_ros)/share/glim_ros/config/ \
    -p dump_path:=/tmp/dump/
```

An Iridescence OpenGL window opens and shows the cloud and trajectory
live. (If you built with `skip-iridescence`, it runs headless.) When
processing reaches the end of the bag, `/tmp/dump/traj_lidar.txt` is
written.

```bash
head -3 /tmp/dump/traj_lidar.txt
# Each line is: timestamp(ns) x y z qx qy qz qw
```

If the file is non-empty and covers the full timestamp range of the
sample bag, the smoke test passed.

## Troubleshooting

### GTSAM build fails on an Eigen version conflict

Symptom: it stops at a `static_assert(EIGEN_VERSION_AT_LEAST(3, 4, 0) ...)`.

Cause: Ubuntu 22.04's `libeigen3-dev` is 3.4.0 (which satisfies the
requirement), but a GTSAM submodule sometimes ships an older bundled
Eigen header.

Fix: verify that `GTSAM_USE_SYSTEM_EIGEN=ON` is in effect in the script.
If it still fails, delete `~/.cache/whill_lab0_ros2/glim/gtsam` and the
`build/` directory beneath it, then re-run.

If it still fails after that (e.g. an upstream GTSAM change in how it
consumes Eigen), flipping `GTSAM_USE_SYSTEM_EIGEN` to `OFF` so GTSAM uses
its bundled Eigen can work around it. The catch: gtsam_points and GLIM
were already built against the `ON` setting, so their Eigen ABI no
longer matches and they have to be rebuilt in lockstep:

1. Edit `~/.cache/whill_lab0_ros2/glim/gtsam/build/CMakeCache.txt` and
   change `GTSAM_USE_SYSTEM_EIGEN:BOOL=ON` to `OFF`.
2. Rebuild and reinstall GTSAM:
   `cmake --build ~/.cache/whill_lab0_ros2/glim/gtsam/build --parallel`
   then
   `sudo cmake --install ~/.cache/whill_lab0_ros2/glim/gtsam/build`.
3. Wipe the gtsam_points build cache with
   `rm -rf ~/.cache/whill_lab0_ros2/glim/gtsam_points/build` and re-run
   `./scripts/install_glim.sh`, which cascades the rebuild through
   gtsam_points and GLIM under the new Eigen setting.

### gtsam_points linker fails with `undefined reference to 'cuda'`

Symptom: `nvlink error: Undefined reference to ...` during link.

Cause: CMake may have picked up a different nvcc from `PATH`. The
`CMAKE_CUDA_COMPILER` argument did not take effect.

Fix:

```bash
# Clear the build cache and try again
rm -rf ~/.cache/whill_lab0_ros2/glim/gtsam_points/build
./scripts/install_glim.sh
```

If it still happens, confirm with `which -a nvcc` that no nvcc other
than `/usr/local/cuda-12.4/bin/nvcc` is on the path.

### glim_rosbag dies from VRAM exhaustion

Symptom: `cudaMalloc returned cudaErrorMemoryAllocation` or OOM-kill.

Cause: in GPU mode GLIM can exceed 16 GB VRAM on large bags. The host
RTX 3080 Laptop has 16 GB.

Fix:

1. Split the bag in time and feed it in pieces. (`ros2 bag info` shows
   the duration. `ros2 bag record --start-offset / --end-offset` is the
   wrong tool here; use an external bag editor.)
2. Switch GLIM to CPU mode by editing the glim_ros config (package name) (`config_path`
   → `config_sensors.json` etc.) and setting the `gpu` flag to `false`.
   CPU mode is 3–5× slower but unbounded by VRAM.
3. Rebuild with `skip-iridescence` to free the visualiser's VRAM.

### Iridescence window does not appear (remote SSH, etc.)

Without X11 forwarding, Iridescence dies in `glfwInit`. Either rebuild
with `./scripts/install_glim.sh skip-iridescence`, or set `DISPLAY` to a
working X server.

### GTSAM build OOMs

Symptom: `cc1plus: error: out of memory allocating ...`, with oom-killer
lines in `dmesg`.

Cause: a single cc1plus process for GTSAM template instantiations eats
2–4 GB. The script defaults to `nproc - 1` parallelism (13 on the
Alienware host), which can starve a 32 GB machine.

Fix: edit the `JOBS=` line in the script down to 4–8 and rerun.

## Related

- Strategy: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
  §3.3 (rationale for GLIM as M5-R first candidate), §3.4 (license
  policy), and §9 (development hardware check).
- Prerequisite: [`m5r-cuda-setup.md`](m5r-cuda-setup.md) — CUDA Toolkit
  12.4 and cuDNN 8 setup, the entry point to this document.
- ADR 0001: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md)
  — new documents are authored in parallel under `docs/ja/` and `docs/en/`.
- Script: [`scripts/install_glim.sh`](../../scripts/install_glim.sh) —
  the idempotent installer that this document is paired with.
- Sibling document: [`m5r-fastlio-sam-eval.md`](m5r-fastlio-sam-eval.md)
  — the clone-on-demand procedure and license discussion for the
  second-candidate FAST-LIO SAM (M5R-3 compares it against this
  document).
- Related issues: #23 (the CUDA document and script), #45 (this document
  and script), and the upcoming M5-R SLAM candidate comparison ADR (to
  be settled as ADR-0003 by empirical comparison of GLIM versus FAST-LIO
  SAM on real bags).
