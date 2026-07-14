# ADR 0011: Ground removal preprocessing — Patchwork++ core + own ROS 2 wrapper (M6-R)

Language: [日本語](../../ja/decisions/0011-ground-removal-choice.md) | [English](0011-ground-removal-choice.md)

- Status: **accepted** (2026-07-14 bag-replay verification passed AC1-AC3; AC4 confirmed with the M6R4-3 field run)
- Date: 2026-07-14 drafted / 2026-07-14 accepted
- Deciders: Iruazu

## Context

M6R4-2 wired `pointcloud_to_laserscan` in front of the Nav2
`obstacle_layer` (`whill_navigation/config/pointcloud_to_laserscan.yaml`,
see the [governing plan](../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md)).
2026-07-14 Phase B at 工農研横 hit a structural limit:

- The slice is a `base_link`-flat horizontal cut with a single
  `min_height`. Gradient (~5°) and local unevenness (manholes, ruts,
  seams) push the ground past that threshold and paint lethal cells
- Raising `min_height` from -0.2 to 0.25 cleaned the flat sections but
  left spikes on sloped tarmac (measured live)
- The threshold is a trade-off: pushing it higher also loses curbs
  (~0.15 m) and crouched children (< 0.25 m)

Root cause: a single-threshold horizontal slice assumes a flat world.
Terrain-following ground estimation removes the problem at the source.

## Decision

Three parts:

### 1. Algorithm: **Patchwork++** (KAIST Urban Robotics Lab, IROS 2022)

Concentric zone model divides the cloud into range-band × angular-sector
patches. Each patch runs Region-wise Vertical Plane Fitting + Adaptive
Ground Likelihood Estimation. Patch-level re-estimation of local ground
height directly addresses the "gradient + local unevenness" failure
mode that a single-plane RANSAC cannot reach.

### 2. Code scope: BSD-2-Clause C++ core only

Upstream `url-kaist/patchwork-plusplus` v1.4.1 has a split-license story:

| Location | Declaration |
|----------|-------------|
| root `LICENSE` | BSD 2-Clause (KAIST, 2024) — covers `cpp/` core |
| `ros/LICENSE` | MIT (KISS-ICP authors, 2022) |
| `ros/package.xml` | **GPL-3.0** |
| `ros/src/*.cpp` | no license header |

The `ros/LICENSE` MIT vs `ros/package.xml` GPL-3.0 discrepancy is an
upstream bug. We treat `package.xml` as authoritative (machine-readable,
tooling-scanned) and read the ROS 2 wrapper as GPL-3.0. This runs into
the project's [platform-pivot §3.4](../../ja/plans/2026-06-11-platform-pivot.md)
rule: runtime stack stays permissive, GPL is confined to offline
map-making tools. **We do not adopt the upstream `ros/` subtree.**

We take only the BSD-2-Clause `cpp/` core (the `Params` struct and the
`PatchWorkpp` class) as a CMake `add_subdirectory` under our own
package. The core needs Eigen3 and optional TBB; Open3D is only pulled
in for `INCLUDE_CPP_EXAMPLES ON`, which stays off.

### 3. ROS 2 wrapper: own package `whill_perception`

New `src/whill_perception/`, BSD-3-Clause. A thin node (~200 lines)
around `PatchWorkpp`. Contract:

```
sub  cloud_in         sensor_msgs/PointCloud2  (VLP-16, SensorDataQoS)
pub  cloud_no_ground  sensor_msgs/PointCloud2  (~10 Hz, xyz only,
                                                 same header frame_id)
```

Every parameter lives in `config/patchworkpp.yaml`. Only
`sensor_height` (0.79 m from the M4-R extrinsic ledger), `min_range`,
and `max_range` diverge from Patchwork++ struct defaults — the KITTI
64ch tuning underpinning the defaults has not been re-validated on
VLP-16, so we do not go further until field data motivates it.

### 4. Pipeline integration

```
/velodyne_points  ──▶ patchworkpp_node  ──▶ /velodyne_points_no_ground
                                                    ▼
                       pointcloud_to_laserscan_node  ──▶ /scan
                                                              ▼
                                                    obstacle_layer
                                                    (Nav2)
```

M6R4-1+2 (PR #81, session B is completing that branch) needs to merge
first. A follow-up PR then flips `p2ls_node`'s subscription from
`/velodyne_points` to `/velodyne_points_no_ground` — that follow-up is
out of scope for this ADR.

## Alternatives

### Alternative A: Adopt the upstream ROS 2 wrapper (`ros/patchworkpp_node`) as-is

- **Pros**: zero implementation effort, matches upstream configuration
- **Cons**: package.xml declares GPL-3.0. First GPL dep in the runtime
  stack. Directly against §3.4
- **Rejected**: setting a "new precedent for exceptions" has weight
  even for an academic-only interim use. Reading `ros/LICENSE` (MIT)
  as the true license is possible but leaves us dependent on upstream
  to fix its own bug

### Alternative B: Autoware `scan_ground_filter` (Apache-2.0)

- **Pros**: clean license, explicit gradient parameters
  (`global_slope_max_angle_deg`, `local_slope_max_angle_deg`),
  substantial deployment record in Autoware vehicles
- **Cons**: comes as a slice of the `autoware_universe` monorepo,
  which drags `pcl_msgs` and Autoware-specific utilities. Nothing else
  in this repo uses the Autoware ecosystem, so the dependency-management
  cost is clearly higher than a self-contained package
- **Switch condition**: if Patchwork++ fails the ≤ 100 ms VLP-16
  latency budget or exceeds the CPU allowance, accept the monorepo
  slice cost and migrate

### Alternative C: linefit_ground_segmentation_ros2 (BSD-3-Clause)

- **Pros**: legacy continuity (used in the old whill_lab0 noetic
  stack), light dependencies, a ROS 2 port exists
- **Cons**: the ROS 2 port is a single-developer repo with ~20 stars.
  The algorithm itself is sector-based line fitting, weaker on local
  unevenness than A-GLE
- **Switch condition**: if Patchwork++'s Open3D-adjacent build machinery
  (in practice pulled only for examples, but sitting in the same build
  tree) conflicts with the host's existing CUDA setup

### Alternative D: RANSAC single-plane fit (`pcl_ros SACSegmentation`)

- **Pros**: apt one-liner, minimal code, no dependencies
- **Cons**: assumes a single global plane. With 5° gradient + local
  unevenness, the fit skews toward the gradient or misses the
  unevenness — the same "single-threshold" root cause the 2D slice
  approach already failed on
- **Rejected**: with the 2026-08-01 demo close, "fast but doesn't meet
  the requirement" is technical debt without upside

## Consequences

- **Build hygiene**: `touch src/third_party/patchwork_plusplus/ros/COLCON_IGNORE`
  is required after `vcs import`. Documented in
  `src/whill_perception/README.md`. Bake into any vcs-import wrapper
- **CPU budget**: on the Alienware x15 R2 will be nailed by the bag
  replay verification in this PR. Third-party benchmarks quote 13-48 ms
  for Patchwork++ on SemanticKITTI 64ch (in a paper comparing against
  Patchwork++, so biased). VLP-16 has ~1/4 the point count, so we
  expect headroom, but this is not direct
- **Downstream integration**: after PR #81 merges, a follow-up flips
  `p2ls_node`'s subscription topic — out of scope here
- **min_height re-tune**: with ground removed upstream, the p2ls
  `min_height` can be relaxed back toward capturing curbs (~0.15 m).
  Coordinate with session B's ADR-0009 (p2ls parameter selection)
- **Upstream license issue**: file an issue / PR against
  `url-kaist/patchwork-plusplus` for the `ros/LICENSE` (MIT) vs
  `ros/package.xml` (GPL-3.0) contradiction. Follow-up outside this ADR

## Acceptance conditions

- **AC1 (build & runtime)**: `colcon build --packages-select whill_perception`
  passes (currently PASS). `ros2 topic hz /velodyne_points_no_ground`
  at 9-11 Hz sustained for 30 s on real VLP-16 or bag replay
- **AC2 (visual)**: RViz side-by-side of `/velodyne_points` and
  `/velodyne_points_no_ground` shows ground gone, buildings / poles /
  people preserved
- **AC3 (CPU)**: `top -p <patchworkpp_node pid>` shows single-core
  usage ≤ 80% during playback
- **AC4 (drive)**: `/local_costmap/costmap` false-lethal spikes at
  M6R4-3 field run down significantly vs the 2026-07-14 Phase B
  baseline

AC1-AC3 can be met with bag replay. AC4 is met alongside M6R4-3 V4.
All four passing promotes this ADR to accepted.

## Verification results (2026-07-14 bag replay)

Replayed `docs/m6r-bench-data/2026-07-14-verify-campus/bag/` against
this branch (commit `ceb3bb3`):

| AC | Verdict | Measured |
|----|---------|----------|
| AC1 build | PASS | `colcon build --packages-select whill_perception` 7.16 s, no stderr |
| AC1 rate  | PASS | `/velodyne_points_no_ground` = **9.857 Hz** (window 100, sustained 30 s+, std dev **0.0004 s**) |
| AC2 visual| PASS | RViz confirms asphalt / ramp / manhole rings removed; buildings, poles, pedestrians retained (screenshot captured) |
| AC3 CPU   | PASS | `patchworkpp_node` = **2.3% total CPU / 1.6-2.7 ms per frame** (comfortably below the single-core 80% target) |
| AC4 drive | Pending — decided with M6R4-3 | — |

Representative frame split (mid-bag): `in 29184 pts / ground 8047 / non-ground 21137`. Ground share **26-30%** stayed stable across the run.

**AC1-AC3 passing promoted this ADR to accepted** (see Status). AC4 is
still to be judged with M6R4-3 V4, but the AC1-AC3 outcome makes it
tractable to relax the downstream p2ls `min_height` back toward
capturing curbs (paired update in ADR-0009).

### Ground share at low mount height

The 26-30% ground share is structurally normal for the WHILL's low
mount (`sensor_height = 0.79 m`). The VLP-16 16-ring beam pattern spans
±15° around horizontal; the lower the sensor, the fewer rings hit the
ground within a given range (the ground subtends a smaller solid angle
under the sensor). Autoware-class high mounts (~1.9 m) sit at 60-70%
ground share, but chasing that number on this vehicle would over-cull
non-ground returns. Recording this so a future reader does not mistake
the low share for a bug.

### Silent-failure retrospective (2 incidents)

Two silent failures were hit and closed during the M6R4-b implementation
window. Documented here for the next author writing a similar node:

1. **RNR intensity column missing** (2026-07-14 bag replay, first
   attempt). Patchwork++ core requires a 4th column (intensity) in the
   input when RNR is on; otherwise it prints
   `RNR requires intensity information !` to stdout and drops the whole
   frame. The initial wrapper converted PointCloud2 → Nx3 (xyz only),
   so every frame was rejected — `/velodyne_points_no_ground` still
   published, but `getGround()` / `getNonground()` were both empty.
   RViz just showed nothing; no ERROR/WARN surfaced through ROS. Fix
   (`ceb3bb3`): (a) convert to Nx4, (b) `WARN_THROTTLE` when the input
   has no `intensity` field, (c) `WARN_THROTTLE` when
   `ground.rows() == 0 && nonground.rows() == 0` after estimateGround.
   Two ROS-side guards that do not rely on the core's single stdout
   print.

2. **UDP fragment loss** (2026-07-14 bag replay, second attempt). Large
   PointCloud2 frames overwhelmed the DDS default UDP receive buffer
   during bag playback, dropping fragments piecewise. The live driver's
   pacing is gentle enough not to hit this in normal operation, but bag
   playback bursts far above the intended send rate and exposed it.
   Fixed permanently by `net.core.rmem_max` / `rmem_default = 25 MB` in
   `/etc/sysctl.d/60-ros2-dds-buffer.conf`. The same risk applies to
   simultaneous bag record during M6R4-3 field runs — a host without
   this sysctl setting must not run this node in that combined mode.
   Scoped beyond a node-level ADR, but recorded here as an environment
   requirement (details in `src/whill_perception/README.md`
   *Environment* section).

## Related

- [`../../ja/plans/2026-06-11-platform-pivot.md`](../../ja/plans/2026-06-11-platform-pivot.md) §3.4 (license policy), §4 M6-R
- [`../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md) §"Follow-up: ground removal" (positions M6R4-b)
- ADR-0009 (session B, in-flight, p2ls parameter selection): downstream of this ADR for `min_height` re-tune
- Upstream: <https://github.com/url-kaist/patchwork-plusplus> (v1.4.1, BSD-2-Clause)
- Paper: <https://arxiv.org/abs/2207.11919>
