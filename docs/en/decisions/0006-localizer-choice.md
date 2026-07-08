# ADR 0006: Runtime localizer choice (M6-R)

Language: [日本語](../../ja/decisions/0006-localizer-choice.md) | [English](0006-localizer-choice.md)

- Status: **proposed** (drafted at M6R-1 start; will be promoted to accepted at M6R-5 completion)
- Date: 2026-07-08
- Deciders: Iruazu (awaiting approval)

## Context

The parent strategy doc
([`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md))
§3.3 nominates `lidar_localization_ros2` as the first-choice scan-to-map
localizer for the operational (online, on-vehicle) stack. The M6-R execution
plan ([`../plans/2026-06-24-m6r-localization.md`](../plans/2026-06-24-m6r-localization.md))
§6 M6R-1 requires this ADR to record the pinned commit SHA and the IMU
wiring decision after live bag-replay verification.

This ADR is drafted **proposed** at M6R-1 start and promoted to **accepted**
when M6R-5 acceptance (G1-G3) passes. In the interim, switching to another
candidate (`hdl_localization`, Autoware `ndt_scan_matcher`, …) requires
revising this ADR from scratch.

## Decision

1. **Adopt `rsasaki0109/lidar_localization_ros2`** (upstream fork).
   - License: BSD-2-Clause (satisfies parent §3.4 permissive requirement)
   - Registration: NDT_OMP (depends on the same fork's `ndt_omp_ros2`)
   - Emits: `map -> odom` TF, `/pcl_pose`
     (`geometry_msgs/PoseStamped`), `/alignment_status`
     (`diagnostic_msgs/DiagnosticArray`)
2. **VCS pin (proposed stage)**: `whill_lab.repos` uses `version: main`
   (HEAD as of 2026-07-08). Once M6R-1 smoke test completes, **pin to the
   exact commit SHA** (matches the tag-pin pattern used by ADR-0003 / GLIM).
3. **Dependency**: `ndt_omp_ros2` (rsasaki0109 fork, humble branch) added
   to `whill_lab.repos` alongside the localizer. Single `vcs import`
   remains sufficient.
4. **IMU wiring default = `use_imu: false`**:
   - Verified by reading `lidar_localization_component.cpp` at v1.1.0
     (default is false).
   - When `use_imu: true`, the IMU feeds scan undistortion, **not** EKF
     prediction.
   - For this phase the EKF (M4-R) consumes `/imu/data_rep145` and supplies
     `odom -> base_link`; the localizer independently supplies `map -> odom`.
     The IMU path stays separate.
   - If scan undistortion becomes necessary later (fast turns, etc.),
     switch `use_imu: true` and remap the `imu` topic to
     `/imu/data_rep145` (**not the raw `/imu/data_raw`** — PR #56 flipped
     the acceleration sign to REP-145; the raw stream would corrupt
     undistortion).
5. **Map input path**: `docs/maps/<site>/static.pcd` (per ADR-0005).
   Passed via the `map_path` parameter.
6. **Initial NDT parameters** (cloned from
   `param/boreas_ndt_velodyne.yaml`, materialised per-run inside
   `scripts/m6r_smoke_test.sh`):
   - `ndt_resolution: 1.0` m
   - `ndt_step_size: 0.1`
   - `ndt_max_iterations: 25`
   - `transform_epsilon: 0.01`
   - `voxel_leaf_size: 0.5` m (down from Boreas 128's 1.5 to suit
     VLP-16's sparser cloud)
   - `score_threshold: 6.0`
   - `scan_max_range: 80.0`, `scan_min_range: 1.0` m
7. **map -> odom TF**: `enable_map_odom_tf: true` explicit. Assumes the
   REP-105 continuous `odom -> base_link` from the M4-R EKF is connected
   downstream.

## Alternatives rejected

- **`koide3/hdl_localization`** (BSD-2-Clause):
  - Proven; kept as an M9 fallback but not first choice.
  - Why rejected: NDT_OMP + KDTree switchable is a plus, but the parent
    §3.3 "Tsukuba 2024 winners with odometry-constrained localization"
    evidence points at `lidar_localization_ros2`. Rejection is purely
    on track record, not technical grounds.
- **Autoware `ndt_scan_matcher`** (Apache 2.0):
  - High quality. Rejected because it drags in the entire `autoware_*`
    stack — overkill for a single-package deployment in M6-R.
- **`FAST_LIO_LOCALIZATION` family** (GPL):
  - GPL propagation is a concern per parent §3.4. Rejected.
- **In-house NDT** (from scratch):
  - No engineering ROI — the upstream is sufficient. Rejected.
- **Enable IMU by default** (`use_imu: true`):
  - Scan-undistortion benefit is likely negligible at WHILL's speed range
    (up to 1.7 m/s max). It also adds the cost of verifying the
    `/imu/data_rep145` REP-145 sign is correct on the localizer side. If
    M6R-1 smoke passes with `use_imu: false`, no need to enable.

## Consequences

Positive:
- Single `vcs import` + `colcon build` reproduces the environment. Build
  wall time observed: 2 min 20 s (`ndt_omp_ros2` 25 s +
  `lidar_localization_ros2` 115 s) on Alienware x15 R2 (i9-12900H).
- `use_imu: false` cleanly separates M4-R EKF and the localizer's
  responsibilities.
- Meshes well with the `docs/maps/<site>/` convention (ADR-0005).
- BSD-2-Clause satisfies the parent §3.4 permissive requirement.

Negative / TBD:
- **[Resolved at M6R-1] Upstream `main` HEAD drift**: current pin uses
  `version: main`, which is unstable. M6R-1 will replace it with a
  concrete commit SHA.
- **[Input to M6R-3] `/alignment_status` field schema unknown**: not
  documented in the upstream README. Captured by `ros2 topic echo
  /alignment_status --once` at M6R-1 and logged in
  `docs/ja/m6r-localizer-eval.md`.
- **[Resolved at M6R-1] GLIM PCD voxel × NDT resolution compatibility**:
  today's `campus-half-v3` PCD carries 66 m of drift. Whether a
  `voxel_leaf_size: 0.5 m` × `ndt_resolution: 1.0 m` combo can lock on it
  is only knowable via smoke test.
- **[Post-demo] `use_imu: true` unevaluated**: scan-undistortion benefit
  at high yaw rates is outside this ADR's scope. Post-demo may add a
  separate ADR.
- **[Depends on M6R-3] Failure threshold**: `score_threshold: 6.0` comes
  straight from the Boreas preset. The real threshold for
  WHILL / VLP-16 / outdoor campus is tuned across M6R-1..M6R-3; the
  confirmed value lands in "Decision 6" here.

## M6R-5 accepted-promotion criteria

Promote to **accepted** when all four hold:

- [ ] M6R-1 smoke test shows `map -> odom` TF continuously published
  (parent §6 M6R-1)
- [ ] M6R-5 hardware trials pass G1-G3 (plan §5)
- [ ] Confirmed commit SHA pinned in `whill_lab.repos`
- [ ] `/alignment_status` schema and measured NDT failure threshold
  recorded in "Decision 6" above
