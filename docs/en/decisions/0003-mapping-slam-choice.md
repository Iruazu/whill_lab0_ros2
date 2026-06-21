# ADR 0003: M5-R map-building SLAM final choice

Language: [日本語](../../ja/decisions/0003-mapping-slam-choice.md) | [English](0003-mapping-slam-choice.md)

- Status: accepted
- Date: 2026-06-22 (Phase A skeleton) / 2026-06-21 (Phase B measurement complete, Decision filled, accepted)
- Deciders: Iruazu

## Context

The platform-pivot plan [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
§7 (ADR candidates) states:

> ADR: final choice of map-building SLAM. The GPU host is secured
> (§9), so GLIM's prerequisite is met. Settle the choice after the
> empirical GLIM-vs-FAST-LIO-SAM comparison on a real bag.

§3.3 (candidate table) lists GLIM (first candidate, MIT, ROS 2 humble
upstream, GPU host post-processing) and FAST-LIO SAM (alternative,
VLP-16 track record). §3.4 (license policy) states "keep the
operational stack permissive (MIT/BSD/Apache)" and "GPL-family code
(FAST-LIO and friends) is restricted to use as an offline
map-building tool in a separated process".

The M5-R execution plan
[`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
§6 acceptance B4 requires "ADR-0003 merged in `accepted` state with
real-bag comparison as evidence" and B5 requires a license inventory.
This ADR satisfies both.

### State of the SLAM candidates

| SLAM | Upstream | License | Repo integration |
|---|---|---|---|
| GLIM | [`koide3/glim`](https://github.com/koide3/glim) + [`koide3/glim_ros2`](https://github.com/koide3/glim_ros2) | MIT | M5R-1 (#45) source-built on the host, CUDA 12.4 + cuDNN 8. See [`../m5r-glim-setup.md`](../m5r-glim-setup.md). |
| FAST-LIO SAM | [`RightTr/FAST-LIO-SAM`](https://github.com/RightTr/FAST-LIO-SAM) | **No LICENSE file** in upstream; `package.xml` claims `BSD` unilaterally. Origin is FAST-LIO (HKU-MaRS) which is **GPL-2.0**, so copyleft propagation is possible. | M5R-2 (#46) put the clone-on-demand path in place behind `FASTLIO_SAM_LICENSE_ACK=yes`. See [`../m5r-fastlio-sam-eval.md`](../m5r-fastlio-sam-eval.md). |

### Evaluation conditions

- Input bag: same ~50 m indoor loop bag, recorded with the M4-R
  bringup launch capturing `/velodyne_points`, `/imu/data_raw`,
  `/tf_static`.
- Measurement wrappers: `scripts/m5r3_run_glim.sh` and
  `scripts/m5r3_run_fastlio_sam.sh`. Both produce a `manifest.yaml`
  with wall time, peak VRAM, and run logs.
- Loop error:
  - Formal metric (B1): mean distance between three wall points
    picked in CloudCompare at start and end of the loop, target
    ≤ 0.5 m.
  - Complementary: `scripts/m5r3_loop_error.py` computes the TUM
    trajectory's first-vs-last pose distance, capturing the SLAM's
    self-reported internal pose-graph closure.
- Operability: observe Iridescence (GLIM) / RViz (FAST-LIO SAM) for
  manual relocalization need, keyframe density, loop-closure trigger
  frame.
- GTSAM coexistence: snapshot `ldconfig -p | grep libgtsam` to
  `gtsam_env.log` to capture the 4.3a0 (GLIM) vs 4.1.1 (FAST-LIO SAM)
  resolution state at run time.

The full procedure is in [`../m5r3-comparison-protocol.md`](../m5r3-comparison-protocol.md).

### Phase structure

Two phases complete this ADR:

1. **Phase A (this commit)**: skeleton. Put the measurement wrappers,
   protocol doc, and ADR structure in place. The Decision section is
   a placeholder.
2. **Phase B (follow-up commit, after user-side bag work)**: capture
   real bags, run both SLAMs, transcribe numbers and qualitative
   notes into the Alternatives table and Consequences, fill in the
   Decision, and move the PR to ready-for-review. A final commit
   moves Status from `proposed` to `accepted` after approval.

## Decision

**Adopted SLAM: GLIM** (`koide3/glim` + `koide3/glim_ros2`, MIT).

**Commit SHA / pin** (transcribed from
`docs/m5r-bench-data/2026-06-21-loop-outdoor/glim-out/manifest.yaml`
at Phase B run time):
- This repo `git_commit`: `48b746a` (the M5R-3 branch tip that
  includes the bag-rewrite script)
- Upstream (source-built via `install_glim.sh`): the version pinned in
  M5R-1 (#45). Stack is CUDA 12.4 + cuDNN 8 + GTSAM 4.3a0 (with
  UNSTABLE).
- Sensor config: the per-run config copy mechanism in
  `scripts/m5r3_run_glim.sh` rewrites `config_sensors.json`
  `T_lidar_imu` to the SE3 inverse of the M4R-2 measured extrinsic
  and `ring_field` to `"ring"` for VLP-16.

**Rationale summary**:

1. **Ran vs. did-not-run** (heaviest axis): on a Velodyne VLP-16 +
   PCMK-G3X (MPU-9250) bag, GLIM produced `traj_lidar.txt` with 1954
   samples, `graph.bin`, and 17 submaps, and exited cleanly. FAST-LIO
   SAM's upstream ROS2 port has no Velodyne mapping launch; even with
   a Velodyne launch + config written from scratch in this repo, the
   SLAM silent-crashes immediately after the first frame (DNF).
   Details below in Alternatives considered.
2. **License**: GLIM = MIT, permissive. No link constraint on the
   future operational stack. FAST-LIO SAM = no LICENSE file +
   GPL-2.0 propagation risk from its FAST-LIO origin. Consistent with
   the platform-pivot §3.4 license policy.
3. **Loop error** (complementary metric; the formal B1 wall-3-point
   needed both SLAMs to produce a PCD): GLIM end-to-start drift =
   0.838 m over a 52.640 m loop (~1.6%). FAST-LIO SAM produced no
   trajectory, so no direct comparison was possible.
4. **LiDAR class fit** (rig-specific judgement): VLP-16 is a
   mid-tier LiDAR (16 beams; weak in outdoor feature-poor stretches).
   FAST-LIO's strengths show up best on high-density sensors
   (OS1-128, Livox MID-360 etc.) familiar to the HKU-MaRS origin
   stack. With VLP-16, where GLIM ran cleanly, **the engineering
   cost of getting FAST-LIO SAM running (chasing the upstream
   ROOT_DIR bug, debugging the preprocess crash, and likely more
   latent issues) is better redirected to the remaining M5-R phases
   — ERASOR, occupancy-grid conversion, pipeline integration —
   where the ROI is higher**.
5. **GTSAM coexistence**: gtsam_env.log confirms 4.3a0 (in
   `/usr/local`) and 4.1.1 (in `/usr/lib`) coexist under ldconfig
   without surfacing a conflict at this run. Going GLIM-only removes
   the coexistence concern altogether.

## Alternatives considered

### FAST-LIO SAM (rejected)

| Axis | GLIM (adopted) | FAST-LIO SAM (rejected) |
|---|---|---|
| Wall time (s) | 575 (~2.9x of the 199 s bag) | 589 (of which actual SLAM work was ~0.14 s before the crash; the rest is wrapper waiting for a dead process) |
| Peak VRAM (MiB) | 545 | 15 (baseline; SLAM never claimed GPU memory) |
| Peak RSS (KiB) | n/a (not in the GLIM wrapper's schema) | 0 (RSS poller never caught the pid alive; it died too fast) |
| Internal trajectory error (m) | **0.838** (52.640 m loop, ~1.6%, via `m5r3_loop_error.py`) | dnf (no trajectory file written) |
| B1 formal error (wall 3-point mean, m) | TBD (CloudCompare measurement not performed yet. PCDs exist on the GLIM side as 17 submaps but the metric is one-sided so deferred) | dnf (no PCD written) |
| Loop closure trigger timing | No explicit entry in run.log. Expected for an outdoor straight-and-back geometry (50 m straight + 180° U-turn) which struggles to satisfy closure detection criteria | dnf |
| Keyframe density (per m) | 1954 samples / 52.640 m = 37 sample/m; 17 submaps | dnf |
| Manual relocalization needed? | No (M4R-2 extrinsic baked in, bag rewrite normalizes IMU sign, 5 s stationary start lets grav_align converge) | n/a |
| GTSAM resolution state | 4.3a0 alone (`/usr/local/lib`, source-built with GLIM) | gtsam_env.log shows 4.3a0 (`/usr/local`) + 4.1.1 (`/usr/lib`) coexisting in ldconfig. No conflict materialised at this run, but the latent risk remains |
| License | MIT (permissive) | No upstream LICENSE + the FAST-LIO origin is GPL-2.0. `package.xml` self-declares "BSD" but that doesn't match reality |
| Build success | OK (M5R-1 #45 source build) | Package build OK, but the ROS2 port ships no Velodyne mapping launch (only `airy` / `l2` / `mid360`). We wrote `scripts/m5r3_mapping_velodyne_for_fastlio_sam.launch.py` + `scripts/m5r3_fastlio_sam_velodyne_config.yaml` in-repo to get initialisation through; even so, after the preprocess `[WARN] No point, skip this scan!` the process silent-crashes (DNF) |

### Supplementary notes

- **What was the deciding factor on the FAST-LIO SAM side**: the
  upstream ROS2 port not supporting Velodyne natively is the biggest
  one. `config/odom/velodyne.yaml` (odometry-only) exists, but there
  is no velodyne yaml under `config/mapping/` and no Velodyne launch
  under `launch_ROS2/mapping/`. This ADR's scope authored a minimal
  launch + config (commit 4af5ffa); even then, an upstream
  path-construction warning appears at startup (`~~~~<repo>/src/
  third_party/FAST_LIO_SAM/ doesn't exist` — the directory does
  exist, so this is a realpath / trailing-slash bug), and preprocess
  silent-crashes on the first frame. Pinning the crash root cause
  requires upstream-side debugging, which is out of scope for M5R-3.
  Details in
  `docs/m5r-bench-data/2026-06-21-loop-outdoor/fastlio-sam-out/manifest.yaml`.
- **Symmetry caveats**: GLIM's per-run config copy bakes in
  `T_lidar_imu` (SE3 inverse of the M4R-2 measured extrinsic) and
  `ring_field=ring` (VLP-16). FAST-LIO SAM had the same extrinsic in
  its yaml but never reached the phase where extrinsic matters
  because of the crash. Both SLAMs took
  `bag-imu-fixed/` as input (the bag rewritten by
  `scripts/m5r3_fix_imu_bag.py` to normalise PCMK-G3X firmware's
  gravity-vector accel output to REP-145 specific force). The rewrite
  is required for GLIM; FAST-LIO would tolerate either sign via its
  self-estimation, but the same input keeps the comparison
  apples-to-apples.
- **Why li_slam_ros2 stays out of scope**: the platform-pivot §3.3
  defines it as "comparison / fallback" only. This ADR's contest is
  GLIM vs FAST-LIO SAM, and rejecting FAST-LIO SAM uniquely points
  at GLIM (the remaining candidate that ticks Velodyne support +
  permissive + verified to run). No need to re-open li_slam_ros2.

## Consequences

### License inventory (B5 acceptance)

The adopted SLAM is GLIM:

- **GLIM**: MIT, permissive. No link constraint on the operational
  stack. M5-R is the "offline map-building tool" phase
  (platform-pivot §3.1); the runtime localizer belongs to M6-R
  (scan-to-map localizer, separately chosen). Whether to also wire
  GLIM into the operational stack is out of scope here and depends
  on M6-R's evaluation.
- **GTSAM 4.3a0 (BSD-3-Clause, with UNSTABLE) — GLIM's main dep**:
  source-built under `/usr/local/lib` via `install_glim.sh`.
  Permissive, no distribution constraint.
- **FAST-LIO SAM artefacts still in this repo**:
  `src/third_party/FAST_LIO_SAM/` is `.gitignore`d
  (clone-on-demand), nothing of it is redistributed.
  `scripts/m5r3_run_fastlio_sam.sh` +
  `scripts/m5r3_mapping_velodyne_for_fastlio_sam.launch.py` +
  `scripts/m5r3_fastlio_sam_velodyne_config.yaml` are this repo's
  own evaluation code (BSD-3-Clause), not upstream redistribution.
  If upstream later adds a permissive LICENSE, re-evaluate in a
  separate ADR.

### CPU / GPU / memory profile (measured)

Host: Alienware x15 R2 (i9-12900H 32 GiB RAM, RTX 3080 Laptop GPU
16 GB VRAM). GLIM Phase B run
(`docs/m5r-bench-data/2026-06-21-loop-outdoor/glim-out/manifest.yaml`):

- Wall time: 575 s (~2.9x of the 199 s bag)
- Peak VRAM: 545 MiB
- Peak RSS: not measured (the GLIM wrapper does not collect RSS; a
  candidate improvement for the wrapper)
- Average playback speed: 0.35x realtime (1.9–6.6x during startup
  but slows below realtime as submaps accumulate. For an outdoor
  mid-density LiDAR + 50 m loop, this is acceptable on the host;
  whether a vehicle-borne machine can hit 1x realtime is a future
  M9 question)

The vehicle-side move is out of scope here, re-evaluated in M9.
On the host machine (16 GB GPU + 32 GiB RAM) there is plenty of
headroom.

### Impact on downstream phases (M6-R)

- The M6-R scan-to-map localizer ingests GLIM's static PCD via the
  `docs/maps/<site>/static.pcd` convention (ADR-0005).
- PCD format: GLIM with `dump_path` writes per-submap point clouds
  and poses into `000000/` … `000017/`. **Producing a single global
  static PCD requires merging via the upstream `glim_offline` tool
  (or equivalent)**, slated for the M5R-4 ERASOR input stage.
- Coordinate frame: GLIM's
  `auto-detected IMU frame ID: imu_link` /
  `auto-detected LiDAR frame ID: velodyne` worked, and trajectories
  ship in both LiDAR frame (`traj_lidar.txt`) and IMU frame
  (`traj_imu.txt`).
- Frame integrity: the M4-R bringup `/tf_static` carries
  `base_link → velodyne` into the bag, so GLIM can also publish TF
  in `base_link` frame if needed (this evaluation used IMU frame as
  base).

### IMU sign convention spillover (finding, candidate follow-up Issue)

Surfaced during Phase B: PCMK-G3X (MPU-9250 + LPC1343F USB firmware)
outputs `linear_acceleration` as the gravity-acceleration vector,
not REP-145 specific force (measured `linear_acceleration.z = -9.71`
at rest). GLIM requires an explicit fix; FAST-LIO family absorbs it
internally (`IMU_Processing.hpp:196`'s
`init_state.grav = S2(-mean_acc / |mean_acc| * G)`). For this ADR,
`scripts/m5r3_fix_imu_bag.py` rewrites the bag as the minimum
M5R-3-scope workaround. A permanent fix — a republisher in the
sensor bringup layer so every downstream consumer receives
REP-145-compliant IMU — is out of scope here, tracked in a new Issue.
Whether the EKF (M4R-3) was silently miscalibrated is part of that
investigation.

### Follow-ups

- **M5R-4 (#49) ERASOR**: ingests GLIM's submap PCDs + per-frame
  poses, removes dynamic objects. Unblocked by this ADR's
  acceptance.
- **M5R-6 (#50) Occupancy grid conversion**: turns the GLIM →
  ERASOR static PCD into a 2D occupancy grid at
  `docs/maps/<site>/occupancy.{pgm,yaml}`.
- **M5R-7 (#51) Pipeline integration**: documents the E2E flow
  bag → GLIM → ERASOR → occupancy grid → `docs/maps/<site>/`.
- **IMU sign permanent-fix Issue (spilled from the "IMU sign
  convention spillover" section)**: add a
  `/imu/data_raw` → `/imu/data_corrected` REP-145-conformant
  republisher to `whill_sensors_bringup/`. Re-record bags after the
  fix lands; verify EKF behavior.

## Related

- Policy document: [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md)
  §3.3 (candidate table), §3.4 (license policy), §7 (this ADR
  requested)
- M5-R execution plan: [`../plans/2026-06-21-m5r-execution.md`](../plans/2026-06-21-m5r-execution.md)
  §M5R-3 (this Issue), §6 (acceptance B1–B5)
- Comparison protocol: [`../m5r3-comparison-protocol.md`](../m5r3-comparison-protocol.md)
  — Phase B runbook
- Predecessor ADR: [`0005-maps-spec.md`](0005-maps-spec.md) — the
  `docs/maps/<site>/` convention into which the adopted SLAM's PCD
  ultimately flows
- Predecessor docs: [`../m5r-glim-setup.md`](../m5r-glim-setup.md)
  (GLIM source build), [`../m5r-fastlio-sam-eval.md`](../m5r-fastlio-sam-eval.md)
  (FAST-LIO SAM clone-on-demand)
- Scripts: [`../../../scripts/m5r3_run_glim.sh`](../../../scripts/m5r3_run_glim.sh),
  [`../../../scripts/m5r3_run_fastlio_sam.sh`](../../../scripts/m5r3_run_fastlio_sam.sh),
  [`../../../scripts/m5r3_loop_error.py`](../../../scripts/m5r3_loop_error.py)
- Related issues: #48 (this Issue, M5R-3), #45 (M5R-1 GLIM), #46
  (M5R-2 FAST-LIO SAM), #47 (M5R-5 maps convention), #49 (M5R-4
  ERASOR), #50 (M5R-6 occupancy), #51 (M5R-7 integration)
