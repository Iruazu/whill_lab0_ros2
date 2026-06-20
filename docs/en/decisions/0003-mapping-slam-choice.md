# ADR 0003: M5-R map-building SLAM final choice

Language: [日本語](../../ja/decisions/0003-mapping-slam-choice.md) | [English](0003-mapping-slam-choice.md)

- Status: proposed (moves to accepted after Phase B data collection and user approval)
- Date: 2026-06-22
- Deciders: Iruazu (pending review after Phase B)

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

```
(PLACEHOLDER) Filled in after Phase B.

Adopted SLAM: TBD (GLIM | FAST_LIO_SAM)
Commit SHA / tag pin: TBD (transcribe the adopted SLAM's
                           manifest.yaml git_commit and upstream
                           upstream_commit fields)
Rationale summary: TBD (compound judgement across the four axes:
                        B1 error, loop closure behaviour, license,
                        operability)
```

This section is filled by the Phase B evaluator in a follow-up
commit, and Status moves to `accepted` after user approval.

## Alternatives considered

After Phase B, the table below is filled in. **Alternatives = the
candidates not adopted**, so the chosen side is removed from the
table and described in Decision.

### Comparison table (filled in Phase B)

| Axis | GLIM | FAST-LIO SAM |
|---|---|---|
| Wall time (s) | TBD | TBD |
| Peak VRAM (MiB) | TBD | TBD |
| Peak RSS (KiB) | n/a (not in manifest schema) | TBD |
| Internal trajectory error (m) | TBD | TBD |
| B1 formal error (wall 3-point mean, m) | TBD | TBD |
| Loop closure trigger timing | TBD (e.g. fires once at ~80% of loop) | TBD |
| Keyframe density (per m) | TBD | TBD |
| Manual relocalization needed? | TBD | TBD |
| GTSAM resolution state | n/a (4.3a0 alone) | TBD (4.1.1 alone / coexistence warning / required `LD_LIBRARY_PATH` override) |
| License | MIT | No LICENSE + possible GPL-2.0 propagation |
| Build success | OK (verified in M5R-1) | TBD (upstream "Full ROS2 adaptation" TODO is open) |

### Supplementary notes (filled in Phase B)

- (For the rejected SLAM) what was the deciding factor?
- Why li_slam_ros2 is out of scope here: the platform-pivot §3.3 calls
  it out as a "comparison / fallback" candidate. The empirical
  contest in this ADR is GLIM vs FAST-LIO SAM; li_slam_ros2 only
  comes back if both lose, via a separate ADR.
- (Optional) Comparison-symmetry caveats — e.g. "GLIM upstream did
  not ship a Velodyne-specific config so we ran the Ouster one,
  potentially degrading feature extraction".

## Consequences

Filled in Phase B with the structure below.

### License inventory (B5 acceptance)

For the adopted SLAM, document how this repository integrates it and
what link constraints apply to the operational stack:

- **If GLIM is adopted**: MIT, permissive. No link constraint on the
  operational stack. However, M5-R is the "offline map-building
  tool" phase by design; the runtime localizer belongs to M6-R
  (scan-to-map localizer, separately chosen). Whether to also wire
  GLIM into the operational stack is out of scope here and depends
  on M6-R's evaluation.
- **If FAST-LIO SAM is adopted**: no LICENSE file is effectively
  "all rights reserved" under copyright law; the FAST-LIO origin is
  GPL-2.0 with possible copyleft propagation. Apply the
  platform-pivot §3.4 rule "GPL family is restricted to offline
  map-building tools in a separated process":
  - `whill_lab.repos` entry: **forbidden** (keep clone-on-demand)
  - Linking from operational packages: **forbidden**
  - Storing only the static PCD / occupancy grid outputs under
    `docs/maps/<site>/`: allowed (these are evaluation outputs, not
    redistribution of upstream code)
  - If upstream later adds a permissive LICENSE, the policy switch
    is a separate ADR.

### CPU / GPU / memory profile

Record the measured numbers on the development host (Alienware x15
R2, RTX 3080 Laptop GPU 16 GB VRAM, i9-12900H, 32 GiB RAM). Whether
the adopted SLAM can be moved to a vehicle-borne machine is out of
scope for this ADR — it is re-evaluated in M9 (vehicle separation).

### Impact on downstream phases (M6-R)

- The M6-R scan-to-map localizer ingests the adopted SLAM's static
  PCD via the `docs/maps/<site>/static.pcd` convention (see
  ADR-0005).
- Compatibility check results (PCD format binary vs ASCII, coordinate
  frame, coordinate precision) go in this section.
- Coordinate frame integrity: the M4-R bringup `/tf_static` carries
  the `base_link → velodyne` extrinsic into the bag, so the adopted
  SLAM's PCD will be in either `velodyne` or `base_link` frame.
  Record which and reconcile with the M6-R localizer's assumption.

### Follow-ups

- **M5R-4 (#49) ERASOR**: takes the adopted SLAM's output (PCD +
  per-frame poses) and removes dynamic objects. Cannot start until
  this Decision lands.
- **M5R-6 (#50) Occupancy grid conversion**: turns the ERASOR-static
  PCD into a 2D occupancy grid at `docs/maps/<site>/occupancy.{pgm,yaml}`.
- **M5R-7 (#51) Pipeline integration**: documents the E2E flow
  bag → adopted SLAM → ERASOR → occupancy grid → `docs/maps/<site>/`.
- **Move this ADR from `proposed` to `accepted`**: after the
  Decision is filled and user-approved, a final commit flips the
  Status line.

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
