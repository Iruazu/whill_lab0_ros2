# M5R-3: GLIM vs FAST-LIO SAM real-bag comparison protocol

Language: [日本語](../ja/m5r3-comparison-protocol.md) | [English](m5r3-comparison-protocol.md)

## Goal

Give the M5R-3 (Issue #48) Phase B evaluator a reproducible runbook for
the real-bag comparison. The destination is **filling the Context and
Alternatives sections of ADR-0003
([`decisions/0003-mapping-slam-choice.md`](decisions/0003-mapping-slam-choice.md))**
with four things:

- The generated PCD from each SLAM against the same bag,
- Loop-closure error for each SLAM — the formal B1 metric (three wall
  points picked in CloudCompare at start and end of the loop) plus the
  complementary trajectory end-to-start distance,
- Wall time and peak VRAM for each SLAM (plus host RSS for FAST-LIO SAM),
- Qualitative operability notes (need for manual relocalization,
  keyframe density, loop-closure trigger timing, etc.).

Once those numbers land, the ADR Decision section gets filled in a
follow-up commit and the PR moves to ready-for-review. This document is
the Phase B runbook; at Phase A landing time the numerical placeholders
in ADR-0003 remain blank.

For selection rationale and traceability to requirements, see
[`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
§3.3 and §7. Issue structure is
[`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md)
§M5R-3.

## Prereq environment

| | |
|--|--|
| M5R-1 (#45) | GLIM source-build install complete. `/usr/local/lib/libgtsam.so.4.3a0`, `libgtsam_points_cuda.so`, `libiridescence.so` are in place; `ros2 pkg list \| grep glim_ros` succeeds. |
| M5R-2 (#46) | FAST-LIO SAM is clone-on-demand. §3 below covers the clone + build at evaluation time. |
| M5R-5 (#47) | `docs/maps/<site>/` convention is settled (the final destination for the adopted SLAM's static PCD). This Issue's intermediate artifacts go under `docs/m5r-bench-data/`. |
| M4-R bringup | `ros2 launch whill_localization odom_bringup_launch.py` brings up sensors + driver + EKF. The `/tf_static` recorded at bag time carries the M4R-2 (#36) measured extrinsic. |

## GTSAM coexistence (most important precondition)

Two GTSAM versions coexist on the evaluation host:

| Used by | Version | Path | Origin |
|---|---|---|---|
| GLIM | 4.3a0 | `/usr/local/lib/libgtsam.so.4.3a0` | `scripts/install_glim.sh` (M5R-1) |
| FAST-LIO SAM | 4.1.1 | `/usr/lib/x86_64-linux-gnu/libgtsam.so.4.1.1` | `scripts/clone_fastlio_sam_for_eval.sh` (M5R-2, borglab PPA) |

The two are ABI-incompatible. **Linking both into the same process is
undefined behaviour** — the dynamic linker resolves whichever the search
order picks first, then symbol mismatches surface as crashes. Policy:

- Run the GLIM evaluation and the FAST-LIO SAM evaluation in **separate
  terminals / separate shells**.
- If FAST-LIO SAM misbehaves, force the 4.1 lookup with
  `LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH` and re-run.
- `scripts/m5r3_run_fastlio_sam.sh` dumps `ldconfig -p | grep libgtsam`
  to `gtsam_env.log` before the run and warns if both versions are
  visible. The resulting state is recorded as an ADR-0003 Alternatives
  row.

See [`m5r-fastlio-sam-eval.md`](m5r-fastlio-sam-eval.md) §3 for the
extended discussion.

## LiDAR mismatch (GLIM config switch)

GLIM upstream's sample bag is an Ouster OS1-128 (topic `/points`),
whereas the M4-R bringup in this repo publishes Velodyne VLP-16 (topic
`/velodyne_points`). `scripts/m5r3_run_glim.sh` switches the GLIM config
bundle based on whether `/velodyne_points` appears in the bag's
`metadata.yaml`:

- `/velodyne_points` present → `config_velodyne/` (falling back to
  `config_velodyne_vlp16/` then plain `config/` with a warning if
  upstream does not ship one);
- otherwise → default `config/` (the Ouster sample path).

When the fallback fires it can break the symmetry of the comparison
(different ring assumptions in feature extraction), so the script
prints a warning and the evaluator MUST record the fact in the ADR-0003
Alternatives row (e.g. "Upstream ships no Velodyne-specific config; ran
with Ouster config — feature extraction may be degraded").

## Phase B procedure

### 1. Record the bag (user task)

Bring up sensors + driver + EKF via the M4-R bringup launch, then
record an indoor loop.

```bash
cd ~/whill_lab0_ros2
source install/setup.bash
ros2 launch whill_localization odom_bringup_launch.py
```

In a second terminal, record the bag:

```bash
mkdir -p docs/m5r-bench-data/$(date +%Y-%m-%d)-loop
cd docs/m5r-bench-data/$(date +%Y-%m-%d)-loop
ros2 bag record -o bag /velodyne_points /imu/data_raw /tf /tf_static
```

Driving conditions:

- ~50 m indoor loop returning to the exact start point. B1 measures
  wall 3-point distance at start vs end, so the same wall must be
  visible at both ends.
- Mean speed ~0.3 m/s. Avoid sharp acceleration / sharp turns — they
  upset the FAST-LIO family IMU bias estimator.
- A physical marker (tape, floor line, etc.) at the start/end makes
  CloudCompare point picking easier later.

`Ctrl-C` stops the recording. Confirm `bag/*.db3` (or `.mcap`) plus
`bag/metadata.yaml` were written.

### 2. Run GLIM

In a clean shell (no `LD_LIBRARY_PATH` overrides):

```bash
cd ~/whill_lab0_ros2
source install/setup.bash
RUN_DIR=docs/m5r-bench-data/$(date +%Y-%m-%d)-loop

bash scripts/m5r3_run_glim.sh ${RUN_DIR}/bag ${RUN_DIR}/glim-out
```

Things to watch during the run (qualitative data — transcribe into the
ADR):

- In the Iridescence window: **when does loop closure fire?** Which
  keyframe index produces the visible pose-graph jump in the second half?
- **Keyframe density** (how many per metre travelled).
- Whether any manual intervention was needed (relocalization,
  recovering from a pause, etc.).
- Peak VRAM (also captured to `${RUN_DIR}/glim-out/vram.log` at 0.5 s
  cadence).

Outputs:

- `${RUN_DIR}/glim-out/traj_lidar.txt` — TUM-format trajectory
- `${RUN_DIR}/glim-out/dump.pcd` or `map.pcd` — generated PCD
- `${RUN_DIR}/glim-out/manifest.yaml` — run metadata; `results:` keys
  stay TBD until §5
- `${RUN_DIR}/glim-out/run.log` — stdout/stderr including the
  `/usr/bin/time -p` output
- `${RUN_DIR}/glim-out/vram.log` — 0.5 s `nvidia-smi` dump

### 3. Run FAST-LIO SAM

First-time-only clone + build. **Read §"GTSAM coexistence" above before
running this.**

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
export FASTLIO_SAM_LICENSE_ACK=yes
bash scripts/clone_fastlio_sam_for_eval.sh
colcon build --packages-up-to fast_lio_sam --symlink-install
```

If `colcon build` fails (upstream still flags "Full ROS2 adaptation" as
TODO, so this is plausible):

1. Save the failure log to `${RUN_DIR}/fastlio-sam-out/build-failure.log`.
2. CLAUDE.md forbids editing `src/third_party/`. Either provide a
   wrapper inside this repo, or record the failure in ADR-0003
   Alternatives as "FAST-LIO SAM upstream master does not build at the
   time of M5R-3" and drop FAST-LIO SAM from the comparison.
3. In the drop case, skip the rest of §3 and use only the GLIM data in
   §4 to fill the ADR.

If the build succeeded, run the wrapper per bag:

```bash
cd ~/whill_lab0_ros2
source install/setup.bash
RUN_DIR=docs/m5r-bench-data/$(date +%Y-%m-%d)-loop

bash scripts/m5r3_run_fastlio_sam.sh ${RUN_DIR}/bag ${RUN_DIR}/fastlio-sam-out
```

If the wrapper warns that both GTSAM 4.1 and 4.3 are visible to
`ldconfig`:

```bash
LD_LIBRARY_PATH=/usr/lib:${LD_LIBRARY_PATH:-} \
  bash scripts/m5r3_run_fastlio_sam.sh ${RUN_DIR}/bag ${RUN_DIR}/fastlio-sam-out --force
```

Outputs (schema matches the GLIM run for direct comparison):

- `${RUN_DIR}/fastlio-sam-out/traj.txt` — TUM-format trajectory
  (upstream may rename it between releases; check first)
- `${RUN_DIR}/fastlio-sam-out/map.pcd` — generated PCD
- `${RUN_DIR}/fastlio-sam-out/manifest.yaml`
- `${RUN_DIR}/fastlio-sam-out/run.log` + `slam.log` — wrapper stdout
  and SLAM node stdout respectively
- `${RUN_DIR}/fastlio-sam-out/vram.log` + `rss.log`
- `${RUN_DIR}/fastlio-sam-out/gtsam_env.log` — `ldconfig -p` snapshot
  taken before the run

### 4. Measure loop error

#### 4.1 Complementary metric: trajectory end-to-start distance

Run `scripts/m5r3_loop_error.py` against both trajectories:

```bash
python3 scripts/m5r3_loop_error.py docs/m5r-bench-data/<run>/glim-out/traj_lidar.txt
python3 scripts/m5r3_loop_error.py docs/m5r-bench-data/<run>/fastlio-sam-out/traj.txt
```

This computes the **SLAM's self-reported loop mismatch** — the Euclidean
distance between the first and last pose in the trajectory. A SLAM that
never closed the loop in its own pose graph produces a large number
here.

For machine-readable transcription to the ADR table:

```bash
python3 scripts/m5r3_loop_error.py docs/m5r-bench-data/<run>/glim-out/traj_lidar.txt --json
```

#### 4.2 Formal metric (B1): CloudCompare wall 3-point mean

[`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md)
§6 B1 fixes the acceptance threshold as "mean distance between three
points on the same wall at start vs end ≤ 0.5 m". This is a physical
measurement on the generated PCD. In CloudCompare:

1. Install CloudCompare 2.12.x or newer (Ubuntu 22.04 universe ships it
   as `sudo apt install cloudcompare`).
2. Open `${RUN_DIR}/glim-out/dump.pcd` (or `map.pcd`).
3. Enter **Point Picking** mode (shortcut `P`, or right-click → Pick
   Points; CC has moved the entry between Edit / Display / Tools across
   versions, so the keyboard shortcut is the only stable invocation).
3. Pick three distinguishable points (corner, feature, floor-wall edge)
   on a wall scanned just after the run started, plus the corresponding
   three points on the same wall at the end of the run.
4. Compute the 3D distance of each pair and average; record the result
   under `loop_error_wall_3pt_m` in the ADR-0003 Alternatives row.
5. Repeat with the FAST-LIO SAM PCD using the same wall, same three
   pick points.

**4.1 and 4.2 measure different things**: 4.1 is internal to the SLAM,
4.2 is the physical error in the world frame. Both go in the ADR.

### 5. Transcribe to ADR-0003

For each SLAM, copy the following into the Alternatives table:

| Datum | Source | ADR-0003 column |
|---|---|---|
| Wall time (duration_sec) | `manifest.yaml` | "Wall time" |
| Peak VRAM (max_vram_mib) | `manifest.yaml` | "Peak VRAM" |
| Peak RSS (FAST-LIO SAM only) | `manifest.yaml` | "Peak RSS" |
| Trajectory end-to-start distance | `m5r3_loop_error.py` output | "Internal loop error" |
| Wall 3-point mean distance | CloudCompare measurement | "B1 error" |
| GTSAM resolution state | `gtsam_env.log` + warning presence | "GTSAM resolution" |
| Operability notes | live observations | "Qualitative notes" |
| License | `m5r-glim-setup.md` / `m5r-fastlio-sam-eval.md` | Consequences section |

If multiple bags exist, capture each as its own row and have the final
Decision draw from all rows together.

### 6. Decision and PR ready-for-review

Once the numbers are filled in, a follow-up commit fills the Decision
placeholder in both
`docs/ja/decisions/0003-mapping-slam-choice.md` and
`docs/en/decisions/0003-mapping-slam-choice.md`. Required content:

- Adopted SLAM (GLIM / FAST_LIO_SAM)
- Commit SHA / tag pin (transcribed from the adopted SLAM's
  `manifest.yaml` `git_commit` field at the time of the run)
- Rationale (B1 error, loop-closure behaviour, license, operability)

Then flip the PR to ready-for-review. After user approval, a final
commit moves the Status from `proposed` to `accepted` (per ADR-0001
§5 workflow).

## Related

- Policy document: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
  §3.3 (SLAM candidates), §3.4 (license policy), §7 (ADR-0003 candidate)
- M5-R execution plan: [`plans/2026-06-21-m5r-execution.md`](plans/2026-06-21-m5r-execution.md)
  §M5R-3, §6 (acceptance B1–B5)
- Predecessor docs: [`m5r-glim-setup.md`](m5r-glim-setup.md) (GLIM
  source build), [`m5r-fastlio-sam-eval.md`](m5r-fastlio-sam-eval.md)
  (FAST-LIO SAM clone-on-demand)
- Scripts: [`scripts/m5r3_run_glim.sh`](../../scripts/m5r3_run_glim.sh),
  [`scripts/m5r3_run_fastlio_sam.sh`](../../scripts/m5r3_run_fastlio_sam.sh),
  [`scripts/m5r3_loop_error.py`](../../scripts/m5r3_loop_error.py)
- ADR: [`decisions/0003-mapping-slam-choice.md`](decisions/0003-mapping-slam-choice.md)
  — populated after Phase B
- Related issues: #48 (this Issue, M5R-3), #45 (M5R-1), #46 (M5R-2),
  #47 (M5R-5), #49 (M5R-4 — feeds the adopted SLAM's output into ERASOR)
