# M5-R execution plan: map-building pipeline

Language: [日本語](../../ja/plans/2026-06-21-m5r-execution.md) | [English](2026-06-21-m5r-execution.md)

- Date: 2026-06-21
- Status: proposed (awaiting user approval)
- Parent policy: [`docs/en/plans/2026-06-11-platform-pivot.md`](2026-06-11-platform-pivot.md)
  §3.1 (two-phase split), §3.3 (candidate shortlist), §3.4 (licensing),
  §4 (M5-R), §6 (acceptance criteria), §7 (ADR candidates), §9 (hardware)
- Predecessor: [`docs/en/plans/2026-06-13-m4r-execution.md`](2026-06-13-m4r-execution.md)
  (M4-R complete at main `1562361`)
- Intended location: `docs/en/plans/2026-06-21-m5r-execution.md`
- Audience: `research-analyst` / `ros2-implementer` / `code-reviewer` taking on
  this phase, plus the user capturing on-chair bags and signing off on
  artefacts.

The Japanese version is the source of truth; this is a translation.

## 0. Understanding of the requirement

Parent §4 defines M5-R as: "map-making pipeline — bring up GLIM (or FAST-LIO
SAM), dynamic-object removal via ERASOR, artefact convention under
`docs/maps/<site>/` (pcd + pgm + yaml + acquisition metadata)". This plan
breaks that down into executable Issues, fixes the decision path for SLAM
selection (GLIM vs FAST-LIO SAM), the dynamic-removal pipeline, the artefact
convention, and the verification procedure.

Difference in character vs M4-R: M4-R was an implementation phase (4 Issues
in strict serial). M5-R is a "selection phase + post-processing chain", so
the Issue layout makes the decision point explicit because the ADR branches
on the empirical bag comparison result.

## 1. Background

### 1.1 Known issues this phase resolves

From parent §2 diagnosis, this phase covers:

| ID | Content | Resolution path in M5-R |
|----|---------|-------------------------|
| P5 (map-quality side) | The root cause of "ghost obstacles → `use_collision_detection: false`" and "QoS mismatch → no obstacle layer" is the dynamic-object contamination of the legacy map. Loop-closure SLAM + ERASOR produces a static map. The actual revival of the obstacle layer and `use_collision_detection: true` belongs to M6-R | Quality-guarantee for static PCD + occupancy grid |

P1-P4 / remaining P2 are M4-R / M6-R territory and out of scope here.

### 1.2 M4-R artefact assumptions

M5-R consumes the M4-R hand-off (`docs/en/plans/2026-06-13-m4r-execution.md`
§10):

- TF: single chain `odom -> base_link -> {imu_link, velodyne, camera_link}`
- Input topics: `/whill/odom` (~2.5 Hz), `/imu/data_raw` (100 Hz),
  `/velodyne_points` (10 Hz)
- Launch: `whill_localization/launch/odom_bringup_launch.py`
  (sensors + driver + EKF)

Bags must be recorded with this launch up so that the TF tree is present
and `/velodyne_points`, `/imu/data_raw`, `/tf_static` land in the bag. This
gives the generated PCD map a TF tree **consistent by construction** with
the runtime TF (no re-calibration in M5-R).

### 1.3 Frozen in M5-R (don't touch)

- **FAST-LIO runtime hardening**: Parent §5 prohibition #2. The §5 carve-out
  "parameter retuning for map-making quality" leaves FAST-LIO SAM (uses
  FAST-LIO2 as its frontend) as a legitimate candidate
- **scan-to-map localizers**: `lidar_localization_ros2` / `hdl_localization` /
  `mcl_3dl` belong to M6-R. M5-R only prepares their input (the static PCD)
- **Reviving Nav2 obstacle layer**: M6-R
- **`navsat_transform_*` WIP**: per M4-R §3.A, untouched

## 2. Scope

### 2.1 In scope

1. **Verify NVIDIA driver / CUDA 12.4 / cuDNN 8 status**. Issue #23 (closed)
   shipped `scripts/install_cuda.sh` and `docs/en/m5r-cuda-setup.md`; the
   verification date 2026-06-13 records that the host (Alienware x15 R2,
   driver 595, RTX 3080 Laptop GPU) is set up. This phase only re-runs the
   smoke test before GLIM build, no re-install
2. **GLIM on host + minimum smoke test**. Install `glim_ros2` via PPA or
   Docker, run it against an M3-era bag (`m3_chair_motion_*`), confirm a
   trajectory comes out
3. **FAST-LIO SAM cleanup**. `src/third_party/FAST_LIO_SAM/` currently exists
   as a hand-cloned directory not listed in `whill_lab.repos`
   (`CMakeLists.txt`, `README.md` confirmed). License / dependencies /
   vcs handling sorted out (see §3)
4. **GLIM vs FAST-LIO SAM empirical comparison**. Same M4-R-bringup bag fed
   into both GLIM (GPU) and FAST-LIO SAM; compare generated PCD, loop-closure
   error, usability (manual relocalization, keyframe export, etc.). Record
   the decision in ADR-0003 (filed by this plan)
5. **ERASOR-family dynamic removal pipeline**. Offline script that takes the
   loop-closure SLAM output (PCD + per-frame poses) and outputs a static
   point cloud. Verify "trailing residual" disappears on a bag with a
   pedestrian crossing (run3-equivalent)
6. **`docs/maps/<site>/` artefact convention**. Lock in the parent §6 (3)
   "pcd + pgm + yaml + date / route / weather metadata" with a directory
   layout and a README template. Includes a cleanup plan for existing
   `docs/m5-maps/` (M5-b legacy)
7. **Occupancy-grid conversion**. Script that takes the static PCD and
   produces a 2D occupancy grid (pgm + yaml). Reference the legacy M5-b
   `pcd_to_occupancy_grid.py` while emitting under the `docs/maps/<site>/`
   convention
8. **M5-R closing doc**. Adopted SLAM, bag-capture procedure, end-to-end
   post-processing chain, and the M6-R hand-off (static PCD + occupancy
   grid) in one README

### 2.2 Out of scope (explicit)

- **scan-to-map localizer selection / implementation**: M6-R
- **Nav2 costmap pipeline**: M6-R
- **Real-time / on-chair map-making**: Parent §3.1 sets map-making as offline
  on the host. Re-evaluation is for post-M9 if on-chair separation is
  decided
- **Campus-scale bag capture**: In-scope verification is a lab loop bag,
  enough to certify the pipeline. Campus bags are recorded after M5-R, at
  or just before M6-R kick-off — that's an operational call
- **New IMU / LiDAR calibration**: Use the M3 / M4-R extrinsics. If map
  quality forces a redo, separate Issue
- **In-house implementation of dynamic-removal algorithms**: Upstream OSS,
  not reimplemented

## 3. Pre-existing WIP code and remnants

### 3.1 Handling `src/third_party/FAST_LIO_SAM/`

Facts established:

- `src/third_party/` is `.gitignore`'d and recreated via vcs import
  (`whill_lab.repos`)
- `whill_lab.repos` has **no** entry for FAST_LIO_SAM
- Yet `src/third_party/FAST_LIO_SAM/CMakeLists.txt` and `README.md` exist —
  someone hand-cloned it
- Upstream is `https://github.com/RightTr/FAST-LIO-SAM.git`, README declares
  ROS 2 humble support, license not confirmed yet (Issue M5R-2 checks it)

Decision: at the top of Issue M5R-2 (FAST-LIO SAM evaluation prep), pick one:

- (a) If license / deps are permissive enough to be a candidate, add it to
  `whill_lab.repos` as a formal entry, restoring vcs-import reproducibility
- (b) If GPL-family with a risk of contaminating the operational stack,
  delete the physical directory and switch to a "clone on demand" workflow
  documented in `docs/en/m5r-fastlio-sam-eval.md`. Parent §3.4 ("GPL-family
  is restricted to map-making tools as a separate process") makes
  map-making-only use admissible, but linking from operational packages is
  banned and must be stated

### 3.2 Legacy M5-b remnants in `docs/m5-maps/`

Facts: `docs/m5-maps/` contains `lab.pcd`, `lab.pgm`, `lab.yaml`,
`global_2026-06-04_10min.pcd` (`.gitignore` excludes `*.pcd`; `lab.yaml` /
`lab.pgm` are tracked). These are M5-b prototype output (frozen, replaced by
the M5-R defined in parent §4).

Decision: Issue M5R-5 (artefact convention) absorbs these into the
`docs/maps/<site>/` convention:

- Rename `docs/m5-maps/` to `docs/maps/lab-legacy-m5b/` to mark them as
  "pre-freeze prototype" (M5-b is not M5-R)
- Or delete them if they fall below the quality bar and only record the
  history in legacy-findings

Final call is made on inspection at M5R-5 kick-off.

### 3.3 `velodyne_whill.yaml` `pcd_save_en` / `map_file_path`

Facts: `src/whill_localization/config/velodyne_whill.yaml` still has
`map_file_path: /home/systemlab/whill_lab0_ros2/docs/m5-maps/lab.pcd` and
`pcd_save_en: true` with an "M5-b" comment.

Decision: this yaml remains in service during M5-R because FAST-LIO is used
as a map-making tool (offline replay). Do not delete. But repoint the
hardcoded `map_file_path` to a `docs/maps/<site>/` path. Handled in Issue
M5R-7 (pipeline integration).

## 4. Assumptions

- M4R-1 through M4R-4 all merged. `/odometry/filtered`, `odom -> base_link`
  and `whill_localization/launch/odom_bringup_launch.py` work
- Host (Alienware x15 R2): `nvidia-smi` reports driver 595,
  `/usr/local/cuda-12.4/bin/nvcc --version` reports CUDA 12.4 (Issue #23
  done; we just re-run `docs/en/m5r-cuda-setup.md`'s smoke test)
- WHILL Model CR2 / Velodyne VLP-16 / IMU all available. The user drives the
  chair (push or joystick) to capture bags (CLAUDE.md rule)
- At least one loop-drive bag (start = end) and at least one bag with a
  pedestrian crossing, indoor scope, are obtainable
- Any decision overriding parent §3.3 is recorded in ADR-0003 (e.g., putting
  FAST-LIO SAM first instead of GLIM, choosing Removert instead of ERASOR)

## 5. Acceptance criteria (parent §6 + this plan's reinforcement)

Parent §6's three M5-R items, in observable commands:

- [ ] **B1: Loop-closure visual alignment**
  - Command: same start/end loop bag through the adopted SLAM (GLIM or
    FAST-LIO SAM); inspect the PCD in CloudCompare / RViz
  - Expected: the structures at start and end (walls, distinctive corners)
    overlap within tens of cm. Verdict by visual inspection plus distance
    measurement (CloudCompare's Point picking)
  - Pass threshold proposed: average of 3 picks on the same wall at start
    vs end ≤ 0.5 m (= 1% for a 50 m loop. The M4 FAST-LIO solo result was
    18%, so loop-closure + GLIM global optimization should be an order of
    magnitude better)
- [ ] **B2: Dynamic-object removal**
  - Command: bag with pedestrian crossing → adopted SLAM → ERASOR for the
    static PCD; compare before/after
  - Expected: pre-removal shows the pedestrian as a "trailing residual"
    point cluster; post-removal it is gone. Same in the occupancy grid
  - Verification script: a Python script (new
    `scripts/m5r_erasor_diff.py` filed by this plan, Issue M5R-4) overlays
    pre/post PCDs and highlights the diff
- [ ] **B3: `docs/maps/<site>/` artefact completeness**
  - Command: `ls docs/maps/<site>/` shows all of:
    - `static.pcd` (static PCD, post-ERASOR)
    - `occupancy.pgm` (2D occupancy grid)
    - `occupancy.yaml` (Nav2 map_server-compatible metadata)
    - `metadata.yaml` (date / route / weather / adopted SLAM /
      SLAM params / ERASOR params / source bag /commit SHA)
  - Expected: all four present, `metadata.yaml` has every required field.
    Schema locked in Issue M5R-5
- [ ] **B4: ADR-0003 (SLAM choice) reaches `accepted`**
  - The `docs/decisions/0003-mapping-slam-choice.md` filed by this plan is
    accepted (after user review) and merged, backed by the empirical
    comparison from Issue M5R-3

Reinforcement criteria (code-reviewer checks):

- [ ] **B5: License audit recorded**
  - For the adopted SLAM and the adopted dynamic-removal tool, explicitly
    list the license and whether it is linked from operational packages in
    the Consequences section of `docs/decisions/0003-mapping-slam-choice.md`.
    Verify in prose that parent §3.4 ("GPL-family restricted to separate
    process") is honoured
- [ ] **B6: M6-R hand-off conditions stated**
  - `docs/en/m5r-pipeline.md` (new, Issue M5R-7) states "M6-R's
    scan-to-map localizer takes `docs/maps/<site>/static.pcd` as input by
    this convention" and "the occupancy grid is the input for reviving
    Nav2's obstacle layer (M6-R)"

## 6. Issue breakdown

Seven Issues. More than M4-R because M5-R is "selection + 4 post-processing
steps" in a strict chain. Each Issue is sized to stand alone.

### Issue M5R-1: CUDA / GLIM host setup verification

- **Goal**: Re-confirm the CUDA 12.4 + cuDNN 8 environment from Issue #23 is
  still functional, then install GLIM (`glim_ros2`) on the host and run a
  minimum smoke test against a sample bag
- **Acceptance**:
  - [ ] `nvidia-smi` reports driver 595-series
  - [ ] `/usr/local/cuda-12.4/bin/nvcc --version` reports CUDA 12.4
  - [ ] The `vectorAdd` sample in `docs/en/m5r-cuda-setup.md` §2.4 reports
    `Result = PASS`
  - [ ] GLIM installed via apt PPA or Docker; a command equivalent to
    `ros2 run glim_ros glim_rosbag <bag>` launches and outputs a trajectory
  - [ ] The procedure is appended to `docs/en/m5r-cuda-setup.md` or a new
    `docs/en/m5r-glim-setup.md`
- **Out of scope**: Adoption decision (M5R-3 empirical comparison), FAST-LIO
  SAM install (M5R-2)
- **Assumptions**: CUDA 12.4 install is done from Issue #23 (verification
  date 2026-06-13 in `docs/en/m5r-cuda-setup.md`). This Issue is
  reproducibility check only
- **Owning agent**: `ros2-implementer` (the actual install steps are based on
  the existing `research-analyst` survey; no new web research needed)
- **Branch**: `m5r/1-glim-setup`

### Issue M5R-2: FAST-LIO SAM evaluation prep (cleanup + license)

- **Goal**: Decide whether the hand-cloned `src/third_party/FAST_LIO_SAM/`
  should be (a) formalised via `whill_lab.repos`, or (b) deleted and turned
  into "clone on demand for evaluation". License / dependencies sorted out
- **Acceptance**:
  - [ ] LICENSE confirmed on upstream
    (`https://github.com/RightTr/FAST-LIO-SAM.git`) → permissive → (a),
    GPL-family → (b)
  - [ ] (a) case: entry added to `whill_lab.repos`, `vcs import` reproduces
    cleanly. Repo stays vcs-import + `.gitignore` excluded
  - [ ] (b) case: current directory deleted, "clone-on-demand procedure
    when evaluating" recorded in `docs/en/m5r-fastlio-sam-eval.md`
  - [ ] gtsam 4.1 install (`libgtsam-dev`) added to
    `docs/en/m5r-fastlio-sam-eval.md` (per the FAST-LIO SAM README prereq)
  - [ ] In path (a) or (b), the M5R-3 evaluator runs `colcon build --packages-up-to fast_lio_sam` and confirms success (this Issue does not run the build).
- **Out of scope**: empirical bag comparison (M5R-3), linkage into the
  operational stack
- **Owning agent**: `research-analyst` (license fact check) → `ros2-implementer`
  (`whill_lab.repos` edit or directory removal)
- **Branch**: `m5r/2-fastlio-sam-prep`

### Issue M5R-3: GLIM vs FAST-LIO SAM empirical comparison + file ADR-0003

- **Goal**: Same bag (user-captured indoor loop drive with M4-R bringup) fed
  into both GLIM (GPU) and FAST-LIO SAM. Compare PCD, loop-closure error,
  usability (manual relocalization, keyframe export). Decide in
  `docs/decisions/0003-mapping-slam-choice.md` (new)
- **Acceptance**:
  - [ ] User captures at least one loop bag, placed under
    `docs/m5r-bench-data/<YYYY-MM-DD>-loop/bag/` (layout finalised in M5R-7;
    a minimum capture protocol shared with the user before this Issue
    starts)
  - [ ] GLIM run: generated PCD, loop-closure error (B1 metric), elapsed
    time, peak VRAM all recorded
  - [ ] FAST-LIO SAM run: same metrics
  - [ ] `docs/decisions/0003-mapping-slam-choice.md` (proposed) filed with
    the comparison in Context, the chosen tool in Decision, and the other in
    Alternatives
  - [ ] ADR status `proposed` at end of this Issue; user accepts later (per
    ADR-0001 workflow)
- **Out of scope**: dynamic removal (M5R-4), scan-to-map localizer eval (M6-R)
- **Owning agent**: `research-analyst` (structures the comparison) → ADR
  drafting by `pm-orchestrator` to ADR template
- **Branch**: `m5r/3-slam-comparison`

### Issue M5R-4: ERASOR dynamic-removal pipeline

- **Goal**: With the adopted SLAM (from M5R-3), build an offline script that
  takes the SLAM output (PCD + per-frame poses) and ERASOR's static PCD. On
  a bag with a pedestrian crossing (run3-equivalent), visually confirm the
  trailing residual is gone
- **Acceptance**:
  - [ ] ERASOR (`https://github.com/LimHyungTae/ERASOR`, Apache-2.0)
    installed on host. `scripts/m5r_run_erasor.sh` (new) wraps "SLAM output
    → static PCD" idempotently
  - [ ] On a bag with a pedestrian crossing (≥ 1), pre/post PCDs overlaid
    via `scripts/m5r_erasor_diff.py` (new). Pedestrian trace disappears on
    visual inspection
  - [ ] ERASOR parameters (voxel size, PR / RR thresholds, etc.) recorded in
    `docs/en/m5r-pipeline.md` (Issue M5R-7)
  - [ ] B2 (dynamic removal) satisfied
- **Out of scope**: occupancy-grid conversion (M5R-6)
- **Owning agent**: `research-analyst` (final decision including the Removert
  alternative) → `ros2-implementer` (scripting)
- **Branch**: `m5r/4-erasor-dynamic-removal`
- **Uncertainty**: ERASOR's ROS 2 humble build status is not verified
  (upstream is ROS 1 / Ubuntu 18.04 centric). If it fails to build, fall
  back to Removert or GLIM's internal dynamic remover (if any), recorded as
  Alternatives

### Issue M5R-5: Lock in `docs/maps/<site>/` artefact convention

- **Goal**: Translate parent §6 (3)'s "pcd + pgm + yaml + date / route /
  weather metadata" into a directory layout + README template +
  `metadata.yaml` schema. Includes cleanup plan for existing `docs/m5-maps/`
  (M5-b legacy)
- **Acceptance**:
  - [ ] `docs/maps/README.md` (new) describes:
    - Directory rule: under `docs/maps/<site>/`, place `static.pcd` /
      `occupancy.pgm` / `occupancy.yaml` / `metadata.yaml`
    - Required fields of `metadata.yaml`: `acquired_at` (ISO8601),
      `route_summary`, `weather`, `slam_method` (`glim` or `fast_lio_sam`),
      `slam_params` (ref to yaml ok), `erasor_params`, `source_bag` (rel
      path under `docs/m5r-bench-data/...`), `commit` (this SHA)
    - Reference sample under `docs/maps/_template/`
  - [ ] `.gitignore` updated: existing `docs/maps/**/*.pcd` (= exclude large
    files) stays; `*.pgm` / `*.yaml` tracked (Nav2 needs them)
  - [ ] Existing `docs/m5-maps/` fate settled (rename or delete + history in
    legacy-findings)
- **Out of scope**: real PCD / pgm generation (M5R-3, M5R-6)
- **Owning agent**: `pm-orchestrator` (sets the convention) →
  `ros2-implementer` (writes `docs/maps/README.md` and `_template/`)
- **Branch**: `m5r/5-maps-spec`

### Issue M5R-6: Occupancy-grid conversion script

- **Goal**: Convert a static PCD into a Nav2-compatible 2D occupancy grid
  (pgm + yaml). Reference the legacy M5-b `pcd_to_occupancy_grid.py` (which
  produced `docs/m5-maps/lab.yaml`) and emit under the `docs/maps/<site>/`
  convention
- **Acceptance**:
  - [ ] `scripts/m5r_pcd_to_occupancy.py` (new, or refactored from the M5-b
    one): input `static.pcd`, output `occupancy.pgm` + `occupancy.yaml`
    (Nav2 map_server fields: `image` / `resolution` / `origin` / `negate` /
    `occupied_thresh` / `free_thresh`)
  - [ ] Idempotent: overwriting an existing file requires `--force`; by
    default abort
  - [ ] End-to-end conversion of at least one bag (from M5R-3 adopted SLAM)
    `static.pcd → occupancy.pgm + .yaml` succeeds
  - [ ] Parameter docs (`resolution`, `z_clip` lower / upper, etc.) added to
    `docs/en/m5r-pipeline.md` (M5R-7)
- **Out of scope**: dynamic removal (M5R-4), Nav2 verification (M6-R)
- **Owning agent**: `ros2-implementer`
- **Branch**: `m5r/6-occupancy-grid`

### Issue M5R-7: Pipeline integration + M5-R closing doc

- **Goal**: Stitch M5R-1 through M5R-6 into one documented pipeline (bag
  capture → SLAM → dynamic removal → occupancy grid → `docs/maps/<site>/`
  archival). State the M6-R hand-off
- **Acceptance**:
  - [ ] `docs/ja/m5r-pipeline.md` / `docs/en/m5r-pipeline.md` (new,
    bilingual per ADR-0001) document:
    - Bag capture procedure (stand up M4-R bringup, record what)
    - SLAM run command (per adopted tool)
    - ERASOR run command
    - Occupancy-grid conversion command
    - `metadata.yaml` fill-in guide
    - Final artefact location (`docs/maps/<site>/...`)
  - [ ] `docs/m5r-bench-data/README.md` (new, follows `docs/m4r-bench-data/README.md`
    template) fixes the bag-capture convention. `.gitignore` updated
  - [ ] `velodyne_whill.yaml` `map_file_path` hardcode pointed at the M5-R
    output path (run not required immediately; the comment notes "aligned
    to M5-R convention")
  - [ ] CLAUDE.md's "in-progress known issues" P5 (map-quality side) marked
    resolved (separate commit; this Issue drafts the text)
  - [ ] The acceptance flow for ADR-0003 (move `proposed` → `accepted`) is
    documented at the tail of `docs/en/m5r-pipeline.md`
- **Out of scope**: scan-to-map localizer impl (M6-R), Nav2 costmap pipeline
  (M6-R)
- **Owning agent**: `pm-orchestrator` → `ros2-implementer` (writes the docs)
- **Branch**: `m5r/7-pipeline-doc`

## 7. Execution order and dependencies

```
M5R-1 (GLIM setup) ────┐
                       ├──> M5R-3 (compare + ADR-0003) ──> M5R-4 (ERASOR) ──> M5R-6 (occupancy)
M5R-2 (FAST-LIO SAM ──┘                                                                │
       prep)                                                                           │
                                                                                       ▼
M5R-5 (maps spec) ──────────────────────────────────────────────────────────> M5R-7 (closing doc)
```

- M5R-1 and M5R-2 are independent and parallelisable
- M5R-3 needs both M5R-1 and M5R-2 (both tools run for the comparison)
- M5R-5 (convention) is independent; running it in parallel with M5R-3 is
  efficient because the template is needed downstream
- M5R-4 → M5R-6 → M5R-7 is strict serial. M5R-7 wraps up the final
  `docs/maps/<site>/` artefact

Hardware-sharing constraint (one chair, user-only driving): batch all bag
captures right before M5R-3 starts. M5R-1 / M5R-2 are pure software and run
in parallel.

## 8. Verification strategy

### 8.1 Per-Issue verification

| Issue | How to verify |
|-------|---------------|
| M5R-1 | `vectorAdd` PASS; GLIM trajectory on a sample bag (screenshot) |
| M5R-2 | Upstream LICENSE confirmed → physical deletion + clone-on-demand procedure documented → acceptance (a)/(b) met. The actual colcon build is executed by the M5R-3 evaluator. |
| M5R-3 | Same bag through GLIM and FAST-LIO SAM; overlay PCDs in CloudCompare, quantify loop error; file ADR-0003 |
| M5R-4 | Pre/post PCDs on a dynamic-bag via `scripts/m5r_erasor_diff.py`; visually verify pedestrian trace removed |
| M5R-5 | `docs/maps/_template/` carries every file in the spec; `metadata.yaml` lints (key-presence script ok) |
| M5R-6 | Sample `static.pcd` → `occupancy.pgm + .yaml` succeeds; RViz `map_server` loads it |
| M5R-7 | E2E: bag → SLAM → ERASOR → occupancy → `docs/maps/<site>/` for one site, end-to-end |

### 8.2 Bench-data convention (locked in M5R-7)

`docs/m5r-bench-data/<YYYY-MM-DD>-<run-id>/` holds the bag (gitignored), a
README, and intermediate artefacts (PCDs gitignored as large files;
screenshots tracked). Same "README lifted outside the dir to stay tracked"
pattern as `docs/m4r-bench-data/`.

Final artefacts (static PCD + occupancy grid + metadata) live under
`docs/maps/<site>/` separately. The split keeps "intermediate artefacts"
distinct from "operational inputs".

### 8.3 Rationale for the 0.5 m loop-closure threshold

- M4-era FAST-LIO solo (no loop closure): 18% over 60 s (`run2`, measured)
- Loop-closure SLAM (GLIM / FAST-LIO SAM) globally optimises start/end to
  coincide; the order of "tens of cm" is structurally expected
- ICP / NDT reproducibility on 50 m indoor loops (M3-era NDT eval): tens of
  cm
- 50 m × 1% = 0.5 m. Worse than this signals map quality that would hurt
  M6-R's scan-to-map; we mandate recapture or param retune in this phase

## 9. Risks and uncertainties

### 9.1 Risks

- **GLIM GPU VRAM overrun**: RTX 3080 Laptop has 16 GB. GLIM is benchmarked
  on Jetson Orin (8 / 32 GB) and x86 GPUs. A long bag (> 10 min) might OOM.
  Mitigation: split into 1-3 min bags or run GLIM in CPU mode; record peak
  VRAM in M5R-3 comparison
- **ERASOR humble compatibility unknown**: Upstream is ROS 1 / Ubuntu 18.04
  centric. ROS 2 humble + Ubuntu 22.04 build status not verified. Mitigation:
  at M5R-4 kick-off, PoC build first; fall back to Removert or GLIM's
  built-in dynamic processing (if available) as Alternatives
- **Bag capture under M4-R bringup**: assumed but may fail (USB enumeration,
  udev not yet set, `/whill/odom` not emitted). Mitigation: before M5R-3,
  ask the user to do a short smoke-test bag confirming all of `/tf_static`,
  `/whill/odom`, `/imu/data_raw`, `/velodyne_points` arrive
- **FAST-LIO SAM GPL contagion**: parent §3.4 allows separate-process use.
  But adopting it locks us into a permanent "no operational link" contract.
  Spell this out in M5R-2 Alternatives and in M5R-7's
  `docs/en/m5r-pipeline.md`
- **Adopted SLAM upstream churn**: GLIM / FAST-LIO SAM both move quickly;
  API churn may break the build. Mitigation: ADR-0003 pins commit SHA /
  tag; `whill_lab.repos` (if adopted via vcs) or the procedure (FAST-LIO SAM
  case) records it
- **Legacy `docs/m5-maps/` deletion accident**: If `lab.yaml` / `lab.pgm` is
  referenced anywhere, deletion breaks something. Mitigation: at M5R-5
  kick-off, grep for references; rename instead of delete if used

### 9.2 Uncertainties

- **Campus-scale bag captured during M5-R?**: Parent §6 B1-B3 are
  achievable on an indoor loop bag. Campus bags happen after M5-R, at or
  just before M6-R (per §2.2). The "does GLIM scale to campus?" risk gets
  verified during M6-R if it materialises. M5-R's scope is **indoor 50 m
  loop scale** only
- **Dynamic-removal thresholds**: ERASOR's `r_min`, `r_max`, voxel size
  behave differently indoor vs outdoor. M5R-4 locks in one indoor set; an
  outdoor retune is likely at M6-R kick-off, noted
- **`base_link` origin impact on M5-R**: M4R-2 provisionally pinned
  base_link at "rear-axle centre, ground level". Whether that aligns with
  the map origin emerges in M6-R. M5-R does not redefine base_link (sticks
  with M4-R policy)

## 10. Hand-off to subsequent phases

At M5-R completion:

- Adopted SLAM (fixed by ADR-0003: GLIM or FAST-LIO SAM)
- Adopted dynamic-removal tool (ERASOR or alternative, locked in M5R-4)
- `docs/maps/<site>/` convention (static.pcd + occupancy.pgm + occupancy.yaml
  + metadata.yaml)
- Pipeline doc `docs/en/m5r-pipeline.md`
- Bag-capture convention `docs/m5r-bench-data/README.md`
- One indoor test site's complete artefact set (e.g. `docs/maps/lab-loop/`)

Downstream:

- **M6-R**: A scan-to-map localizer (`lidar_localization_ros2` first
  candidate) takes `docs/maps/<site>/static.pcd` and publishes `map -> odom`.
  Initial-pose UX also lands in M6-R
- **M6-R**: Nav2's obstacle layer is fed `docs/maps/<site>/occupancy.yaml`
  via map_server; `use_collision_detection: true` returns
- **Campus bag**: same pipeline applied to the campus loop bag. Captured by
  the user just before or in parallel with M6-R kick-off, yielding
  `docs/maps/utsunomiya-campus/`

## 11. ADR candidates

Technical decisions from this phase worth recording:

- [ ] **ADR-0003: Map-making SLAM choice (GLIM vs FAST-LIO SAM)**. Issue
  M5R-3 fills `docs/decisions/0003-mapping-slam-choice.md` (proposed) after
  empirical comparison; user accepts. Parent §7: "GLIM adoption prereqs are
  met; final call is the empirical comparison against FAST-LIO SAM"
- [ ] **ADR-0004 candidate: Dynamic-removal tool (ERASOR vs Removert vs
  GLIM-internal)**. Filed only if M5R-4 hits ERASOR build/perf trouble.
  Default path needs no ADR (parent §3.3 has ERASOR as first candidate)
- [ ] **ADR-0005 candidate: `docs/maps/<site>/` convention**. Filed if
  M5R-5's `metadata.yaml` schema + `_template/` warrants an ADR (it does if
  M6-R / M9 will keep editing it). Decided at M5R-5 kick-off

## 12. Next actions

Once this plan is `accepted`:

1. Open the seven Issues (M5R-1 to M5R-7) via `gh issue create` (this plan
   defines the breakdown; opening is a separate step)
2. Start M5R-1 (CUDA verify + GLIM host install) and M5R-2 (FAST-LIO SAM
   cleanup) in parallel
3. Run M5R-5 (maps spec) in parallel with M5R-3 to land the template early
4. After M5R-1 / M5R-2, ask the user for "indoor loop drive bag" and
   "pedestrian-crossing bag". After capture, start M5R-3
5. M5R-3 → M5R-4 → M5R-6 → M5R-7 serial. Accept ADR-0003 to close the
   phase. Reflect P5 resolution in CLAUDE.md and prep M6-R
