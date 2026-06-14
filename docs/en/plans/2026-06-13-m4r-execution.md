# M4-R execution plan: odom-foundation rebuild

Language: [日本語](../../ja/plans/2026-06-13-m4r-execution.md) | [English](2026-06-13-m4r-execution.md)

- Date: 2026-06-13
- Status: accepted (Iruazu, 2026-06-14)
- Parent policy: [`docs/en/plans/2026-06-11-platform-pivot.md`](2026-06-11-platform-pivot.md) §4 (M4-R), §6 (acceptance criteria)
- Intended location: `docs/en/plans/2026-06-13-m4r-execution.md`
- Audience: `ros2-implementer` / `debugger` / `code-reviewer` engaging this phase,
  and the user running on-chair verification.

The Japanese version is the source of truth; this is a translation.

## 0. Understanding of the requirement

Parent §4 fixes this phase as "M4-R = odom-foundation rebuild". This document
breaks that down into executable Issue-sized units. Parent §6 fixes three
acceptance criteria (single-chain TF / `/odometry/filtered` published /
`tf_bridge_launch.py` retired). This plan settles the work order, dependencies,
and on-chair verification procedure that lets us satisfy those three.

## 1. Background

### 1.1 Why M4-R goes first (re-statement of parent §4 footer)

If M6-R (scan-to-map localizer) lands first, correction jumps in `map -> base_link`
hit the controller directly and cause sharp jerks on the chair. REP-105 expects
the discontinuity of `map -> odom` to be absorbed by a continuous
`odom -> base_link`. Establishing `odom -> base_link` first is the only reason
M4-R is sequenced before M6-R.

### 1.2 Known issues this phase resolves

From parent §2 diagnosis, P4 and part of P2:

| ID | Content | Resolution path in M4-R |
|----|---------|-------------------------|
| P4 | Missing odom frame, wheel odometry unused — no cushion for correction jumps, no fallback when LiDAR degrades | Build `odom -> base_link` via wheel+IMU EKF |
| P2 (partial) | base_link → each sensor is still an identity placeholder | Expand the measured LiDAR↔IMU extrinsic (`docs/en/m3-extrinsics-from-noetic.md`) into `base_link -> imu_link` / `base_link -> velodyne`, replacing the identities. The rest of P2 (initial-pose UX) carries over to M6-R |

### 1.3 Issues left alone in M4-R

- P1 (no map correction path) → M6-R
- P2 remainder (initial pose UI / arbitrary-location start) → M6-R
- P3 (divergence detection, failsafe) → M6-R
- P5 (map quality, obstacle layer) → M5-R / M6-R

## 2. Scope

### 2.1 In scope

1. **`/whill/odom` supplier**. Upstream `whill_driver` only publishes
   `/whill/states/model_cr2` (`whill_msgs/ModelCr2State`); see
   `src/third_party/ros2_whill/whill_driver/src/whill_node.cpp:41-44`. We add a
   publisher to the `Iruazu/ros2_whill` fork (already used via `whill_lab.repos`,
   with the M2 cold-boot init patch on record) so that `whill_driver` itself
   computes `nav_msgs/Odometry` from left/right motor angles and speeds and
   publishes it on `/whill/odom` (option 1; details in §6 M4R-1)
2. **robot_localization EKF**. Fuses `/whill/odom` + `/imu/data_raw` to produce
   the `odom -> base_link` TF and `/odometry/filtered`
3. **base_link static TFs with measured values**. Replace the three identity
   transforms (`base_link -> imu_link` / `base_link -> velodyne` /
   `base_link -> camera_link`) with measured values, using the noetic-inherited
   LiDAR↔IMU pose plus a newly defined base_link origin
4. **Retire `tf_bridge_launch.py`**. Parent §5 prohibition 1, §6 acceptance 3
5. **A new bringup launch**. Brings up sensors + whill_driver (with the fork
   patch, also publishing `/whill/odom`) + EKF in one launch
6. **Verification artefacts**. A `docs/m4r-bench-data/<run>/` layout and a
   README template

### 2.2 Out of scope (explicit)

- **Do not touch FAST-LIO**. Parent §5 prohibition 2. `fast_lio_launch.py`
  and `velodyne_whill.yaml` are frozen until M5-R
- **No GPS / navsat_transform**. The WIP `navsat_transform_launch.py` /
  `navsat_transform.yaml` is excluded; see §3
- **No Nav2 reintegration**. `src/whill_navigation/launch/nav_launch.py` remains
  broken once `tf_bridge_launch.py` is retired. Rewiring TF into Nav2 and
  reviving the obstacle layer are M6-R work
- **Do not revisit `use_collision_detection: false`**. Parent §5 prohibition 3
  and M6-R scope
- **Do not directly edit the local checkout under `src/third_party/ros2_whill/`**.
  Parent §5 prohibition 5. `/whill/odom` supply is implemented by adding a
  publisher to the `Iruazu/ros2_whill` fork (option 1; the fork URL is already
  set in `whill_lab.repos`, with the M2 cold-boot init patch on record).
  The no-edit rule on the local checkout and the fork-patch path are
  orthogonal conventions (see §6 M4R-1 for the convention re-interpretation)

## 3. Handling of pre-existing WIP code

Phase A through C (M5-e sub-phases) were formally frozen by Issue #28
(merged 2026-06-13). The archive is reachable via the annotated tag
`legacy/m5e-phase-abc-2026-06-13` (= `origin/m5e/velodyne-self-filter` HEAD =
`9b5be71`).

The `src/whill_odometry/` package included in the freeze (created in the
Phase A commit `204989e` — a 276-line C++ node) is **not pulled back into
main** (user judgement α, 2026-06-13). Reading the archive as a reference
for design decisions is allowed (consistent with α).

See `docs/en/legacy-findings/2026-06-13-m5e-frozen.md` for the rationale.

### §3.A `navsat_transform` WIP preservation policy

These two untracked files are *not* incorporated into M4-R:

- `src/whill_localization/config/navsat_transform.yaml`
- `src/whill_localization/launch/navsat_transform_launch.py`

Rationale:

1. Parent §3.2 makes `map -> odom` the responsibility of the scan-to-map
   localizer (M6-R); GPS is listed in §7 as an undecided ADR ("GNSS/RTK
   integration for outdoor expansion"). Bringing GPS into M4-R puts
   implementation ahead of a layer that the parent policy reserves for a
   decision
2. That said, the WIP is not an experiment — it is intentionally designed
   ("anchor the map frame at a fixed campus datum (36.550814, 139.928684)") for
   an August open-campus demo (see the yaml header). This is an alternative
   path to requirement R2 (persistent map frame), distinct from the scan-to-map
   direction the parent policy declares. We preserve it rather than delete it
   under §3.A
3. None of the three M4-R acceptance criteria need GPS. Including it forces a
   two-stage EKF (ekf_odom + ekf_map) and inflates the M4-R scope

#### Post-M4-R decision flow

The two WIP files stay untracked and untouched throughout M4-R. After M4-R, a
separate Issue decides among:

- (a) File an ADR: "Take the GPS-datum path as an interim route to a fixed map
  frame for the August demo?" Logged in `docs/decisions/`, evaluated against
  parent §3.2 (scan-to-map for map→odom) and R2 — coexist or replace
- (b) Use as input to the parent §7 "GNSS/RTK integration" ADR, after M5-R /
  M6-R have shaped
- (c) Reject; record the commit history in an ADR

The job of this plan is to push that decision out to a post-M4-R Issue. We
do not decide here because the inputs (formal demo approval, scan-to-map
shortlist, university outdoor-driving permission) are not yet aligned.

## 4. Assumptions

- The robot_localization EKF (parent §3.3 first choice) is used for odom fusion;
  binaries already exist under `/opt/ros/humble/setup.bash`
- Upstream `ros2_whill` publishes `whill_msgs/ModelCr2State` on
  `/whill/states/model_cr2`. `motor_angle` may be cumulative degrees (not rad)
  given `whill.cpp`'s raw conversion; `motor_speed` may be km/h. Units are
  pinned by a one-rotation test on real hardware (handled in Issue M4R-1)
- Chassis parameter seeds: wheel radius 0.1325 m, tread 0.520 m, both from
  `src/third_party/ros2_whill/whill_description/urdf/whill_model_cr2.urdf:62, 99`.
  Note that `whill_node.cpp:120` comments `tread = 0.496 m`, contradicting the
  URDF. M4R-1 settles this empirically or by cross-checking the noetic
  `ros_whill/ros_whill.cpp`
- The IMU is already activated by `whill_sensors_bringup/imu_launch.py` and
  publishes `/imu/data_raw` (100 Hz); the EKF consumes it as-is
- The LiDAR↔IMU extrinsic is in `docs/en/m3-extrinsics-from-noetic.md`. It is
  "LiDAR origin in the IMU frame", so after fixing `base_link -> imu_link`
  we compute `base_link -> velodyne = (base_link -> imu_link) * (imu_link -> velodyne)`.
  The physical definition of `base_link` (seat centre / rear-axle centre /
  body centre) is pinned in Issue M4R-2
- On-chair verification is done by the user (push or joystick). Claude
  delivers launches and metrics scripts; the user runs the driving (per
  CLAUDE.md)

## 5. Acceptance criteria (parent §6 + this plan's reinforcement)

The three M4-R items from parent §6, made into observable commands with
expected values:

- [ ] **A1: Single-chain TF**
  - Command: `ros2 launch whill_localization odom_bringup_launch.py` (new),
    then in another terminal `ros2 run tf2_tools view_frames`
  - Expected: the generated `frames.pdf` has no `map` node (or it is isolated),
    and `odom -> base_link -> {imu_link, velodyne, camera_link}` is a single
    chain
  - (Note: no one publishes `map` in M4-R. `map -> odom` returns in M6-R)
- [ ] **A2: `/odometry/filtered` quality**
  - Command: `ros2 topic hz /odometry/filtered` shows about 30 Hz (EKF default);
    `ros2 topic echo /odometry/filtered --once` shows `frame_id=odom` and
    `child_frame_id=base_link`
  - 10-m straight push test: after resetting at the start (equivalent to a
    `ros2 service call /set_pose ...`), push 10 m straight along a chalked /
    taped line. Record the end-position from `/odometry/filtered.pose.pose.position`
    against the physically measured 10 m
  - **Pass threshold proposed by this plan: end-position error ≤ 0.5 m (= 5%)**
    Rationale: WHILL Model CR2 factory-calibrated wheel odometry is typically
    2-3% accurate. IMU fusion reduces yaw error but for straight pushes leaves
    tread-estimate error, tyre-radius temperature drift, and floor slip,
    summing to 3-5%. 5% over 10 m is a realistic M4-R pass line and is well
    within what scan-to-map (post-M5-R) can absorb
- [ ] **A3: `tf_bridge_launch.py` retired**
  - `ls src/whill_navigation/launch/tf_bridge_launch.py` shows "No such file"
  - `colcon build --packages-select whill_navigation whill_localization
    whill_sensors_bringup` succeeds
  - `ros2 launch whill_localization odom_bringup_launch.py` runs (we accept
    that `nav_launch.py` breaks; M6-R restores it)

Reinforcement criteria (added here; checked by code-reviewer):

- [ ] **A4: extrinsics with documented provenance**
  - The value used for `base_link -> imu_link` is backed by a note in
    `docs/en/legacy-findings/<topic>.md` or an addendum to
    `docs/en/m3-extrinsics-from-noetic.md`
- [ ] **A5: launch exclusivity declared**
  - The new `odom_bringup_launch.py` and the old `localization_launch.py`
    must not be launched together (they would both publish `base_link ->
    imu_link`). README states this explicitly

## 6. Issue breakdown

Four Issues. Each is sized to build/launch cleanly on its own.

### Issue M4R-1: Add `/whill/odom` publisher to the `Iruazu/ros2_whill` fork (case 1)

- **Goal**: Cut a new branch `feature/add-odom-publisher` on the existing
  `Iruazu/ros2_whill` fork (already pinned in `whill_lab.repos`; M2 cold-boot
  init patch precedent) and extend `whill_node.cpp::OnStatesModelCr2Timer()`
  to compute and publish `nav_msgs/Odometry` on `/whill/odom`. Then PR-merge
  on the fork → tag → update `whill_lab.repos` to the new tag
- **Decision (pinned 2026-06-14)**:
  - **Case 1 adopted** (fork patch). **Case 2 (new `whill_odometry` package)
    rejected**: a wrapper node in this repo splits state into two nodes and
    breaks the "driver publishes state" contract. The case-1 fork-patch
    pattern is already proven by M2, and upstream (whill-labs/ros2_whill) is
    tracked via the `upstream` remote (the background is to be migrated into
    the follow-up ADR-0002)
  - **Convention re-interpretation**: parent §3.4 and §5's "no edits to
    `src/third_party/`" targets the **local** checkout of `src/third_party/`
    in this repo. Forking an upstream GitHub repo and pointing
    `whill_lab.repos` at the fork URL is orthogonal to that rule. The
    `Iruazu/ros2_whill` fork already exists and carries the M2 patch
    (`whill_lab.repos`)
- **Units and signs (pinned in Issue #30)**:
  - `motor_angle` = rad, `motor_speed` = km/h (upstream `whill.cpp:62-69`
    comments)
  - Wrap handling: ROS 2 standard `angles::shortest_angular_distance()`
    (`ros-humble-angles`)
  - Odometry method: **angle-based** (robust against the ~3 Hz publish rate)
  - Sign convention: **negate the right wheel**
    (`d_right = -angles::shortest_angular_distance(...)`,
    `d_left = angles::shortest_angular_distance(...)`)
  - `WHEEL_RADIUS = 0.1325 m`, `TREAD = 0.496 m` (nominal; adopted in
    `docs/en/m4r-whill-units.md`)
  - Full C++ skeleton: `docs/en/m4r-whill-units.md` "Transcribing into M4R-1"
- **Acceptance**:
  - [ ] Open PR on the `Iruazu/ros2_whill` fork with branch
    `feature/add-odom-publisher`
  - [ ] Add `<depend>angles</depend>`, `<depend>nav_msgs</depend>`,
    `<depend>tf2</depend>`, `<depend>tf2_geometry_msgs</depend>` to
    `whill_driver/package.xml`
  - [ ] Add `find_package(angles REQUIRED)` (and the same for `nav_msgs`,
    `tf2`, `tf2_geometry_msgs`) to `whill_driver/CMakeLists.txt` and link
    `angles::angles`
  - [ ] Create the `/whill/odom` (`nav_msgs/Odometry`) publisher in
    `whill_node.cpp` and compute the angle-based odometry in
    `OnStatesModelCr2Timer()` (based on the code skeleton in
    `docs/en/m4r-whill-units.md`)
  - [ ] After the fork PR is merged, attach an annotated tag
    `humble-with-odom-2026-MM-DD`
  - [ ] Update the `third_party/ros2_whill:` `version:` in `whill_lab.repos`
    to the new tag; `vcs import` then pulls the new fork
  - [ ] `colcon build --packages-up-to whill_driver` succeeds
  - [ ] `ros2 topic echo /whill/odom --once` returns a `nav_msgs/Odometry`
    message
  - [ ] On-chair joystick: 1 m forward causes
    `/whill/odom.pose.pose.position.x` to grow by +1 m ± 5% (positive sign,
    distance integrity)
  - [ ] On-chair joystick: 90° left turn causes
    `/whill/odom.pose.pose.orientation` yaw to grow by +π/2 ± 5% (REP-103
    convention)
- **Out of scope**:
  - EKF integration (M4R-3)
  - TF publication (M4R-1 is `publish_tf: false`-equivalent; TF is owned by
    the EKF)
  - New `whill_odometry` package in this repo (case 2 rejected)
  - Other modifications to `whill_node.cpp` (the cold-boot init patch is kept
    as-is)
- **Assumptions**:
  - Issue #30 completed (verification results recorded in
    `docs/en/m4r-whill-units.md`)
  - Issue #28 completed (Phase A-C freeze finalised; archive is for reference
    only)
  - WHILL Model CR2 hardware is available
- **Branch (this repo)**: `m4r/1-fork-add-odom-publisher` (this repo only
  carries the `whill_lab.repos` bump; the body of the patch lives on the
  fork)
- **Fork branch**: `Iruazu/ros2_whill/feature/add-odom-publisher`

### Issue M4R-2: Replace base_link static TFs with measured values

- **Goal**: Replace the three identities in
  `whill_sensors_bringup/static_tf_launch.py` with measured values
- **Acceptance**:
  - [ ] `base_link -> imu_link` / `base_link -> velodyne` /
    `base_link -> camera_link` are non-zero
  - [ ] `ros2 run tf2_tools view_frames` shows the 4-link tree
  - [ ] An addendum in `docs/en/m3-extrinsics-from-noetic.md` records
    "physical definition of base_link (which point is zero)" and "derivation
    path of the three extrinsics"
- **Out of scope**: Full re-calibration of the camera. If M3-era measurement
  is insufficient, take a simple tape-measure value and move on; full
  re-calibration waits until M5-R's map-quality requirement forces it
- **Assumptions**: The noetic LiDAR↔IMU value is `extrinsic_T = LiDAR_in_IMU`.
  If we put `base_link` at "rear-axle centre, ground level", we derive
  `base_link -> imu_link` from the noetic note that IMU sits 0.324 m above
  and 0.412 m behind the chair's reference
- **Branch**: `m4r/2-base-link-static-tf`

### Issue M4R-3: robot_localization EKF (ekf_odom) introduction

- **Goal**: Run the EKF with `/whill/odom` + `/imu/data_raw` as inputs,
  produce the `odom -> base_link` TF and `/odometry/filtered`
- **Acceptance**:
  - [ ] `ros2 topic hz /odometry/filtered` is 30 Hz ± 5 Hz
  - [ ] `ros2 run tf2_tools view_frames` shows `odom -> base_link` published
    by the EKF (the fork-patched `whill_driver` itself does *not* publish TF)
  - [ ] 10-m straight push: end-position error ≤ 0.5 m (criterion A2)
- **Out of scope**: navsat / map frame / two-stage EKF. This Issue keeps a
  single-stage EKF with `world_frame: odom` and `two_d_mode: true` (the
  campus indoor is effectively planar)
- **Assumptions**: With the RT 9-axis driver's raw IMU
  (`orientation_covariance[0] = -1`), the EKF must consume only
  `angular_velocity_*` and `linear_acceleration_*` from `imu0`, not
  `roll/pitch/yaw`. Yaw comes from `/whill/odom`
- **Branch**: `m4r/3-ekf-odom`

### Issue M4R-4: Retire `tf_bridge_launch.py` + new bringup launch + docs

- **Goal**: Clear parent §5 prohibition 1 and provide the canonical launch
  for the new TF structure
- **Acceptance**:
  - [ ] `src/whill_navigation/launch/tf_bridge_launch.py` is deleted
  - [ ] The corresponding include in `src/whill_navigation/launch/nav_launch.py`
    is removed (nav_launch.py remains broken; its restoration is M6-R, noted
    in README)
  - [ ] New `whill_localization/launch/odom_bringup_launch.py` brings up
    sensors + whill_driver (the `Iruazu/ros2_whill` fork that now publishes
    `/whill/odom` after M4R-1) + EKF in one command
  - [ ] `docs/m4r-bench-data/README.md` (template) and the 10-m straight test
    procedure ship together
  - [ ] CLAUDE.md "in-progress known issues" entry P4 is updated to "resolved"
    at the end of this phase (separate commit; this Issue prepares the draft)
- **Out of scope**: nav_launch.py Nav2 restoration (M6-R)
- **Assumptions**: `whill_driver` runs via upstream
  `whill_bringup/launch/whill_launch.py`. A symlink such as `/dev/whill`
  would be cleaner than `/dev/ttyUSB0`; if no udev rule exists yet, add one
  in this Issue
- **Branch**: `m4r/4-bringup-and-retire-tf-bridge`

## 7. Execution order and dependencies

```
M4R-1 (fork patch)
   │
   ├──> M4R-3 (EKF) ──┐
   │                  │
M4R-2 (static TF) ────┤
   │                  │
   └──────────────────┴──> M4R-4 (bringup launch + retire tf_bridge)
```

- M4R-1 and M4R-2 are independent and can run in parallel (separate branches)
- M4R-3 needs both M4R-1 (the whill_odom topic) and M4R-2 (the base_link TF)
- M4R-4 needs M4R-3 done

Realistic call for one developer sharing one chair: serial order
M4R-1 → M4R-2 → M4R-3 → M4R-4. Parallelising makes EKF debugging painful
(is the problem whill_odom or the TF?).

## 8. Verification strategy

### 8.1 Per-Issue on-chair verification

| Issue | What the user runs |
|-------|--------------------|
| M4R-1 | (1) Joystick-spin one full rotation in place; read cumulative yaw via `ros2 topic echo /whill/odom`. (2) Push 1 m straight; compare `pose.position.x` with the tape value |
| M4R-2 | In RViz, fix `base_link`; each sensor frame should sit at the chair's physical mounting position |
| M4R-3 | (1) `ros2 topic hz /odometry/filtered` at 30 Hz. (2) 10-m straight push: end-position error ≤ 0.5 m. (3) Static for 30 s: yaw drift ≤ 0.1 rad |
| M4R-4 | A single `ros2 launch whill_localization odom_bringup_launch.py` brings everything up; `view_frames` shows the M4-R-complete TF tree |

### 8.2 Bench data convention

Save under `docs/m4r-bench-data/<YYYY-MM-DD>-<run>/`:

- `bag/` (ros2 bag: `/whill/states/model_cr2`, `/whill/odom`, `/imu/data_raw`,
  `/odometry/filtered`, `/tf`, `/tf_static`)
- `README.md` (date, chair, floor, operation, measured end-position error)
- Raw bag files are gitignored (existing CLAUDE.md rule); only README and PDFs
  are committed

### 8.3 Rationale for the 10-m straight pass threshold

- Factory wheel-odometry accuracy on Model CR2 is roughly ±2-3% (no internal
  docs cross-checked; industry-standard range. The exact value can be
  requested from the vendor but is not needed for M4-R pass/fail)
- IMU fusion benefit is limited on straight pushes (yaw estimate is a
  weighted average of wheel-derived yaw and IMU `angular_velocity_z`, which
  is ~0 during straight motion; wheel-side dominates)
- Floor slip on indoor carpet tile is ≤ 1%
- Total: ≤ 0.5 m / 10 m = 5% as the pass line, ≤ 0.3 m as the signal that
  wheel calibration is good

This threshold is only used for M4-R standalone pass/fail. Dispatch-service
(M9) accuracy requirements are defined separately under the map frame once
M6-R's scan-to-map correction is in place.

## 9. Risks and uncertainties

### 9.1 Risks

- **TF restructuring breaks `nav_launch.py`**. Known and intentional. Nav2
  will not launch at M4-R completion. The first M6-R task restores
  nav_launch.py. Alternative (renaming to `nav_launch.py.disabled`) is
  rejected because hiding the breakage is riskier than the explicit break
- **Unit ambiguity in ModelCr2State** (closed by Issue #30): `motor_angle` =
  rad and `motor_speed` = km/h, pinned via the upstream `whill.cpp:62-69`
  comments and on-chair verification (details in
  `docs/en/m4r-whill-units.md`). The risk is closed; the entry stays here
  for reference
- **Tread value inconsistency** (provisionally pinned by Issue #30): URDF
  0.520 m vs the `whill_node.cpp:115` comment 0.496 m. The adopted value is
  the latter (0.496 m), captured in the "adopted" column of
  `docs/en/m4r-whill-units.md`. Re-measure only if the M4R-1 1-m straight
  test or 90° turn exceeds tolerance
- **Bag record structure changes**. Pre-M3 bags are FAST-LIO-centric. After
  M4-R, bags include `/whill/odom` and `/odometry/filtered`. Pre-M3 bags
  remain replayable via `fast_lio_launch.py` (compatibility preserved).
  `docs/m4r-bench-data/README.md` documents the new layout so post-M4-R
  bags do not mix with pre-M3
- **`whill_driver` TF publication**. Re-verified: `whill_node.cpp` does not
  publish TF (good — EKF stays the sole TF source). The risk is that a
  future upstream version adds TF. The bringup launch should pass
  parameters that suppress upstream TF if it ever appears (defensive line)
- **Reconciling EKF `world_frame: odom` with the absence of `map`**. Nobody
  publishes `map` in M4-R. RViz Fixed Frame must be switched to `odom`;
  screenshots during this period should annotate "Fixed Frame: odom"

### 9.2 Uncertainties

- **Physical definition of base_link**. We provisionally pick "rear-axle
  centre, ground level" but may have to revisit when M5-R's map origin or
  Nav2's footprint forces a different choice. M4R-2 records the choice with
  a comment so we can re-derive
- **IMU sign convention**. Whether the RT 9-axis mounting orientation has
  carried over identically from noetic to humble is unverified. During the
  M4R-1 joystick spin, also check that `/imu/data_raw.angular_velocity.z`
  matches the sign of the yaw rate derived from `/whill/odom`

## 10. Hand-off to subsequent phases

At M4-R completion:

- TF: `odom -> base_link -> {imu_link, velodyne, camera_link}` single chain
- Inputs: `/whill/odom`, `/imu/data_raw` (continuing from M3)
- Output: `/odometry/filtered` (continuous, smooth, 30 Hz)
- Launch: `whill_localization/launch/odom_bringup_launch.py`

With that:

- **M5-R**: Map building does *not* use the M4-R launch (offline bag
  post-processing). The benefit is that bags recorded under the M4-R TF
  tree make the resulting PCD map consistent with the runtime TF
- **M6-R**: A scan-to-map localizer publishes `map -> odom`. The M4-R
  `odom -> base_link` continues to be owned by the EKF, so `map -> odom`
  jumps are absorbed by the continuous `odom -> base_link` (the REP-105
  intent). Initial-pose UI lands in M6-R. The full restoration of
  `nav_launch.py` and the obsolescence of `tf_bridge_launch.py` complete
  in M6-R

## 11. ADR candidates

Technical decisions worth recording from this phase:

- **ADR-0002: `/whill/odom` supply architecture (case 1 = fork patch
  adopted)** — pinned by user judgement 2026-06-14. The full ADR body
  (background, alternatives considered, and the convention re-interpretation)
  will be filed as `docs/decisions/0002-whill-odom-supply.md` as a follow-up
- [ ] **ADR-0003 candidate: Handling of the GPS-datum path** (see §3.A).
  Filed as a separate post-M4-R Issue

## 12. Next actions

With this plan now `accepted` (Iruazu, 2026-06-14), the remaining work to
kick off M4-R is:

1. Open the four Issues via `gh issue create` for M4R-1 through M4R-4 (this
   plan defines the breakdown; the actual `gh issue create` is a separate
   step)
2. Start M4R-1 (case 1 fork patch). The verification skeleton is already in
   `docs/en/m4r-whill-units.md` and Issue #30 is closed
3. Issue M4R-2 (base_link static TF replacement) can run in parallel with
   M4R-1 on a separate branch
4. After M4R-1 and M4R-2 land on main, M4R-3 (EKF) becomes ready; M4R-4
   (retire `tf_bridge_launch.py` + bringup launch) closes the phase
