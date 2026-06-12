# Development policy: architectural pivot toward a dispatch platform

Language: [日本語](../../ja/plans/2026-06-11-platform-pivot.md) | [English](2026-06-11-platform-pivot.md)

- Date: 2026-06-11
- Status: accepted (user-approved on 2026-06-11; on the same day reflected into CLAUDE.md's Import section, architecture-layer figure, known-issues list, and team-composition table)
- Intended location: `docs/en/plans/2026-06-11-platform-pivot.md`
- Audience: every Claude Code session and subagent working in this repository.
  When `pm-orchestrator` drafts a phase plan, this document is its top-level input.
  Where CLAUDE.md defines "conventions and file locations," this document defines
  "what to build, in what order, and which directions are off-limits."

## 0. Background

The 2026-06 technical survey (SLAM / localization methods, Tsukuba Challenge
finisher configurations, licensing, compute) and a diagnosis of the current
implementation jointly establish the following:

- The current M5-a TF bridge (`map -> camera_init` pinned to identity) is an
  approximation that only holds under the demo conditions of "short duration,
  attended operation, and start from the mapping origin."
- As run3 demonstrated (FAST-LIO diverged when a pedestrian crossed the
  sensor), this approximation is structurally incompatible with the real goal
  of "operation in an environment with people moving through it."
- Stacking M5-d (goal-following) and M5-e (tuning) on top of that
  approximation is the "wrong direction" we suspected. We must stop adding
  features and replace the foundation.

## 1. North Star (the final product)

A campus autonomous-mobility platform where the rider holds a tablet and uses
a web-based platform app to:

1. specify a destination (dispatch target), and
2. summon an empty chair (the chair drives unmanned to pick them up).

Working backwards from that picture, the driving stack must satisfy the
following six requirements. Every subsequent design choice must be defensible
against this table.

| ID | Requirement | What in the North Star drives it |
|----|-------------|----------------------------------|
| R1 | Startup and re-localization from an arbitrary point | To answer a summon, the chair must self-determine "where on the map I am right now." The implicit assumption of a fixed startup pose does not hold. |
| R2 | A persistent map frame | Dispatch destinations are stored as named points on a map. A configuration whose coordinate frame changes per session cannot define those destinations. |
| R3 | Self-localization that survives long, repeated operation | Several minutes per ride, many rides per day. Drift-accumulating odometry (without correction) is unacceptable. |
| R4 | Unmanned-driving safety (divergence detection, automatic / remote stop) | Summoning means driving with zero riders aboard. We cannot assume a human will notice the anomaly and stop the chair with the joystick. |
| R5 | Dynamic avoidance of pedestrians | The campus has people walking through it. Relying only on a static map for driving is not allowed. |
| R6 | A clear operational boundary (API) with the web side | Between the tablet UI and ROS 2 we need a boundary that can carry authentication, state sync, and job management. |

## 2. Current problems (diagnostic summary)

Detailed evidence is in the 2026-06 diagnostic report; the key points are
copied here. The IDs are referenced from the phase definitions below.

| ID | Problem | Evidence | Conflicting requirements |
|----|---------|----------|--------------------------|
| P1 | No correction path for runtime self-localization. With `map -> camera_init` pinned to identity, FAST-LIO drift (measured 18% / 60s) becomes map-frame error directly. | `tf_bridge_launch.py` | R2, R3 |
| P2 | No initial-pose alignment mechanism. Because camera_init equals the startup pose, the chair is offset from the start unless it boots at the mapping origin in the same orientation. | `tf_bridge_launch.py`, the origin assumption in `pcd_to_occupancy_grid.py` | R1 |
| P3 | Divergence is neither detected nor recovered. A single pedestrian can cause divergence (run3 measured), yet TF keeps flowing and Nav2 keeps driving. | `whill_localization/README.md` run3 | R4 |
| P4 | No odom frame; the wheel odometry is unused. There is no cushion against jumps when correction is introduced, and no backup when LiDAR degrades. | `nav2_params.yaml` comments, CLAUDE.md known issues | R3, R4 |
| P5 | Map-quality issues are cascading to disable safety features. Ghost obstacles -> `use_collision_detection: false`; QoS mismatch -> obstacle layer absent. Pedestrians during operation never appear in the costmap. | `nav2_params.yaml` comments | R5 |

## 3. Adopted architecture (decisions)

### 3.1 Two-phase separation

Separate the "map-building phase (offline, high precision)" from the
"operational phase (online localization)." This is the standard configuration
common to Tsukuba Challenge finishers, commercial delivery robots, and
WHILL's own field-trial research, and it also lowers the required
on-vehicle-PC spec (the heavy global optimization of mapping can be done
offline on the workstation from bags).

```
[Map building (offline, workstation)]
  Manually drive and record a bag
    -> SLAM with loop closure to produce a point-cloud map
    -> Dynamic-object removal (delete pedestrian traces)
    -> Save static PCD + 2D occupancy grid under docs/maps/<site>/

[Operation (online, on-vehicle)]
  Saved map + scan-to-map localization provides map -> odom
  Wheel odometry + IMU EKF provides odom -> base_link
  Nav2 handles planning and path following
  Dispatch gateway translates web jobs into Nav2 actions
```

### 3.2 TF and responsibilities (REP-105 compliant)

```
map -> odom        : localizer (scan-to-map correction; jumps allowed)
odom -> base_link  : robot_localization EKF (/whill/odom + /imu/data_raw; continuous and smooth)
base_link -> sensors: static TF with measured extrinsics (replaces today's identity placeholders)
```

- FAST-LIO is repurposed as a "map-building tool." Its use and reinforcement
  as a runtime localizer is frozen (reason: it cannot structurally resolve
  P1-P3).
- The two identity transforms in `tf_bridge_launch.py` are retired when M4-R
  completes.

### 3.3 Candidate selection (first choice / alternatives / rationale)

| Role | First choice | Alternatives | Rationale |
|------|--------------|--------------|-----------|
| Map-building SLAM | GLIM (MIT, official ROS 2 humble support, offline on a GPU workstation) | FAST-LIO SAM / li_slam_ros2 (VLP-16 track record; bridge and comparison) | Global optimization and interactive map editing. Permissive license. |
| Dynamic-object removal | ERASOR family | Removert | Fast; preserves static points. Offline, so no on-vehicle constraints. |
| Operational localization | lidar_localization_ros2 (NDT_OMP) | hdl_localization / Autoware ndt_scan_matcher / FAST_LIO_LOCALIZATION family | Tsukuba Challenge 2024 track record. Designed to be used with an odometry constraint. |
| Odom fusion | robot_localization EKF | — | Nav2 standard. /whill/odom has already worked since M2. |
| Failsafe | A small custom node (matching-score / covariance monitor -> gate cmd_vel) | — | Minimal implementation of R4. When selecting the localizer, include the presence of reset mechanisms (the inflation-reset idea from emcl2 / mcl_3dl) in the evaluation axes. |
| Dynamic obstacles | A QoS bridge such as pointcloud_to_laserscan -> revive the obstacle layer -> restore `use_collision_detection: true` | — | Resolves P5. The minimum bar for R5. |

Overturning a selection requires an ADR (`docs/decisions/`).

### 3.4 Licensing policy

- Keep the operational stack (the parts that run on the vehicle and might be
  distributed in the future) composable from permissive (MIT / BSD / Apache)
  components.
- GPL-family code (FAST-LIO family and similar) is restricted to use as an
  "offline map-building tool" via process separation. Note that any artifact
  that modifies sources, includes headers, or links against such code becomes
  GPL itself.
- Keep `src/third_party/` excluded from the tree (vcs import + gitignore) and
  keep "no copy-pasted code from non-BSD-3-Clause sources" as a code-reviewer
  check item.
- Once delivering artifacts to a company becomes concrete, create a LICENSE
  inventory table for each third_party upstream under `docs/` and consult the
  university's intellectual-property office.

### 3.5 Platform-layer boundary

- On the ROS side, add a new package `whill_dispatch` (working name). Its
  gateway node is responsible for
  (a) resolving named (semantic) waypoints,
  (b) managing the dispatch-job queue (accept, run, cancel),
  (c) issuing Nav2 `NavigateToPose` actions and relaying progress,
  (d) publishing vehicle state (pose, battery, driving state).
  The web side knows nothing beyond this boundary.
- The first-choice web connection is rosbridge_suite (websocket). When
  authentication and multi-vehicle management come into scope, decide via ADR
  whether to replace it with an independent gateway API such as FastAPI.
- The tablet app may live in a separate repository. This repository's
  responsibility ends at the API boundary.

## 4. Milestone redefinition

The old M5-d / M5-e are frozen and replaced by the phases below. The detailed
plan for each phase is produced by `pm-orchestrator` under `docs/plans/`,
using this document as input, when the phase starts.

| Phase | Content | Problems resolved | Lead agent |
|-------|---------|-------------------|------------|
| M4-R | Odom-foundation rebuild: robot_localization EKF (/whill/odom + IMU), TF re-wiring, applying measured extrinsics for base_link -> sensor, retiring tf_bridge | P4, part of P2 | ros2-implementer |
| M5-R | Map pipeline: introduce GLIM (or FAST-LIO SAM), dynamic removal with ERASOR, an artifact convention under `docs/maps/<site>/` (pcd + pgm + yaml + capture metadata) | P5 (map quality) | research-analyst -> ros2-implementer |
| M6-R | Operational localization + Nav2 reintegration: introduce a scan-to-map localizer, an initial-pose workflow, the failsafe node, and revive the obstacle layer with collision detection re-enabled | P1, P2, P3, P5 (safety) | ros2-implementer + debugger |
| M7 | Dispatch API layer: whill_dispatch (named waypoints, job queue, NavigateToPose wrapper, state publishing) and rosbridge connectivity | R6 | pm-orchestrator -> ros2-implementer |
| M8 | Tablet web app: map view, destination selection, summon UI | R6 | (separate repository allowed) |
| M9 | Integration verification: on-vehicle validation including unmanned-summon driving; check physical E-stop and remote stop | R4 | debugger + user on real hardware |

Rationale for the order: M4-R goes first because, without an odom frame,
introducing correction in M6-R would deliver jumps directly to the
controller. Each phase is cut into a unit that can be verified
independently.

## 5. Rules of conduct for Claude Code (absolute under this policy)

Prohibited:

1. Adding new features that assume the identity-`tf_bridge_launch.py`
   configuration (including continuing the old M5-d).
2. Reinforcing FAST-LIO as a runtime localizer. (Re-tuning parameters is
   allowed only when the goal is to improve map-building quality.)
3. Adding autonomous-driving features while `use_collision_detection: false`
   remains in effect.
4. Coupling dispatch / web-layer logic tightly inside Nav2 or localization
   nodes. (The boundary lives in whill_dispatch as defined in 3.5.)
5. (Re-stating existing rules.) Editing `src/third_party/`, or copy-pasting
   GPL code.

Required:

1. At the start of every phase, `pm-orchestrator` reads this document and
   expands phase / acceptance criteria / risks into `docs/plans/` before
   asking the user for approval.
2. Decisions that overturn or modify the selections in 3.3 are recorded as
   ADRs.
3. Acceptance criteria are written as observable commands with expected
   values (the granularity in section 6 is the minimum bar).
4. Verification that requires real hardware (WHILL / Velodyne / RealSense /
   IMU) is handed to the user (existing convention).

## 6. Acceptance criteria (per phase, observable)

- M4-R:
  - `ros2 run tf2_tools view_frames` shows a single chain `map -> odom -> base_link`.
  - `/odometry/filtered` is published from wheel + IMU, and a 10 m manually
    pushed straight line lands within the acceptable terminal error
    (concrete value set in the phase plan).
  - tf_bridge_launch.py is removed and the build / launch still succeeds.
- M5-R:
  - On a map built from a closed-loop drive bag, start and end overlap on
    visual inspection (within tens of cm).
  - Even on a bag where pedestrians crossed, the occupancy grid after
    removal has no "tail-like" residuals.
  - `docs/maps/<site>/` contains pcd, pgm, yaml and metadata (capture date,
    route, weather).
- M6-R:
  - Localization converges from an RViz initial-pose hint, and one campus
    loop produces zero TF jumps above threshold.
  - Self-localization does not break when a pedestrian crosses the sensor
    (reproduction test under conditions equivalent to run3).
  - In a test where matching score is artificially degraded, the failsafe
    gates cmd_vel.
  - The obstacle layer reflects a person ahead in the costmap, and driving
    proceeds with `use_collision_detection: true`.
- M7:
  - A dispatch job to a named waypoint is issued over websocket, the chair
    drives, and completion is reported back.
  - The web side can subscribe to the chair's pose and state during
    operation.
- M8:
  - On a real tablet, the sequence "select a point -> summon -> arrive" is
    operable.
- M9:
  - Unmanned-summon drives succeed N times in a row (N is set in the phase
    plan); physical E-stop and remote stop both function.

## 7. Open items (ADR candidates)

- [ ] ADR: Final selection of the map-building SLAM. Since the GPU
      workstation is now confirmed (section 9), GLIM's hardware prerequisite
      is met. Settle after a real-bag comparison of GLIM vs FAST-LIO SAM.
- [ ] ADR: Final selection of the localizer (lidar_localization_ros2 vs
      hdl_localization). Include reset mechanism and initial-pose UX as
      evaluation axes.
- [ ] ADR: Web connection method (rosbridge direct vs an independent
      gateway API) and authentication scheme.
- [ ] ADR: Safety requirements for unmanned operation (physical E-stop,
      remote stop, speed limit, whether a supervisor is required). Include
      the interface to the university's safety review process.
- [ ] ADR: GNSS/RTK integration for outdoor extension (including the
      switching scheme under covered sections).

## 8. Reflection into CLAUDE.md (to be done after acceptance)

1. Replace the "architecture layer" figure with the two-phase separation
   from 3.1 / 3.2 and the REP-105 configuration.
2. Update "ongoing known issues" to be based on P1-P5. (Note that the
   "FAST-LIO lacks loop closure" and "wheel odometry not integrated" items
   now have a resolution path under this policy.)
3. Prepend "policy decisions: first consult this document" as a row at the
   top of the "team composition" table.
4. Add `@docs/plans/2026-06-11-platform-pivot.md` to the Import section.

## 9. Development hardware (confirmed 2026-06-11)

Alienware x15 R2 (hostname systemlab-Alienware-x15-R2):

- CPU: Core i9-12900H (14 cores, 20 threads) / RAM: 32 GiB / SSD: 2 TB
- GPU: NVIDIA installed (the model needs confirmation via `nvidia-smi`. The
  GNOME Settings reading stopping at "NVIDIA Corporation / Mesa Intel
  Graphics" is a sign that the NVIDIA proprietary driver is either not
  installed or not active. GLIM's GPU mode requires the driver and CUDA.)
- OS: Ubuntu 22.04.5 LTS — matches the requirement of ROS 2 humble
  (jammy).

Implications:

- A single machine covers both tier B (on-vehicle operation) and tier C
  (mapping workstation) of the spec guideline. GLIM's hardware
  prerequisite is met. NVIDIA driver and CUDA setup should be planned as a
  predecessor task in M5-R.
- The 1.4 Hz /Odometry problem during live operation is corroborated by
  this CPU as not "underpowered" but rather "the combined load of record +
  RViz + all drivers running at once." Keep the policy of removing record
  / RViz from the operational launch.
- The development phase is completed on this single machine. Splitting off
  a power-efficient on-vehicle PC (mini PC / Jetson) is a problem for M9
  onwards; no investment is made now.

The lab's Jetson TX2 Developer Kit (2017; Pascal 256 CUDA cores; Denver2 x2
+ Cortex-A57 x4; RAM 8 GB; eMMC 32 GB) is excluded from on-vehicle
candidates. Reason: its software support stops at the JetPack 4 series
(Ubuntu 18.04, CUDA 10.2), so it cannot meet ROS 2 humble (which requires
Ubuntu 22.04) or GLIM's GPU requirement (CUDA 12 series). There is a
loophole of running CPU-only humble in Docker, but running localization +
Nav2 on a 6-core ARM with 8 GB shared memory is tight, and paying
engineering cost to an EOL platform is not justified. Treat the TX2 as
teaching material / spare hardware; if M9 calls for splitting off an
on-vehicle PC, procure a new Orin-class device (officially validated by
GLIM) or an x86 mini PC.
