# Research: campus self-localization / SLAM methods (summary)

Language: [日本語](../../ja/research/2026-06-11-localization-survey.md) | [English](2026-06-11-localization-survey.md)

## Survey date
2026-06-11

## Position of this document

This is the technical basis for `docs/en/plans/2026-06-11-platform-pivot.md`
(the development policy). Where the policy document records "decisions,"
this document keeps the candidate comparison and the detailed diagnosis of
the current implementation. The fully-illustrated human-readable version
(two HTML files) is stored outside the repository; only this summary is
under repository management.

## TL;DR

- The 18% / 60s drift of FAST-LIO is a structural limit of any LIO without
  loop closure. The fix is not to swap methods but to "separate map
  building (offline, with loop closure) from operation (scan-to-map
  localization against the prior map)."
- Tsukuba Challenge finishers predominantly use the configuration "build a
  prior 3D point-cloud map; localize during operation with NDT or MCL."
  REAL_C of Utsunomiya University also finished in 2024.
- Recommendation: map building = GLIM (first choice) or FAST-LIO SAM /
  li_slam_ros2; dynamic removal = ERASOR; operation =
  lidar_localization_ros2 (NDT_OMP). TF follows REP-105 (map -> odom:
  localizer / odom -> base_link: EKF of wheel + IMU).

## Candidate comparison: map-building SLAM / LIO

| Method | humble support | License | Loop closure | VLP-16 track record | Notes |
|--------|----------------|---------|--------------|----------------------|-------|
| GLIM | Official (PPA / Docker, validated on Jetson Orin) | MIT | Yes (global optimization) | Supports spinning LiDARs | GPU recommended; CPU mode available. First choice. |
| FAST-LIO SAM | Primarily ROS1 plus ROS2 derivatives | GPL-family | Yes | Has a velodyne preset | Smallest migration target from the existing FAST-LIO assets. |
| li_slam_ros2 / lidarslam_ros2 | Official | Apache-2.0 | Yes | Yes (casual_walk.bag) | Lightweight. Permissive. |
| LIO-SAM | Community-ported | BSD-3 | Yes | Strong (official datasets provided) | 9-axis IMU required. Watch build memory. |
| LeGO-LOAM | Ported | BSD-3 | Yes | Yes | Has a track record of map building for Tsukuba's Aqua robot. |
| FAST-LIO2 (current) | Community | GPL-2.0 | No | Yes | Demoted to map-building auxiliary / comparison. |
| DLIO / KISS-ICP / Point-LIO / Faster-LIO | Official to community | Mixed MIT / GPL | No | Yes | Odometry-class. Reference. |

## Candidate comparison: operational localization

| Method | humble support | License | Characteristics |
|--------|----------------|---------|-----------------|
| lidar_localization_ros2 | Official | BSD-family | NDT / GICP / NDT_OMP. Operated stably at Tsukuba 2024. Assumes use with an odometry constraint. First choice. |
| hdl_localization | Community | BSD-2 | NDT + UKF (IMU fusion). Tested with VLP16. |
| Autoware ndt_scan_matcher | Official (autoware_core) | Apache-2.0 | Dynamic map loading, regularization. Targets large scale. |
| FAST_LIO_LOCALIZATION family | ROS2 version exists | GPL-family | FAST-LIO plus low-rate scan-to-map correction. Prior-map correction improves ATE by orders of magnitude (arXiv:2402.05540). |
| mcl_3dl | Supported | BSD-3 | 3D particle filter. Reset-tolerant. Finisher track record at Tsukuba (in a configuration that uses FAST-LIO odom). |
| emcl2 | Supported (2D) | — | Inflation-reset recovers from kidnapping. Borrow its design philosophy for the failsafe. |

## Dynamic-environment (pedestrian) handling

- After map building, run ERASOR (RA-L 2021; fast while preserving static
  points) or Removert to delete dynamic traces and produce a static map.
- During operation, scan-to-map is relatively robust because dynamic
  points are a minority. However, as run3 in this repository shows (FAST-LIO
  diverged when a pedestrian crossed), "odometry without correction" is
  fragile against people. This is the direct motivation for the two-phase
  separation.
- Avoidance during operation requires reviving the Nav2 obstacle layer
  (QoS bridge).

## Tsukuba Challenge takeaways (key points)

- About 2 km, with pedestrians, including covered sections where GNSS is
  poor. In 2024, 14 of the 78 entries finished the main run.
- The typical finisher configuration is "prior 3D point-cloud map + NDT or
  MCL." Reports indicate that the wheel-odometry constraint prevented
  breakage at steps and when the LiDAR degraded (AbudoriLab 2024).
- The most common failure cause is self-localization (fuRo Hara, 2018
  survey).

## Diagnosis of the current implementation (P1-P5 details)

The evidence for policy document section 2.

- P1, no correction path: `tf_bridge_launch.py` pins `map -> camera_init`
  to identity. FAST-LIO drift becomes map-frame error directly, so the
  static map turns into a "drifting wall" from Nav2's perspective.
- P2, initial pose: camera_init equals the startup pose. The map origin
  is the mapping drive's startup pose (the origin assumption of
  `pcd_to_occupancy_grid.py`). Starting at the same point in the same
  orientation is an implicit prerequisite, and there is no re-localization
  mechanism.
- P3, divergence undetected: demonstrated by run3. Even after divergence,
  TF keeps flowing and Nav2 keeps driving. There is no matching-score /
  covariance gate, no reset, and no E-stop.
- P4, odom absent: `/whill/odom` (working since M2) is unwired. There is
  no cushion against jumps when correction is introduced, no backup for
  LiDAR degradation, and the rolling local costmap is spinning in the map
  frame.
- P5, map-quality and safety-feature cascade failure: distortion plus
  ghosting forced `use_collision_detection: false`; a QoS mismatch
  removed the obstacle layer. As a result, pedestrians during operation
  never appear in the costmap.

## Licensing notes

- The trigger for obligations is "distribution." In-lab execution,
  analysis, and paper writing are not subject.
- The conventional ROS interpretation is that, with process separation
  and topic communication, GPL nodes do not make custom nodes derivatives.
  Modifying sources, linking, or copy-pasting makes them derivatives.
- Keep the operational stack composable from permissive components
  (MIT / BSD / Apache) (policy 3.4). Keep `src/third_party/` excluded and
  the no-GPL-copy-paste rule in force.

## Compute notes

- VLP-16 produces few points, so operation (localization + Nav2) runs on
  mid-range CPU without GPU.
- The heavy parts are the global optimization in mapping (GPU recommended)
  and offline post-processing. Process bags on the workstation.
- For the result of confirming the development machine and the hardware
  decisions (Alienware x15 R2 doubling as the workstation, Jetson TX2
  excluded), see policy document section 9.

## References (key)

- GLIM: https://github.com/koide3/glim
- lidar_localization_ros2: https://github.com/rsasaki0109/lidar_localization_ros2
- li_slam_ros2: https://github.com/rsasaki0109/li_slam_ros2
- FAST-LIO SAM: https://github.com/engcang/FAST-LIO-SAM
- LIO-SAM: https://github.com/TixiaoShan/LIO-SAM
- hdl_localization: https://github.com/koide3/hdl_localization
- mcl_3dl: https://github.com/at-wat/mcl_3dl
- emcl2: https://github.com/ryuichiueda/emcl2
- ERASOR: https://github.com/LimHyungTae/ERASOR (arXiv:2103.04316)
- Removert: https://github.com/gisbi-kim/removert
- Quantitative effect of prior-map correction: arXiv:2402.05540
- robot_localization: https://github.com/cra-ros-pkg/robot_localization
- Tsukuba Challenge official records: https://tsukubachallenge.jp/

Re-check the latest commit and license of each repository before adoption.
