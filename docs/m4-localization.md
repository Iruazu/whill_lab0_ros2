# M4 — Localization on ROS 2 humble

## Goal

LiDAR-Inertial Odometry on the chair, producing real-time `/Odometry`
plus a `map` (or `camera_init`) frame the next milestone (M5 / Nav2)
can plan against. Re-uses the calibrated LiDAR↔IMU extrinsic from the
noetic stack since the physical mount has not changed.

End state of M4: from a clean 60 s chair drive, FAST-LIO produces a
bounded trajectory roughly tracing the recorded path and a
`camera_init -> body` TF chain that downstream packages can consume
or remap.

## Scope (in M4)

- Pin `hku-mars/FAST_LIO@ROS2` (note: branch name is capitalised) in
  `whill_lab.repos`, plus its hard dep `Livox-SDK/livox_ros_driver2`,
  build them in this workspace.
- Author a `whill_localization` package that wraps `fastlio_mapping`
  with chair-tuned parameters and two launch entry points (offline
  replay, live operation).
- Validate offline against the M3 chair-mounted motion bag and on
  three new live chair drives.

## Out of scope (deferred to M5)

- 2D / 3D map building for goal-based navigation
- `map -> odom -> base_link` TF wiring for Nav2
- `cmd_vel` consumption — the WHILL driver handles motion already
  (M2), but the localization → controller loop is M5 scope

## Build prerequisites

The two upstreams pulled in for M4 do not all build cleanly with
plain `vcs import + colcon build`. Three extra steps are needed (run
once on each new host):

1. **Init Livox SDK 2.x at the system level.**
   `livox_ros_driver2` hard-links against
   `/usr/local/lib/liblivox_lidar_sdk_shared.so` even when running
   with a Velodyne. Build the SDK out of band:

   ```bash
   git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2
   cd /tmp/Livox-SDK2 && mkdir build && cd build
   cmake .. && make -j$(nproc) && sudo make install
   sudo ldconfig
   ```

2. **Substitute Livox driver's ROS-version-specific files.** The
   upstream repo ships `package_ROS1.xml` / `package_ROS2.xml` and
   `launch_ROS1/` / `launch_ROS2/`; its `build.sh` script copies the
   ROS2 variants into place before invoking colcon. Either run that
   script once or do it manually:

   ```bash
   cd src/third_party/livox_ros_driver2
   cp -f package_ROS2.xml package.xml
   [ -d launch ] || cp -r launch_ROS2 launch
   ```

3. **Init FAST_LIO's `ikd-Tree` git submodule.** `vcs import` does
   not recurse into submodules:

   ```bash
   git -C src/third_party/FAST_LIO submodule update --init --recursive
   ```

Then build with the cmake flags Livox expects:

```bash
colcon build --symlink-install \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

These bootstrapping steps will be folded into
`scripts/import_upstream.sh` in a follow-up commit so a fresh fork
builds in one shot.

## Hardware → FAST-LIO inputs

| Source | Topic | Rate | Notes |
|--------|-------|------|-------|
| Velodyne VLP-16 | `/velodyne_points` | 10 Hz | Per-point time field is in seconds (relative to scan end, hence the negative values). `timestamp_unit: 0` matches. |
| RT 9-axis IMU | `/imu/data_raw` | 100 Hz | Raw — orientation field has covariance[0]=-1 indicating "not provided", which is what FAST-LIO wants. |
| `whill_sensors_bringup` | `/tf_static` | latched | The four `base_link → imu_link / velodyne / camera_link` static transforms; FAST-LIO does not use these but Nav2 will. |

The LiDAR↔IMU **extrinsic** comes from the noetic stack and is
unchanged: LiDAR on the chair's left, IMU under the seat cushion
~30 cm below. See
[`docs/m3-extrinsics-from-noetic.md`](m3-extrinsics-from-noetic.md)
for the values and their derivation.

## Config decisions and why

[`src/whill_localization/config/velodyne_whill.yaml`](../src/whill_localization/config/velodyne_whill.yaml)
overrides several upstream defaults:

| Field | Upstream default | M4 value | Why |
|-------|------------------|----------|-----|
| `common.imu_topic` | `/imu/data` | `/imu/data_raw` | RT 9-axis driver publishes raw IMU on `/imu/data_raw` |
| `preprocess.scan_line` | 32 | 16 | VLP-16 has 16 rings |
| `preprocess.timestamp_unit` | 2 (μs) | 0 (s) | velodyne ROS2 driver outputs per-point time in seconds |
| `preprocess.blind` | 2.0 | 0.5 | inherited from noetic — the chair frame produces returns in 0.5–2 m we don't want to drop |
| `mapping.fov_degree` | 360 | 180 | the back hemisphere of the LiDAR sees the chair body and the seated user; both are rigidly attached and would corrupt the world map |
| `mapping.gyr_cov` | 0.1 | **0.5** | upstream is too tight for joystick-driven sharp in-place rotations; 0.1 caused divergence at the first ~30 s turn, 0.5 stays stable |
| `mapping.extrinsic_T/R` | identity | inherited noetic values | calibrated, unchanged from noetic; the IMU is mounted ~14 deg tilted under the seat cushion and the LiDAR ~30 cm above on the chair's left |
| `cube_side_length` | 1000.0 | **200.0** | 1000³/0.5³ voxels overflows pcl::VoxelGrid's int32 indices and surfaces as a "No Effective Points!" stall |
| `publish.path_en` | false | true | enables RViz trajectory visualisation |

## Replay protocol

Offline-only test (chair powered down, sensors disconnected):

```bash
ros2 launch whill_localization fast_lio_launch.py rviz:=false
# in another terminal:
ros2 bag play <bag_dir> --clock \
    --topics /velodyne_points /imu/data_raw /imu/mag /tf_static
```

Live test on the chair:

```bash
ros2 launch whill_localization localization_launch.py
```

Each run should start with at least 5 s of complete stillness so the
iESKF can converge on the IMU bias before the first chair motion.

## Status

| Step | Status |
|------|--------|
| `hku-mars/FAST_LIO` (ROS2 branch) pinned + built | done |
| `Livox-SDK/livox_ros_driver2` pinned + built (incl. SDK install) | done |
| `whill_localization` package authored (config + 2 launches) | done |
| `IncludeLaunchDescription` config-path bug fixed | done |
| Offline replay produces bounded `/Odometry` on a clean bag | done — `m4_chair_live_2026-05-08_run2`, 40 m path, 18 % loop-closure error |
| Live operation on the chair end-to-end | done — `localization_launch.py`, all nodes come up cleanly |
| Reproducibility quantified | done — 2026-05-08 three-run study (`docs/m3-bench-data/README.md`) |
| Config bootstrapping (Livox SDK, submodule, package.xml) folded into `scripts/import_upstream.sh` | pending — folded into M5 wrap-up |
| `map → odom → base_link` TF wiring | M5 scope |
| 2D / 3D saved map for Nav2 | M5 scope |

## Known limits / TODOs

- **Capture quality dominates.** A contaminated static window or a
  pedestrian in the front 180° FOV breaks registration
  irrecoverably. Future drives need an explicit "go" cue and a quiet
  environment.
- **Long drives still diverge.** The 96.85 s `m3_chair_motion`
  replay still walks off after ~30 s with this config. Either ease
  the in-place rotations during capture, run LI-Init for a
  per-environment finer extrinsic, or accept ≤ 60 s drives as the
  M4 deliverable until the M5 testing surfaces a need to push past
  it.
- **Live `/Odometry` is CPU-bound** when `fastlio_mapping` runs
  alongside RViz, three sensor drivers, and `ros2 bag record`. ~1.5 Hz
  vs FAST-LIO's nominal 10 Hz. Trajectory shape is correct, just
  under-sampled. Either trim the live runtime or move FAST-LIO to a
  dedicated host before M5's controller subscribes to `/Odometry`.
- **FAST-LIO is non-deterministic** — same bag, two replays, ~10 m
  end-pose drift. Acceptable for M4 but worth filing upstream.
