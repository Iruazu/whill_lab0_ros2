# whill_localization

LiDAR-Inertial Odometry (FAST-LIO) bringup for the WHILL chair stack.

This package provides:

- A chair-tuned FAST-LIO config (`config/velodyne_whill.yaml`) — built
  by carrying the calibrated LiDAR↔IMU extrinsic and the IMU noise
  parameters forward from the noetic stack and aligning topic names
  with what `whill_sensors_bringup` publishes.
- Two launch files:
  - `fast_lio_launch.py` — FAST-LIO node alone, for offline replay
    against a recorded rosbag (defaults `use_sim_time:=true`).
  - `localization_launch.py` — sensors (via `whill_sensors_bringup`)
    plus FAST-LIO, for live operation on the chair.

## Quick start (offline replay)

Use the chair-mounted drive bag from M3 as input:

```bash
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash

# Terminal 1
ros2 launch whill_localization fast_lio_launch.py

# Terminal 2
ros2 bag play ~/whill_lab0_ros2/docs/m3-bench-data/m3_chair_motion_2026-05-07 --clock
```

Watch for `/Odometry` updates and a trajectory in RViz.

## Quick start (live)

```bash
ros2 launch whill_localization localization_launch.py
```

This brings up the Velodyne + RealSense + IMU drivers (with the IMU
lifecycle-activated automatically), publishes the `base_link` static TF
chain, and starts FAST-LIO against `/velodyne_points` + `/imu/data_raw`.

## Config diff vs upstream FAST-LIO defaults

The upstream `fast_lio/config/velodyne.yaml` is tuned for VLP-32; the
WHILL stack uses VLP-16 with a chair-mounted IMU at known extrinsics.
Notable overrides:

| Field | Upstream default | `velodyne_whill.yaml` | Reason |
|-------|------------------|----------------------|--------|
| `common.imu_topic` | `/imu/data` | `/imu/data_raw` | RT 9-axis driver publishes raw IMU on `/imu/data_raw` |
| `preprocess.scan_line` | 32 | 16 | VLP-16 has 16 rings |
| `preprocess.timestamp_unit` | 2 (μs) | 0 (s) | velodyne ROS2 driver outputs per-point time in seconds (see `docs/m3-bench-data/README.md`) |
| `preprocess.blind` | 2.0 | 0.5 | inherited from noetic tuning — chair frame causes near returns we don't want to drop |
| `mapping.fov_degree` | 360 | 360 | (matches upstream; the noetic config's 180 was wrong and is not carried forward) |
| `mapping.extrinsic_T/R` | identity | inherited noetic values | calibrated LiDAR↔IMU pose, see `docs/m3-extrinsics-from-noetic.md` |
| `publish.path_en` | false | true | enabled so RViz can show the trajectory polyline |

## Open items

- **Recalibrate the LiDAR↔IMU extrinsic.** The current baseline runs
  with `extrinsic_T/R = identity` because the inherited noetic values
  caused FAST-LIO to diverge on the 2026-05-08 replay (the IMU is now
  mounted at ~14 deg tilt — the noetic 9 deg rotation is approximately
  wrong). Identity produces a bounded trajectory but ~18 % loop-closure
  drift on the M3 motion bag. LI-Init or a similar one-shot extrinsic
  estimator is the next step.
- Wire `map → odom` and `odom → base_link` properly once Nav2 is in
  scope (M5). FAST-LIO publishes `/Odometry` and a `camera_init` frame
  by default — the M5 bringup should remap these into Nav2's expected
  TF tree.
- Save a PCD map by flipping `pcd_save.pcd_save_en` to true and
  rerunning a drive — currently disabled to keep replay runs cheap.

## M4 baseline replay results (2026-05-08)

Played `docs/m3-bench-data/m3_chair_motion_2026-05-07` (96.85 s) against
this package's `fast_lio_launch.py` with `use_sim_time:=true`. FAST-LIO
publishes `/Odometry` at ~6 Hz; the trajectory is bounded and
plausible:

| metric | value |
|--------|-------|
| Start position | (+0.005, -0.000, -0.001) m |
| End position | (+8.911, +1.289, -0.326) m |
| Total path length | 49.99 m |
| Max displacement from start | 16.97 m |
| Loop-closure error (start ↔ end) | 9.00 m  (≈ 18 % of path length) |

The drive went forward through a corridor (peaks around x ≈ +16 m),
turned into the chair / desk room, and was meant to come back near the
start. End-pose drift of 9 m is a clean signal that the pose estimate
itself is fine but the LiDAR↔IMU extrinsic is off — consistent with
the chair-mount tilt change since the noetic-era calibration.
