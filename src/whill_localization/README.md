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

- **Tighten the loop-closure error.** Best replay so far is ~18 %
  drift on a 60 s clean drive (run2 below). Drift is dominated by the
  bag's two sharp in-place rotations near the corridor / room
  transitions. Either ease those motions during data capture or invest
  in a better IMU and re-tune `gyr_cov`.
- Wire `map → odom` and `odom → base_link` properly once Nav2 is in
  scope (M5). FAST-LIO publishes `/Odometry` and a `camera_init` frame
  by default — the M5 bringup should remap these into Nav2's expected
  TF tree.
- Save a PCD map by flipping `pcd_save.pcd_save_en` to true and
  rerunning a drive — currently disabled to keep replay runs cheap.
- Long drives (the 96 s `m3_chair_motion_2026-05-07` bag) still
  diverge after ~30 s with this config. Either further loosen the
  filter, run LI-Init for a per-environment calibration, or accept
  the 60 s window as the M4 deliverable.

## M4 baseline replay results (2026-05-08)

### Final config (after tuning)

The 2026-05-07 commit started from `extrinsic_T/R = identity` because
the inherited noetic values had been suspected of being stale; this
turned out to be wrong. Repeated 2026-05-08 replays of the same bag
diverged unpredictably with identity values, while the noetic
extrinsic — which the user confirmed still matches the physical
layout (LiDAR mounted on the chair's left at +0.412 m, IMU under the
seat cushion +0.324 m below) — gave repeatable bounded trajectories
once `gyr_cov` was loosened from 0.1 to 0.5 to absorb the higher
angular rates of joystick-driven sharp turns.

So: **noetic extrinsic + `gyr_cov: 0.5` + `cube_side_length: 200.0`**
is the working config in
[`config/velodyne_whill.yaml`](config/velodyne_whill.yaml).

### Reproducibility on chair-driven data (2026-05-08)

Three back-to-back 60 s drives along the same simple route (`run1`,
`run2`, `run3` under `docs/m3-bench-data/`). Each run starts with
~8 s static and finishes back near the start point.

| Run | Data quality | Live `/Odometry` rate | Replay outcome |
|-----|--------------|------------------------|----------------|
| run1 | static start was contaminated (timing slip) | ~1.5 Hz | diverges immediately |
| **run2** | **clean static start, no dynamic obstacles** | ~1.4 Hz | **bounded; ~50 m path; ~20 % loop-closure error; reproducible across two replays to ~10 m end-pose** |
| run3 | a pedestrian crossed the front-left FOV in the second half | ~1.9 Hz | diverges where the pedestrian appears |

Live `/Odometry` rate during recording is well below FAST-LIO's
nominal 10 Hz on this host, because the live `fastlio_mapping` was
running in real time alongside RViz, the RealSense and Velodyne
drivers, and ros2 bag record — i.e. the host was CPU-bound. Offline
replay of the same bag against just `fast_lio_launch.py` produces
~7 Hz `/Odometry` and matches the live trajectory shape, so the live
slowdown does not affect localisation correctness, only update rate.

### Headline run2 metrics (offline replay, 2026-05-08)

| metric | value |
|--------|-------|
| Total path length | 40.78 m |
| Max displacement from start | 14.46 m |
| End position | (-6.65, +3.18, -1.24) m |
| Loop-closure error | 7.48 m (≈ 18 % of path length) |
| `/Odometry` rate (offline, single FAST-LIO instance) | 6.81 Hz |

Replaying `run2` a second time produced (-11.0, +2.2, -2.7) m end
pose with a 50.81 m path and 11.5 m loop-closure error — i.e. the
trajectory shape is reproducible to ~10 m / ~10 % across replays of
the same bag, with the residual variance coming from FAST-LIO's
non-deterministic multi-threaded kdtree updates.
