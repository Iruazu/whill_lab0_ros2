# whill_localization

Localization bringup for the WHILL chair stack. As of M4-R this package
covers **two distinct layers** that must not be run together:

- The M4-R **odom-layer EKF** (`robot_localization`), authoritative for
  `odom -> base_link` and `/odometry/filtered`. New surface added by
  Issue #37; this is what Nav2 and any future map-based localizer will
  consume.
- The pre-existing **FAST-LIO** bringup, retained for M5-R map-making
  use only. Its runtime use as a localizer is frozen (see
  `docs/ja/plans/2026-06-11-platform-pivot.md` §3, §5).

This package provides:

- A chair-tuned FAST-LIO config (`config/velodyne_whill.yaml`) — built
  by carrying the calibrated LiDAR↔IMU extrinsic and the IMU noise
  parameters forward from the noetic stack and aligning topic names
  with what `whill_sensors_bringup` publishes.
- An M4-R EKF config (`config/ekf_odom.yaml`) — `robot_localization`'s
  `ekf_node` configured to fuse `/whill/odom` (wheel) +
  `/imu/data_raw` (RT 9-axis IMU) into `/odometry/filtered`.
- Four launch files:
  - `odom_bringup_launch.py` — **M4-R single-command bringup (Issue
    #38)**. Composes `whill_sensors_bringup/sensors_launch.py` +
    upstream `whill_bringup/whill_launch.py` + this package's
    `ekf_odom_launch.py`. This is the launch you should use on the
    chair for everything M4-R covers.
  - `ekf_odom_launch.py` — odom-layer EKF only. Use this when you want
    to bring sensors and the WHILL driver up by hand (debugging) or
    swap one of the inputs (replay a `/whill/odom` bag while live
    sensors run, etc.).
  - `fast_lio_launch.py` — FAST-LIO node alone, for offline replay
    against a recorded rosbag (defaults `use_sim_time:=true`).
  - `localization_launch.py` — sensors (via `whill_sensors_bringup`)
    plus FAST-LIO, for live operation on the chair. **Mutually
    exclusive with `odom_bringup_launch.py` / `ekf_odom_launch.py`**:
    both branches would try to author the `odom -> base_link` TF edge
    (the FAST-LIO branch via the pre-Issue-#38 `tf_bridge_launch.py`
    aliases, the EKF branch directly) and the resulting TF fight
    produces unbounded jitter that breaks downstream Nav2 / RViz. Pick
    one branch per session. The FAST-LIO branch is retained only as a
    map-making prerequisite for M5-R, not as a runtime localizer.

## M4-R odom-layer EKF (Issue #37)

`ekf_odom_launch.py` starts a single `robot_localization` `ekf_node` and
nothing else. It is the **only** publisher of the `odom -> base_link`
TF edge in the new architecture; the WHILL driver was switched to
TF-off in M4R-1, and `tf_bridge_launch.py` (FAST-LIO `map -> camera_init`
identity) was removed by M4R-4 / Issue #38.

Inputs (sourced from elsewhere; this launch does NOT start them):

- `/whill/odom` (`nav_msgs/Odometry`, ~2.5 Hz, frame `odom`, child
  `base_link`) — M4R-1 `whill_driver` output. Only `vx`, `vy`, `vyaw`
  are fused; the driver's pose is intentionally discarded so the EKF
  is the single integrator (see header comment in `ekf_odom.yaml`).
- `/imu/data_raw` (`sensor_msgs/Imu`, 100 Hz, frame `imu_link`) — RT
  9-axis IMU. `angular_velocity` (vyaw effective under `two_d_mode`)
  and `linear_acceleration` (ax, ay effective) are fused; orientation
  is skipped because `orientation_covariance[0] == -1` (REP-145
  "unknown").
- Static TF `base_link -> imu_link` from
  `whill_sensors_bringup/launch/static_tf_launch.py` (M4R-2).
  `robot_localization` uses this to rotate IMU measurements into
  `base_link` before fusing.

Outputs:

- `/odometry/filtered` (`nav_msgs/Odometry`) at 30 Hz.
- TF `odom -> base_link` at 30 Hz.

How the four Issue #37 acceptance criteria map to runtime checks:

| AC | Command (live) | Expected |
|----|---------------|----------|
| (1) 30 Hz output | `ros2 topic hz /odometry/filtered` | 30 Hz ± 5 Hz |
| (2) EKF is the only TF publisher | `ros2 run tf2_tools view_frames` then check `frames.pdf`; also `ros2 run tf2_ros tf2_echo odom base_link` | edge `odom -> base_link` shows publisher `ekf_filter_node`; `whill_driver` does not appear as a TF source |
| (3) 10 m straight push, ≤ 0.5 m end error | record a bag of `/odometry/filtered`, then `scripts/m4r3_ekf_bench.py <bag>` | `End distance from start: ≤ 0.5 m` |
| (4) 30 s static, ≤ 0.1 rad yaw drift | record a bag with the chair static, then `scripts/m4r3_ekf_bench.py <bag>` | `Yaw drift: ≤ 0.1 rad` |

Quick start (live, unified M4-R bringup — single terminal):

```bash
ros2 launch whill_localization odom_bringup_launch.py
```

This is the M4R-4 / Issue #38 path: sensors + `whill_driver` + EKF
launch together; do not mix it with `localization_launch.py` (FAST-LIO)
in the same session.

For debugging individual pieces the three-terminal variant from M4R-3
is still available. **This variant is mutually exclusive with
`odom_bringup_launch.py` and with `whill_safety/m6r_bringup_launch.py`**
— starting any of those three-terminal pieces alongside a full
bringup duplicates that node. 2026-07-16 field: `sensors_launch.py`
started alongside `m6r_bringup_launch.py` produced `/velodyne_points`
at 39.4 Hz (4×) and a RealSense USB contention loop.

```bash
# Terminal 1: sensor drivers + static TFs
ros2 launch whill_sensors_bringup sensors_launch.py

# Terminal 2: WHILL driver (assumes M4R-1 wiring; topic /whill/odom)
ros2 launch whill_bringup whill_launch.py

# Terminal 3: this EKF
ros2 launch whill_localization ekf_odom_launch.py
```

Covariance tuning is intentionally left at upstream defaults for M4-R.
If the four acceptance criteria pass with defaults the run-tuning issue
stays in M5-R / later; if not, a follow-up Issue will own the
`process_noise_covariance` and `initial_estimate_covariance` rework
based on captured run data.

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
- `odom → base_link` is now provided by the M4R-3 EKF
  (`ekf_odom_launch.py` + `config/ekf_odom.yaml`, Issue #37). `map →
  odom` is the M6-R scan-to-map localizer's job; FAST-LIO is no longer
  a runtime localizer candidate per platform-pivot.md §5.
- Save a PCD map by setting both `pcd_save.pcd_save_en: true` **and**
  `publish.map_en: true` (the latter is what actually populates
  `pcl_wait_pub`; `pcd_save_en` alone does not because the historical
  in-loop save path in upstream FAST-LIO's `publish_frame_world` is
  commented out). With both flags on, call
  `ros2 service call /map_save std_srvs/srv/Trigger` to dump
  `pcl_wait_pub` to `map_file_path`. Used in M5-b.
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
