# whill_localization

Odom-layer localization bringup for the WHILL chair stack. This package
provides the **M4-R odom-layer EKF** (`robot_localization`), authoritative
for `odom -> base_link` and `/odometry/filtered`. New surface added by
Issue #37; this is what Nav2 and the M6-R scan-to-map localizer consume.

> FAST-LIO note (2026-07-22): this package previously also shipped a
> FAST-LIO runtime-localizer bringup (`fast_lio_launch.py` /
> `localization_launch.py` / `config/velodyne_whill.yaml`). FAST-LIO's
> runtime use was frozen by platform-pivot §5, the runtime localizer is
> now `lidar_localization_ros2` (BSD-2, ADR-0006), and map-making uses
> GLIM (MIT, ADR-0003). The FAST-LIO launches/config and the GPL sources
> were removed to keep the tree permissive.

This package provides:

- An M4-R EKF config (`config/ekf_odom.yaml`) — `robot_localization`'s
  `ekf_node` configured to fuse `/whill/odom` (wheel) +
  `/imu/data_raw` (RT 9-axis IMU) into `/odometry/filtered`.
- Two launch files:
  - `odom_bringup_launch.py` — **M4-R single-command bringup (Issue
    #38)**. Composes `whill_sensors_bringup/sensors_launch.py` +
    upstream `whill_bringup/whill_launch.py` + this package's
    `ekf_odom_launch.py`. This is the launch you should use on the
    chair for everything M4-R covers.
  - `ekf_odom_launch.py` — odom-layer EKF only. Use this when you want
    to bring sensors and the WHILL driver up by hand (debugging) or
    swap one of the inputs (replay a `/whill/odom` bag while live
    sensors run, etc.).

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
launch together.

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

