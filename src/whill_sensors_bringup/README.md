# whill_sensors_bringup

M3 sensor stack bringup for WHILL: Velodyne VLP-16, Intel RealSense
D435, and the RT 9-axis USB IMU. Wraps the three Group A upstream
drivers and adds:

- **Lifecycle-aware IMU launch** — the upstream `rt_usb_9axisimu_driver`
  is a `LifecycleNode`, so a plain `ros2 run` produces no topics until
  it is manually transitioned. This package's `imu_launch.py` drives
  `configure → activate` automatically.
- **Static TF tree rooted at `base_link`** — wires `base_link →
  imu_link / velodyne / camera_link` so the LiDAR, IMU, and camera no
  longer appear as orphan frames in `view_frames`.

## Quick start

**Standalone use is for sensor-only smoke tests.** This launch is
transitively included by `whill_localization/odom_bringup_launch.py`
and `whill_safety/m6r_bringup_launch.py`. Do NOT run it in parallel
with either — 2026-07-16 field measured `/velodyne_points` at 39.4 Hz
with a doubled bringup, plus RealSense USB device contention. For the
full M6-R stack use `m6r_bringup_launch.py` in ONE terminal.

Sensor-only test:

```bash
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
ros2 launch whill_sensors_bringup sensors_launch.py
```

To also start the D435 (opt-in, default off — see §RealSense below):

```bash
ros2 launch whill_sensors_bringup sensors_launch.py realsense:=true
```

In a second terminal:

```bash
ros2 topic list
ros2 run tf2_tools view_frames
```

## What is launched

| Action | Source | Effect |
|--------|--------|--------|
| `velodyne-all-nodes-VLP16-launch.py` (with `/scan` → `/scan_raw` remap) | `velodyne` (Group A upstream) | `/velodyne_points`, `/scan_raw` — see `sensors_launch.py` docstring for the rename rationale |
| `rs_launch.py` (opt-in via `realsense:=true`, default off) | `realsense2_camera` (Group A upstream) | `/camera/camera/color/...`, `/camera/camera/depth/...` — only when the launch arg is flipped on |
| `imu_launch.py` | this package | `/imu/data_raw`, `/imu/mag`, `/imu/temperature` (after auto `configure → activate`) |
| `imu_sign_corrector` (spawned by `imu_launch.py`) | this package | `/imu/data_rep145` — `/imu/data_raw` with `linear_acceleration.{x,y,z}` negated (Issue #56) |
| `static_tf_launch.py` | this package | `base_link → imu_link / velodyne / camera_link` |

## Expected TF tree

```
base_link
├── imu_link               (static, from this package)
├── velodyne               (static, from this package)
└── camera_link            (static, from this package)
    ├── camera_depth_frame → camera_depth_optical_frame   (from realsense2_camera)
    └── camera_color_frame → camera_color_optical_frame   (from realsense2_camera)
```

## IMU sign correction (Issue #56)

The RT 9-axis IMU's inner board (PCMK-G3X = MPU-9250 + LPC1343F USB
firmware) reports `linear_acceleration` as the gravity-acceleration
vector itself (z ≈ -9.81 at rest, +Z up), not as the REP-145 specific
force (z ≈ +9.81 at rest). The upstream `rt_usb_9axisimu_driver` is a
byte-passthrough and does not correct this. `imu_sign_corrector` is a
small rclpy node spawned by `imu_launch.py` that subscribes to
`/imu/data_raw` and republishes the message verbatim — except
`linear_acceleration.{x,y,z}` are negated — to `/imu/data_rep145`. All
downstream consumers (the `robot_localization` EKF in
`whill_localization`, and the future scan-to-map localizer) MUST
subscribe to `/imu/data_rep145`; `/imu/data_raw` is preserved as raw
passthrough for backward compatibility and debugging.

## Open items

- M4R-2 (Issue #36) replaced the three identity placeholders with
  measurement-based values. `base_link` is defined provisionally as the
  rear-axle midpoint projected to the ground plane; the derivation and
  the numeric values are in
  [`../../docs/ja/m3-extrinsics-from-noetic.md`](../../docs/ja/m3-extrinsics-from-noetic.md).
  Status:
  - `base_link → imu_link` — measured (Issue #61, 2026-06-24) to
    (0.38, -0.03, 0.47); re-evaluation conditions are in
    `m3-extrinsics-from-noetic.md` "再評価のタイミング".
  - `base_link → camera_link` rotation (currently RPY=0) — needs a
    target-based recalibration in M6-R; today's value is good enough
    for `view_frames` and rough RViz overlay only.

## Launch arguments

`sensors_launch.py` exposes:

- `realsense` (default `false`) — start the Intel RealSense D435
  driver. Off by default because the D435 is not consumed by the M6-R
  runtime stack today, and USB 2.1 enumeration issues have burned
  launch cycles when the camera is plugged in but unused. Flip to
  `true` only for camera-specific smoke tests.

`imu_launch.py` exposes:

- `port` (default `/dev/imu`) — serial path for the IMU. The repo udev
  rule (`udev/99-whill-stack.rules`) creates this symlink from VID:PID
  `2b72:0003`.
- `frame_id` (default `imu_link`) — TF frame populated in IMU messages.

Override per-launch with:

```bash
ros2 launch whill_sensors_bringup imu_launch.py port:=/dev/ttyACM0 frame_id:=imu
```

## RealSense (opt-in)

The D435 is currently unused by the M6-R runtime stack; camera-based
localization or perception is not on the roadmap until M7+. Reasons
to keep it off by default:

- USB 2.1 enumeration issues have caused the D435 to appear on
  `lsusb` but fail to start via `rs_launch.py`, burning a launch
  cycle before the failure is visible in RViz.
- When two processes try to claim the D435 (e.g. accidentally
  starting `sensors_launch.py` while `m6r_bringup_launch.py` is
  already running), the second one goes into an error loop that
  drowns other launch output.

To enable it explicitly:

```bash
ros2 launch whill_sensors_bringup sensors_launch.py realsense:=true
# or, threaded through the full bringup:
ros2 launch whill_safety m6r_bringup_launch.py site:=campus realsense:=true
```
