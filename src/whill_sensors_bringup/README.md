# whill_sensors_bringup

M3 sensor stack bringup for WHILL: Velodyne VLP-16, Intel RealSense
D435, and the RT 9-axis USB IMU. Wraps the three Group A upstream
drivers and adds:

- **Lifecycle-aware IMU launch** — the upstream `rt_usb_9axisimu_driver`
  is a `LifecycleNode`, so a plain `ros2 run` produces no topics until
  it is manually transitioned. This package's `imu_launch.py` drives
  `configure → activate` automatically.
- **Velodyne self-filter** (Phase M5-e) — `scripts/velodyne_self_filter.py`
  strips chair-body / mount-frame returns inside a cylinder before
  FAST-LIO sees them. Offline geometry calibration via
  `scripts/analyze_velodyne_arc.py`.
- **robot_state_publisher with chair + sensor URDF** — `urdf/whill_with_sensors.urdf.xacro`
  wraps the upstream `whill_description/urdf/whill_model_cr2.urdf`
  with `imu_link`, `velodyne`, `camera_link`, and `base_footprint`
  using the noetic-inherited LiDAR-IMU extrinsic as initial values
  (Phase A-1). Replaces the old `static_tf_launch.py` of identity
  placeholders.

## Quick start

```bash
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
ros2 launch whill_sensors_bringup sensors_launch.py
```

In a second terminal:

```bash
ros2 topic list
ros2 run tf2_tools view_frames
```

## What is launched

| Action | Source | Effect |
|--------|--------|--------|
| `velodyne-all-nodes-VLP16-launch.py` | `velodyne` (Group A upstream) | `/velodyne_points`, `/scan` |
| `rs_launch.py` | `realsense2_camera` (Group A upstream) | `/camera/camera/color/...`, `/camera/camera/depth/...` |
| `imu_launch.py` | this package | `/imu/data_raw`, `/imu/mag`, `/imu/temperature` (after auto `configure → activate`) |
| `robot_state_publisher_launch.py` | this package | chassis URDF + sensor frames published as `/tf_static` |
| `velodyne_self_filter.py` node | this package | `/velodyne_points_filtered` (cylinder cut of chair-body returns) |

## Expected TF tree

```
base_link
├── base_floor → wheels / seat / sensor arms / backrest / arms     (from upstream whill_description URDF)
├── base_footprint        (ground projection, z = -0.1175 m)
├── imu_link              (xyz = 0.20, 0.00, 0.42; pitch = +0.286 rad ≈ +16.4°)
└── velodyne              (xyz = 0.42, 0.36, 0.78; pitch = +0.157 rad ≈ +9°)
    └── camera_link       (xyz = 0, 0, -0.103 in velodyne; depth/color subtree from realsense2_camera)
```

The numerical values for `imu_link`, `velodyne`, and `camera_link` are
xacro:property entries at the top of `urdf/whill_with_sensors.urdf.xacro`.
Re-measure on the chair and edit those properties — do not patch the
expanded URDF.

The `imu_link` pitch comes from a 5 s static measurement on confirmed
flat floor (2026-05-30, 500 samples on `/imu/data_corrected`,
roll = -0.20°, pitch = +16.37°). The IMU board is bolted to a sloped
bracket; FAST-LIO's `grav_align` absorbs the tilt for SLAM, but
downstream EKF and Nav2 consumers want the URDF to reflect the real
mount geometry. See `docs/session-2026-05-30.md` for the diagnosis.

## Open items

- **Camera extrinsic**: `camera_link` is parented to `velodyne` with
  the bracket-only offset (~10 cm in z). A tape-measure session will
  refine this if RViz overlays drift relative to the LiDAR cloud.
- **`static_tf_launch.py` deprecated**: the file is still present for
  reference but is no longer included by `sensors_launch.py`. Slated
  for removal once Phase A-1 has been validated on the chair.
- **Gyro X bias**: 2026-05-30 static measurement showed a -1.14 deg/s
  bias on the IMU's X axis. Independent of the URDF — see task #36
  for the planned `imu_sign_flip` enhancement.

## Launch arguments

`imu_launch.py` exposes:

- `port` (default `/dev/imu`) — serial path for the IMU. The repo udev
  rule (`udev/99-whill-stack.rules`) creates this symlink from VID:PID
  `2b72:0003`.
- `frame_id` (default `imu_link`) — TF frame populated in IMU messages.

Override per-launch with:

```bash
ros2 launch whill_sensors_bringup imu_launch.py port:=/dev/ttyACM0 frame_id:=imu
```
