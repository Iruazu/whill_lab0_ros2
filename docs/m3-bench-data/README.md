# M3 bench data

Captured artifacts from on-bench / on-chair smoke tests of the M3 sensor
stack. Bag directories themselves are gitignored (they are large), only the
small descriptive files (READMEs, `frames.pdf` snapshots, etc.) live in
git history.

## 2026-05-07 — first all-three-sensors smoke test (D435, VLP-16, RT IMU)

### Environment

- Host: development laptop, Ubuntu 22.04, ROS 2 humble
- WHILL Model CR2 powered on, `/dev/whill` symlink in place
- VLP-16 cabled to host USB-Ethernet adapter (`enx00e04c6808dc`,
  `192.168.1.100/24`), LiDAR responding at `192.168.1.201`
- D435 (serial `938422070073`) on USB 3.2, 5 Gbps
- RT 9-axis IMU on `/dev/imu`

### Drivers exercised

```bash
# in three separate terminals (or via background launch)
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
ros2 run rt_usb_9axisimu_driver rt_usb_9axisimu_driver \
    --ros-args -p port:=/dev/imu -p frame_id:=imu_link
ros2 lifecycle set /rt_usb_9axisimu_driver configure
ros2 lifecycle set /rt_usb_9axisimu_driver activate
ros2 launch realsense2_camera rs_launch.py
```

### Topic / rate observations

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | ~10 Hz (9.8) | 1824 × 16 organized point cloud, `frame_id: velodyne` |
| `/scan` | `sensor_msgs/LaserScan` | ~10 Hz | one ring extracted by `velodyne_laserscan_node`, `frame_id: velodyne` |
| `/imu/data_raw` | `sensor_msgs/Imu` | ~100 Hz | `frame_id: imu_link`. `orientation_covariance[0] = -1` (driver does not fuse orientation — raw gyro/accel only). FAST-LIO consumes this topic name directly. |
| `/imu/mag` | `sensor_msgs/MagneticField` | ~100 Hz | magnetometer, `frame_id: imu_link` |
| `/imu/temperature` | `sensor_msgs/Temperature` | ~100 Hz | |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | 640 × 480 RGB8 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 30 Hz | 848 × 480 Z16 |

### TF snapshot — `frames-2026-05-07.pdf`

The `view_frames` capture shows the RealSense subtree only:

```
camera_link
├── camera_depth_frame → camera_depth_optical_frame
└── camera_color_frame → camera_color_optical_frame
```

`velodyne` and `imu_link` appear as topic `frame_id`s but are **not yet
connected** to a common TF root. Wiring `base_link → velodyne / imu_link /
camera_link` is the remaining M3 task — to be authored as
`whill_sensors_bringup/launch/sensors_launch.py` (or by porting
`tf_imus`), with extrinsics inherited per
[`m3-extrinsics-from-noetic.md`](../m3-extrinsics-from-noetic.md).

### Rosbag — `m3_smoke_2026-05-07/` (gitignored, 437 MiB)

7.83 s capture, 2115 messages across the topics above plus `/tf_static`.
Stored locally for replay / regression checks; not committed because of
size. Reproduce with:

```bash
ros2 bag record -o m3_smoke_$(date +%Y-%m-%d) \
  /velodyne_points /imu/data_raw /imu/mag \
  /camera/camera/color/image_raw \
  /camera/camera/depth/image_rect_raw \
  /tf_static
```

### Lessons captured for downstream milestones

- **IMU is a lifecycle node.** Plain `ros2 run` leaves it in `unconfigured`
  state with no published topics. Bringup must explicitly drive
  `configure → activate` (e.g. via a `LifecycleNode`-aware launch action,
  or a small wrapper script).
- **`/imu/data_raw` matches FAST-LIO's expected topic name** out of the
  box, so the M4 LIO config inherited from
  [`m3-extrinsics-from-noetic.md`](../m3-extrinsics-from-noetic.md) does
  not need a remap on the IMU side.
- **Velodyne driver publishes with sensor-data QoS** (best-effort).
  `ros2 topic hz` in humble does not auto-detect that — confirm with
  `ros2 topic echo --once` or in code with a matching subscription QoS.
- **RealSense topics are namespaced `/camera/camera/...`** by `rs_launch.py`
  defaults (parent ns `camera`, node name `camera`). M4 / M5 launch files
  should remap or accept this prefix instead of the bare `/camera/`.

---

## 2026-05-07 (later) — first chair-mounted bringup + drive

After moving the laptop, RealSense, USB-Ethernet adapter, and IMU from
the bench to their chair mounts (with the user seated in the WHILL CR2),
re-ran the M3 bringup and recorded a static + a 60 s drive bag.

### USB topology survived the move

The chair-mount cable routing pushed the RealSense and the RTL8153
USB-Ethernet adapter from Bus 002 to Bus 004, but `/dev/whill` and
`/dev/imu` still resolved to the right tty devices on first try — the
VID:PID-based udev rule (`udev/99-whill-stack.rules`) is doing what it
was designed to do.

```
WHILL    /dev/whill -> ttyUSB0   (Prolific 067b:2303, Bus 003)
IMU      /dev/imu   -> ttyACM0   (RT 2b72:0003, Bus 003)
D435     Bus 004 (was Bus 002 on the bench)
RTL8153  Bus 004 (was Bus 002 on the bench)
Velodyne carrier=1, ping 192.168.1.201 0% loss
```

### IMU race condition between `on_configure` and `on_activate`

The first two attempts to `ros2 launch whill_sensors_bringup
sensors_launch.py` both produced an immediate `readSensorData() returns
FAILURE` and bounced the IMU back to `unconfigured`. Direct `cat /dev/imu`
showed the device itself was producing healthy ASCII frames, so the
problem was in the bringup, not the hardware.

Cause: with the original `imu_launch.py`, the `OnStateTransition →
inactive → activate` chain fired ~22 ms after `on_configure` returned.
At 100 Hz the device emits a frame every 10 ms, but `on_configure` only
opens the serial port — there is no guarantee a complete frame is
already in the kernel buffer when `on_activate`'s very first
`readSensorData()` call runs. If it isn't, the driver returns FAILURE
and lifecycle bounces to `errorprocessing`.

Fix: insert a `TimerAction(period=1.5)` between the `inactive`-reached
event and the `activate` event in `imu_launch.py`. After the change the
log now reads:

```
on_configure() is called.
reached 'inactive', waiting 1.5 s before activate ...
on_activate() is called.
ros2 lifecycle get /rt_usb_9axisimu_driver  →  active [3]
```

This pattern (configure-then-wait-then-activate) likely also applies
to other USB-serial / CDC-ACM lifecycle drivers; keep it in mind for M4.

### `m3_chair_static_2026-05-07/` — 11.85 s static reference bag

Captured with the user seated and the chair stationary (`655 MiB`, 3193
messages). Same topic set as the 2026-05-07 bench bag plus the four
`/tf_static` entries from `static_tf_launch.py`. Useful as a "noise
floor" baseline for IMU bias and PointCloud2 stability.

### `m3_chair_motion_2026-05-07/` — 64.5 s drive bag (FAST-LIO test data)

Captured while the user joystick-drove the chair (3 s static at the
start for IMU bias, then ~60 s slow forward + turns). `3.5 GiB`,
17412 messages.

| Topic | Type | Count | Rate |
|-------|------|-------|------|
| `/imu/data_raw` | `sensor_msgs/Imu` | 6452 | 100.0 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | 6452 | 100.0 Hz |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 636 | 9.86 Hz |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | 1934 | 29.98 Hz |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 1934 | 29.98 Hz |
| `/tf_static` | `tf2_msgs/TFMessage` | 4 | latched |

This is the canonical FAST-LIO replay input for M4 — replay it with
`ros2 bag play m3_chair_motion_2026-05-07 --clock` against an
unconfigured FAST-LIO node to dial in the LiDAR↔IMU extrinsic before
moving the chair again.
