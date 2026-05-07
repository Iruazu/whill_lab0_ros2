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
