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

## Open items

- The three top-level static transforms are placeholder identities.
  Replace them with calibrated extrinsics before relying on this TF
  tree for FAST-LIO (M4) or Nav2 (M5):
  - `base_link → velodyne` ← LiDAR↔IMU values inherited from the
    noetic stack are captured in
    [`../../docs/m3-extrinsics-from-noetic.md`](../../docs/m3-extrinsics-from-noetic.md).
  - `base_link → camera_link` ← needs a one-shot measurement; the
    camera is rigidly mounted to the LiDAR via the support frame.

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
