# M3 — Sensor stack on ROS 2 humble

## Goal

Bring up the perception sensors mounted on the WHILL — the Velodyne LiDAR,
the Intel RealSense camera(s), and the 9-axis IMU — under ROS 2 humble, with
the right TF tree so M4 (localization) and M5 (navigation) can consume them
without any per-sensor remapping.

End state of M3: each sensor is launchable in isolation, each publishes the
expected ROS 2 topic, and the static TF chain
`base_link → <sensor>_link` is in place. No fusion, no costmaps, no
localization yet — those are M4/M5.

## Scope (in M3)

- Replace each Group A sensor package with its official ROS 2 upstream and
  pin it in `whill_lab.repos`.
- Port `tf_imus` (Group B) — the only Group B package M3 needs — so IMU
  frames are published correctly.
- Author per-sensor launch files under a new `whill_sensors_bringup`
  package and a top-level `sensors_launch.py` that fires all of them.
- Verify each topic on the bench and on the chair, capturing one-shot
  rosbags as evidence.

## Out of scope (defer to later milestones)

- Sensor fusion / EKF — M4
- Camera-LiDAR extrinsic calibration — utility, run once when mounts change
- `velodyne_camera_calibration` — kept as a one-shot tool, not a runtime
  dependency

## Hardware

The chair carries the same physical sensors that the noetic stack used —
confirmed by the user 2026-05-07 — so model and topology are fixed:

| Sensor | Model | Interface | Notes |
|--------|-------|-----------|-------|
| LiDAR | Velodyne **VLP-16** | UDP 2368, 10 Hz (RPM 600), unicast | `frame_id: velodyne`, no IP override on the noetic side |
| Depth camera | Intel RealSense **D455** | USB 3 | No project-specific launch on the noetic side — was driven with upstream defaults |
| IMU | RT 9-axis (`rt_usb_9axisimu_driver`) | USB | May share `/dev/ttyUSB*` numbering with WHILL — `udev` rule needed |

Source of truth for the noetic values:
[`whill_lab0/FAST_LIO/config/velodyne.yaml`](https://github.com/Iruazu/whill_lab0/blob/main/FAST_LIO/config/velodyne.yaml)
and [`whill_lab0/velodyne-mast/velodyne_pointcloud/launch/VLP16_points.launch`](https://github.com/Iruazu/whill_lab0/blob/main/velodyne-mast/velodyne_pointcloud/launch/VLP16_points.launch).

Topic conventions inherited from the noetic stack (M4/M5 expect these):

- `/velodyne_points` — VLP-16 point cloud
- `/imu/data_raw` — RT 9-axis raw IMU (FAST-LIO input)
- RealSense topics: upstream defaults (`/camera/...`)

LiDAR↔IMU extrinsic calibration carried over from the noetic stack is
captured in [m3-extrinsics-from-noetic.md](m3-extrinsics-from-noetic.md) so
M4 can reuse it.

## Upstream packages (Group A)

Pinned in [`whill_lab.repos`](../whill_lab.repos):

| Package | URL | Pinned ref | Notes |
|---------|-----|------------|-------|
| `velodyne` | `ros-drivers/velodyne` | tag `2.5.1` | Latest 2.x release on the `ros2` line |
| `realsense-ros` | `IntelRealSense/realsense-ros` | tag `4.55.1` | Mature humble baseline; needs `librealsense2` system pkg |
| `rt_usb_9axisimu_driver` | `rt-net/rt_usb_9axisimu_driver` | branch `humble-devel` | Vendor's official humble branch |

### System dependencies

The following apt packages are needed before the first `colcon build` —
all of them resolve through the standard ROS 2 / Ubuntu archives, so the
Intel-published apt repo is **not** required for our use:

```bash
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-diagnostic-updater \
  ros-humble-librealsense2 \
  ros-humble-launch-pytest \
  python3-tqdm \
  libpcap0.8-dev
```

`ros-humble-librealsense2` (currently 2.57.7) is the ROS-packaged build of
Intel's `librealsense2`, sufficient for the D455 driver to find headers
and libraries at build time. If a future requirement forces a newer
librealsense than the ROS package ships, switch to Intel's apt repo at
that point — until then, the standard repos are enough.

## Custom packages (Group B in this milestone)

| Package | Source | Action |
|---------|--------|--------|
| `tf_imus` | noetic `whill_lab0/tf_imus` | port to ament; expose `imu_link` static transforms |

Other Group B packages (`sensor`, etc.) are deferred to M4/M5 unless
inspection reveals a runtime dependency from this milestone's launch
files.

## Procedure (planned)

1. Add the three upstream URLs to `whill_lab.repos`, run
   `./scripts/import_upstream.sh`.
2. `colcon build --packages-up-to whill_sensors_bringup`.
3. Per-sensor smoke tests on the bench (LiDAR pointing at a wall, RealSense
   at a textured surface, IMU still on a desk):
   - `ros2 topic hz /velodyne_points`
   - `ros2 topic hz /camera/depth/image_rect_raw`
   - `ros2 topic hz /imu/data`
4. TF check: `ros2 run tf2_tools view_frames` after launching everything,
   confirm `base_link` is the parent of all three sensor frames.
5. Repeat (3) and (4) on the actual chair, USB/power as expected.
6. One-shot rosbag for each sensor, archived under `docs/m3-bench-data/`
   (gitignored if large).

## Status

| Step | Status |
|------|--------|
| Hardware inventory confirmed (same as noetic stack) | done |
| `velodyne` pinned in `whill_lab.repos` | done |
| `realsense-ros` pinned in `whill_lab.repos` | done |
| `rt_usb_9axisimu_driver` pinned in `whill_lab.repos` | done |
| System apt deps installed on the host | done (2026-05-07) |
| `vcs import` + `colcon build` for sensor packages | done — 15/15 packages clean (2026-05-07) |
| `tf_imus` ported to ament | pending |
| `whill_sensors_bringup` package created | pending |
| Per-sensor topic verified on the bench | pending (needs hardware) |
| TF tree verified | pending (needs hardware) |
| Per-sensor rosbag captured on the chair | pending (needs hardware) |

## Open questions (still pending hardware access)

- Whether the IMU enumerates as `/dev/ttyUSB1` (same family as the WHILL
  `/dev/ttyUSB0`) — if so, write a `udev` rule to give it a stable name to
  avoid rotation between ports across reboots.
- Whether RealSense D455 fits within the available USB 3 bandwidth alongside
  the WHILL USB-serial cable on the host's USB controllers.
- Confirm the IMU's actual published topic and remap to `/imu/data_raw` if
  the upstream driver publishes elsewhere (FAST-LIO expects `/imu/data_raw`).
