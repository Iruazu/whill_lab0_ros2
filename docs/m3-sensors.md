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

## Hardware (to be confirmed on the chair)

| Sensor | Model | Interface | Notes |
|--------|-------|-----------|-------|
| LiDAR | Velodyne (model TBD — VLP-16?) | Ethernet | Power source on the chair TBD |
| Depth camera | Intel RealSense (D435 / D435i?) | USB 3 | Confirm USB hub bandwidth |
| IMU | RT 9-axis (`rt_usb_9axisimu_driver`) | USB | May share `/dev/ttyUSB*` numbering with WHILL — check `udev` rules |

The exact models and bus assignments need to be read off the chair before
the first launch attempt — the noetic-side `whill_lab0` repo contains hints
in its launch files.

## Upstream packages (Group A)

Pin in [`whill_lab.repos`](../whill_lab.repos) once M3 starts:

| Package | URL | Branch |
|---------|-----|--------|
| `realsense-ros` | `IntelRealSense/realsense-ros` | `ros2-master` (current) |
| `velodyne` | `ros-drivers/velodyne` | `ros2` |
| `rt_usb_9axisimu_driver` | upstream repo | `ros2` branch |

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
| Hardware inventory confirmed on the chair | pending |
| `realsense-ros` pinned and built | pending |
| `velodyne` pinned and built | pending |
| `rt_usb_9axisimu_driver` pinned and built | pending |
| `tf_imus` ported to ament | pending |
| `whill_sensors_bringup` package created | pending |
| Per-sensor topic verified on the bench | pending |
| TF tree verified | pending |
| Per-sensor rosbag captured on the chair | pending |

## Open questions (to resolve before opening the M3 PR)

- Velodyne model and IP / multicast configuration on the chair.
- Whether the IMU enumerates as `/dev/ttyUSB1` (same family as the WHILL
  `/dev/ttyUSB0`) — if so, write a `udev` rule to give it a stable name to
  avoid rotation between ports across reboots.
- Whether RealSense fits within the available USB 3 bandwidth alongside the
  WHILL USB-serial cable on the host's USB controllers.
