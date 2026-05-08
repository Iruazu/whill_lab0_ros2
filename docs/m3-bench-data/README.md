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

### `m3_chair_static_2026-05-07/` — 19.85 s static reference bag

Captured with the user seated and the chair stationary (`1.1 GiB`,
5362 messages, 19.85 s). Same topic set as the 2026-05-07 bench bag
plus the four `/tf_static` entries from `static_tf_launch.py`.

A first 11.85 s capture taken right before this one was discarded —
the user noticed it likely contained some unintended residual motion.
The redo was sanity-checked from the IMU stream itself (1986 samples
of `/imu/data_raw`, see `scripts/check_static.py` for the script):

| metric | value |
|--------|-------|
| `\|omega\|` max / RMS | **0.0256 / 0.0200 rad/s** — within typical RT IMU gyro noise + bias floor |
| `omega_x` mean | -0.0198 rad/s — DC gyro bias (FAST-LIO will estimate this on startup) |
| `omega_y` / `omega_z` RMS | 0.0019 / 0.0013 rad/s — pure noise floor |
| `\|a\|` RMS | **10.00 m/s²** — clean gravity vector |
| Linear-acc components | `z=-9.70, y=-2.40, x=-0.40` → IMU mounted at ~14° tilt on the chair |
| Motion bursts (gyro RMS > 0.05 rad/s in 0.2 s windows) | **0 of 99** — no detectable motion the entire bag |

i.e. the bag is clean enough to use as the reference noise floor and
as the warm-up region for FAST-LIO bias estimation in M4.

### `m3_chair_motion_2026-05-07/` — 96.85 s drive bag (FAST-LIO test data)

Captured while the user joystick-drove the chair: 5 s static at the
start (IMU bias warm-up) followed by ~90 s of motion through a hallway
and a room with chairs and desks, returning toward the start point for
loop-closure evaluation. `5.3 GiB`, 26137 messages.

| Topic | Type | Count | Rate |
|-------|------|-------|------|
| `/imu/data_raw` | `sensor_msgs/Imu` | 9687 | 100.0 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | 9687 | 100.0 Hz |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 951 | 9.82 Hz |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | 2904 | 29.98 Hz |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 2904 | 29.98 Hz |
| `/tf_static` | `tf2_msgs/TFMessage` | 4 | latched |

This is the canonical FAST-LIO replay input for M4 — replay it with
`ros2 bag play m3_chair_motion_2026-05-07 --clock` against an
unconfigured FAST-LIO node to dial in the LiDAR↔IMU extrinsic before
moving the chair again.

**Leading 5 s static window verified** with `scripts/check_static.py
docs/m3-bench-data/m3_chair_motion_2026-05-07 5.0`: 0 of 25 windows
flagged as motion bursts, gyro RMS 0.020 rad/s and DC bias on omega_x
of -0.020 rad/s — consistent with the dedicated static bag, so FAST-LIO
will get a clean bias estimation window before the chair starts moving.

A previous 64.5 s drive captured immediately before this one was
discarded — the user noted afterwards that the time budget had
slipped (motion vs static phases not as cleanly separated as
intended). The 96.85 s redo above replaces it.

---

## 2026-05-08 — M4 reproducibility study

Recorded three back-to-back 60 s drives along the same simple route
(approximately a corridor + a turn into a room and back) at WHILL
speed mode 2. Goal: quantify how reliably FAST-LIO can localise the
chair from the bench point of view, given sensors that are not
re-calibrated between runs.

Each run was: kill all nodes → `ros2 launch whill_localization
localization_launch.py` (fresh FAST-LIO state) → wait 12 s for IMU
configure → activate → kdtree init → 60 s `ros2 bag record` of the
sensor topics + the live `/Odometry` and `/tf` → kill the launch.

Bags (gitignored, on disk only):

| File | Notes |
|------|-------|
| `m4_chair_live_2026-05-08_run1/` | static start was contaminated — user mistimed the recording start vs their phone stopwatch |
| `m4_chair_live_2026-05-08_run2/` | clean static start, no dynamic obstacles in view |
| `m4_chair_live_2026-05-08_run3/` | a pedestrian crossed the front-left FOV during the second half of the drive |

Live `/Odometry` was sparse (~1.5 Hz instead of the expected ~10 Hz):
the live `fastlio_mapping` was running concurrently with RViz, the
RealSense / Velodyne / IMU drivers, the static TF broadcasters, and
`ros2 bag record` writing 3.3 GiB to disk. The host became
CPU-bound; offline replay against just `fast_lio_launch.py` recovers
the full ~7 Hz output.

### Replay outcomes (with the final config — see `whill_localization/README.md`)

| Run | Total path | Max disp | End pose | Loop-closure error | Verdict |
|-----|-----------|----------|----------|-------------------|---------|
| run1 | 887 m | 841 m | (-783, +174, +252) | 95 % | diverges from the start; the ill-formed static window starves the iESKF of an IMU bias estimate |
| **run2** | **40.78 m** | **14.46 m** | **(-6.65, +3.18, -1.24)** | **18 %** | **bounded; trajectory shape matches the recorded drive** |
| run3 | 679 m | 631 m | (-409, +471, +101) | 93 % | diverges as the pedestrian enters the front-180° hemisphere — moving features in the world map break registration |

Replaying run2 a second time reproduces the bounded shape with end
pose (-11.0, +2.2, -2.7) m, 50.81 m path, 22.7 % loop-closure error
— i.e. across two replays of the same bag, the same FAST-LIO
binary produces ~10 m end-pose variation. That residual is from
FAST-LIO's multi-threaded kdtree updates being non-deterministic;
the trajectory shape and rough magnitude are reproducible.

### Lessons captured for M4 / M5

- **Capture quality dominates** under FAST-LIO. With this config a
  clean 60 s drive is fine; a contaminated static window or an
  unannounced pedestrian in front of the LiDAR breaks localisation
  irrecoverably. Future drives should announce themselves and run
  during quieter intervals.
- **The 2026-05-07 "identity extrinsic worked" finding was a
  multi-threading lucky outcome.** Running the same `m3_chair_motion`
  bag with identity extrinsics multiple times today produced
  divergence in every retry — yesterday's bounded result happened
  once and could not be reproduced. The noetic-inherited
  `extrinsic_T = [0.104, 0.412, 0.324]` matches the user-confirmed
  physical layout (LiDAR mounted on the chair's left, IMU under the
  seat cushion ~30 cm below) and is the right value going forward.
- **Loose `gyr_cov` is needed** because joystick-driven sharp turns
  produce angular rates the upstream default (0.1) over-trusts the
  IMU prediction for. 0.5 was the smallest value that kept the
  filter stable through the t ≈ 30 s in-place rotation in run2;
  tighter values reproduced today's "diverges at the first turn"
  failures.
