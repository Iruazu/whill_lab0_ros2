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
| Depth camera | Intel RealSense **D435** | USB 3 (VID:PID `8086:0b07`) | Confirmed by physical side-label and `dmesg` product string 2026-05-07. Earlier handover note saying "D455" was wrong — the actual unit is D435. The same `realsense-ros` driver supports both, so no upstream change is needed. No project-specific launch on the noetic side — was driven with upstream defaults |
| IMU | RT 9-axis (`rt_usb_9axisimu_driver`) | USB CDC-ACM (VID:PID `2b72:0003`) | Enumerates as `/dev/ttyACM*`, not `/dev/ttyUSB*` — does **not** share numbering with WHILL. Stable path `/dev/imu` is provided by the repo-tracked udev rule (see below). |

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
Intel's `librealsense2`, sufficient for the D435 driver to find headers
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
| Stable device paths (`/dev/whill`, `/dev/imu`) via repo-tracked udev rule | done (2026-05-07) |
| Host-side static IP for Velodyne (`192.168.1.100/24`) via repo-tracked netplan template | done (2026-05-07) — config in place, link verification pending hardware |
| `tf_imus` ported to ament | superseded — replaced by `whill_sensors_bringup/launch/static_tf_launch.py` (placeholder identity TFs; calibration values still TODO) |
| `whill_sensors_bringup` package created | done (2026-05-07) — `ros2 launch whill_sensors_bringup sensors_launch.py` brings up all three sensors and the `base_link`-rooted static TF, including auto `configure → activate` for the IMU lifecycle node. See [`src/whill_sensors_bringup/README.md`](../src/whill_sensors_bringup/README.md). |
| Per-sensor topic verified on the bench | done (2026-05-07) — see [`m3-bench-data/README.md`](m3-bench-data/README.md) |
| TF tree verified | partial (2026-05-07) — RealSense subtree captured in [`m3-bench-data/frames-2026-05-07.pdf`](m3-bench-data/frames-2026-05-07.pdf); `velodyne` and `imu_link` still need a static parent (deferred to `whill_sensors_bringup`) |
| Per-sensor rosbag captured on the chair | done (2026-05-07) — `m3_chair_static_2026-05-07/` (655 MiB, 11.85 s, stationary) and `m3_chair_motion_2026-05-07/` (3.5 GiB, 64.5 s, joystick-driven). Both gitignored. See [`m3-bench-data/README.md`](m3-bench-data/README.md) for topic counts and the IMU race-condition fix that turned up during the chair bringup. |

## Velodyne network setup

The VLP-16 is reached over Ethernet rather than USB. The factory-default
LiDAR IP is `192.168.1.201`, so the host-side USB-Ethernet adapter needs
to be on the same `/24` subnet. Repo-tracked netplan template
[`network/01-velodyne-static.yaml.template`](../network/01-velodyne-static.yaml.template)
gives the host `192.168.1.100/24` on the chosen interface and disables
DHCP / gateway / DNS on that NIC (general internet stays on Wi-Fi).

```bash
ip -br link show | grep -E '^(enx|eth|enp)'         # find your USB-Ethernet iface
./scripts/install_velodyne_network.sh enx00e04c6808dc   # substitute and apply
```

The script renders `${IFACE}` from the template, installs the config to
`/etc/netplan/01-velodyne-static.yaml` (mode 600, root:root — netplan
rejects loose permissions on recent releases), and runs `netplan apply`.
Verify with `ping -c 3 192.168.1.201`.

If your VLP-16 was reprogrammed to a different subnet (Velodyne web UI
at the LiDAR's own IP), edit the addresses block in the template before
running the installer.

## Stable device paths (resolved 2026-05-07)

The IMU enumerates as a CDC-ACM device (`/dev/ttyACM*`), not a usb-serial
device (`/dev/ttyUSB*`), so its kernel numbering never collides with the
WHILL USB-serial port. To insulate launch files from `ttyUSB0/ttyACM0`
numbering anyway — and to make the stack reproducible after a fresh clone —
the udev rule [`udev/99-whill-stack.rules`](../udev/99-whill-stack.rules)
maps both devices to fixed symlinks by VID:PID:

| Device | VID:PID | Kernel name | Stable symlink |
|--------|---------|-------------|----------------|
| WHILL CR2 (PL2303) | `067b:2303` | `/dev/ttyUSB0` | `/dev/whill` |
| RT 9-axis IMU | `2b72:0003` | `/dev/ttyACM0` | `/dev/imu` |

Install with `./scripts/install_udev_rules.sh` (idempotent). Launch files
under `whill_sensors_bringup` should point at `/dev/whill` and `/dev/imu`,
not the underlying tty paths.

## Open questions

All previously-listed open questions resolved on 2026-05-07. The full
on-host smoke-test capture — topic rates, frame_ids, TF snapshot, and
rosbag2 details — lives in
[`m3-bench-data/README.md`](m3-bench-data/README.md).

Closed:

- ~~Whether RealSense D435 fits within the available USB 3 bandwidth alongside
  the WHILL USB-serial cable on the host's USB controllers.~~ — D435
  enumerates at SuperSpeed (5 Gbps) on Bus 2 of this host via a Genesys
  Logic USB3.1 hub; WHILL only consumes a Full-speed (12 Mbps) port on
  Bus 3, so they do not contend for bandwidth.
- ~~Confirm the IMU's actual published topic and remap to `/imu/data_raw` if
  the upstream driver publishes elsewhere.~~ — the upstream driver
  publishes `/imu/data_raw` directly, matching FAST-LIO's expected input.
  No remap needed. **Important nuance:** the driver is a `LifecycleNode`;
  plain `ros2 run` leaves it `unconfigured` with no topics. Bringup must
  drive `configure → activate` before subscribers see data.

## New observations from the smoke test

- **RealSense topic prefix is `/camera/camera/...`** (parent namespace
  `camera`, node name `camera`, both from `rs_launch.py` defaults).
  Downstream M4/M5 launches should remap or accept this prefix instead of
  the bare `/camera/`.
- **Velodyne driver uses sensor-data QoS** (best-effort). `ros2 topic hz`
  in humble does not auto-detect it — verify Velodyne liveness with
  `ros2 topic echo --once /velodyne_points` or with a sensor-data-QoS
  subscriber in code.
