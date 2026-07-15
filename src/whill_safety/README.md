# whill_safety

Runtime safety layer for the WHILL chair — the M6-R / M7 / M9 destination
package for anything that keeps the chair from moving when it shouldn't.

M6R-2 shipped the **bringup composition**: sensors + WHILL driver + M4-R
EKF + M6-R scan-to-map localizer, in one launch. M6R-3 lite (per
ADR-0007 §Demo-scope reduction) adds a minimal **failsafe_node**
(A layer: `/reinitialization_requested`; B layer: `/alignment_status`
fitness threshold + `/pcl_pose` silence watchdog) plus a `twist_mux`
gate on `/cmd_vel`, both inside this package. M9 will add the
physical-E-stop and remote-stop hookups here too.

## Runtime dependencies (apt)

```
sudo apt install -y \
  ros-humble-twist-mux \
  ros-humble-diagnostic-updater
```

`twist_mux` is the arbiter for the shared `/cmd_vel` bus; `safety_launch.py`
starts it and remaps its output. `diagnostic_updater` is a transitive
dependency of `twist_mux` but is called out explicitly here because of a
version-pinning trap: on hosts where apt updates have been held back for
a while, an older `diagnostic-updater` (≤ 4.0.6) can be installed alongside
a newer `twist_mux` and the ABI mismatch surfaces at runtime as
`twist_mux` **exiting immediately with code 127**
(missing symbol at dynamic-link time — no useful stderr in the launch
log). Installing both with the same `apt install` line above forces
them to the same repository snapshot and avoids the trap.

If `apt install` reports either package as already at the newest version
but `twist_mux` still exits 127, run `apt list --upgradable | grep -E
'twist-mux|diagnostic'` and follow up with `sudo apt upgrade` for those
packages specifically.

## Launching

```
ros2 launch whill_safety m6r_bringup_launch.py site:=campus
```

`site` names a directory under `docs/maps/`. The launch resolves
`docs/maps/<site>/static.pcd` at launch time and injects it as the
localizer's `map_path`. Default site is **`campus`** (the
2026-07-12 M6R-2 live acceptance map — 工農研横 origin). The
`campus-outdoor-corrected` map (7号館 発進、M6R-1 smoke, PR #74) is
still available via `site:=campus-outdoor-corrected` but is not the
default: launching that map on a chair positioned at 工農研横 will
reject every scan.

To change the NDT tuning (score_threshold, resolution, etc.), edit
`config/m6r_lidar_localization.yaml`; to change which map is loaded,
pass a different `site`.

## Boot sequence (operator)

**One bringup terminal only.** `m6r_bringup_launch.py` transitively
includes the sensor drivers, WHILL driver, M4-R EKF, localizer, and
safety layer. Running any additional bringup (`sensors_launch.py`,
`odom_bringup_launch.py`, ...) in parallel duplicates every node in
the subtree — see §Mutual exclusion below and the field measurement
(2026-07-16, `/velodyne_points` at 39.4 Hz under a doubled bringup).

Sensor pre-flight is NOT a separate launch — verify in-place after
step 1.

1. **Bringup terminal** (single command):
   ```
   ros2 launch whill_safety m6r_bringup_launch.py site:=campus
   ```
2. **Fresh terminal — verify no duplicate nodes** (mandatory before
   proceeding):
   ```
   ros2 node list | sort | uniq -c | sort -rn | head
   # every count MUST be 1. A "2 /velodyne_driver_node" line means a
   # duplicate bringup is running — kill the extra one before AC runs.
   ```
3. **Fresh terminal — sensor sanity** (once nodes are singletons):
   ```
   ros2 topic hz /velodyne_points     # ~10 Hz (a doubled bringup shows
                                      # ~20 Hz or higher — bail out and
                                      # re-check node list)
   ros2 topic hz /imu/data_rep145     # ~100 Hz (REP-145 corrected)
   ros2 topic hz /whill/odom          # ~2.5 Hz
   ros2 lifecycle get /lidar_localization    # active [3]
   ```
4. **RViz** (fresh terminal): click **2D Pose Estimate** on the map.
   For a chair already positioned at the map's origin (`campus` map
   from 工農研横 is this case), the identity pose (0, 0, 0) is
   correct; otherwise drag on the map. `/initialpose` publishes, the
   localizer converges within a few seconds, and `map -> odom` starts
   flowing.

To also start the D435 camera (opt-in — not consumed by the M6-R
stack today, USB 2.1 enumeration has bitten past sessions), append
`realsense:=true` to step 1's command line.

## DDS runtime configuration

`~/.bashrc` must point `CYCLONEDDS_URI` at the runtime xml:

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/systemlab/whill_lab0_ros2/configs/cyclonedds-runtime.xml
```

The runtime xml uses an **allow-list** of network interfaces (lo + LiDAR
wired NIC only) so that Wi-Fi / tethering / Docker bridges cannot
interfere with the `/velodyne_points` data path — the specific failure
mode that took 2 days of M6R-2 debugging to isolate (see
`docs/ja/plans/2026-06-24-m6r-localization.md` §10.2). The LiDAR NIC
name is a TODO in the xml; update it (`ip -brief link show`) the first
time you connect a live VLP-16.

For **bag recording** switch the current terminal only (leave
`~/.bashrc` on the runtime xml):

```
export CYCLONEDDS_URI=file:///home/systemlab/whill_lab0_ros2/configs/cyclonedds-bag-record.xml
ros2 daemon stop && ros2 daemon start
ros2 bag record /velodyne_points /imu/data_rep145 /whill/odom /tf
```

For bag replay use `scripts/m6r_smoke_test.sh` instead — it sets
`use_sim_time`, publishes `/initialpose` on a delay so it lands after
the bag's clock is live, and produces the ADR-0006 evidence bundle
(`docs/m6r-bench-data/<date>-smoke-<site>/`).

## Mutual exclusion — read before running

`m6r_bringup_launch.py` **transitively includes**:

```
m6r_bringup_launch.py
├── odom_bringup_launch.py           (whill_localization)
│   ├── sensors_launch.py            (whill_sensors_bringup)
│   │   ├── velodyne-all-nodes-VLP16-launch.py (with /scan → /scan_raw)
│   │   ├── rs_launch.py             (opt-in via realsense:=true, default off)
│   │   ├── imu_launch.py            (rt_usb_9axisimu_driver + imu_sign_corrector)
│   │   └── static_tf_launch.py      (base_link → imu_link/velodyne/camera_link)
│   ├── whill_launch.py              (whill_bringup — WHILL driver)
│   └── ekf_odom_launch.py           (whill_localization — M4-R EKF)
├── OpaqueFunction → lidar_localization.launch.py (M6-R scan-to-map localizer)
└── safety_launch.py                 (whill_safety — failsafe_node + twist_mux)
```

Running **any** of the launches inside that tree in parallel with
`m6r_bringup_launch.py` duplicates every node in the subtree.
Measured 2026-07-16 field with `sensors_launch.py` started
alongside: `/velodyne_points` at 39.4 Hz (4× normal), RealSense USB
device contention loop, doubled EKF / failsafe / lidar_localization /
twist_mux — AC4 could not be executed.

Effective operator rule: exactly one of the three launches below is
running at any moment.

- `whill_localization/odom_bringup_launch.py` — for M4-R-only debugging
  (odom stack without a map)
- `whill_localization/localization_launch.py` — for offline FAST-LIO
  map making (M5-R prerequisite; runs against a bag, no live vehicle)
- `whill_safety/m6r_bringup_launch.py` — full M6-R operation (live)

None of these compose with a separate `sensors_launch.py` — the sensor
stack is already inside each.

## Expected TF chain

```
map (lidar_localization_ros2, this package's include)
└── odom (ekf_filter_node, M4-R)
    └── base_link (EKF-integrated, smooth)
        ├── imu_link       (static, RPY 0, -8 deg pitch after PR #74)
        ├── velodyne       (static, PR #61 measurements)
        └── camera_link    (static, PR #74; target-based recal is a
                            post-demo item per Issue #70 archive)
            └── (realsense2_camera subtree)
```

Verify with `ros2 run tf2_tools view_frames` a few seconds after
launch settles.

## After the localizer configures / activates

The upstream launch drives the `lidar_localization` lifecycle node
through `configure -> active`, so `ros2 lifecycle get
/lidar_localization` should read `active [3]` a few seconds after
launch. Then:

1. In RViz, click **2D Pose Estimate** and drag on the map to set the
   initial pose. `/initialpose` publishes, the localizer converges,
   `map -> odom` starts publishing continuously.
2. `ros2 topic hz /pcl_pose` should read ~10 Hz.
3. `ros2 topic echo /alignment_status --once` should report
   `message: ok`, `has_converged: true`, `fitness_score < 6.0`.

If the pose does not converge, verify the map matches the physical
environment (`ros2 topic echo /map_points` in a fresh terminal will
show the loaded PCD; overlaying it against a live `/velodyne_points`
in RViz is the quickest sanity check).

## Files

```
whill_safety/
├── package.xml
├── CMakeLists.txt
├── config/
│   └── m6r_lidar_localization.yaml   NDT tuning (map_path filled at launch)
├── launch/
│   └── m6r_bringup_launch.py         Sensors + driver + EKF + localizer
├── whill_safety/                      Python package (empty at M6R-2,
│                                      failsafe_node lands at M6R-3)
│   └── __init__.py
└── README.md                          this file
```

## Related planning

- Parent phase plan: `docs/ja/plans/2026-06-24-m6r-localization.md`
  (§3.B for the whill_safety package rationale, §6 M6R-2 for the
  acceptance criteria of this launch)
- Parent strategy: `docs/ja/plans/2026-06-11-platform-pivot.md`
  (§3.5 for the safety-layer boundary rules)
- ADR-0006: `docs/ja/decisions/0006-localizer-choice.md` (the config
  yaml here reflects the NDT parameters pinned there)
