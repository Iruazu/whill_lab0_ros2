# whill_safety

Runtime safety layer for the WHILL chair — the M6-R / M7 / M9 destination
package for anything that keeps the chair from moving when it shouldn't.

M6R-2 (this iteration) ships the **bringup composition**: sensors +
WHILL driver + M4-R EKF + M6-R scan-to-map localizer, in one launch. M6R-3
will add the **failsafe node** (3-layer subscription to
`/reinitialization_requested`, `/alignment_status`, and `/pcl_pose`
continuity) plus a `twist_mux` gate on `/cmd_vel`, both landing inside
this same package. M9 will land the physical-E-stop and remote-stop
hookups here too.

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

`m6r_bringup_launch.py` already includes `odom_bringup_launch.py`, so the
minimum steady-state run is just steps 4 + 5 + 6 below. Steps 1-3 are an
*optional* pre-flight that verifies the sensor pipe alone before adding
the localizer on top; skip 1-3 if the sensors were already confirmed
healthy in a previous session.

**If you use the pre-flight (steps 1-3), Terminal A MUST be killed
before Terminal B is started** — running both in parallel violates the
mutual exclusion below (double publisher on `odom -> base_link`, TF
jitter, localizer fighting the doubled odom stream).

1. **[Optional pre-flight] Terminal A** (sensors + M4-R EKF, ~10 s to
   settle):
   ```
   ros2 launch whill_localization odom_bringup_launch.py
   ```
2. **[Optional pre-flight] Wait ~30 s**, verify in a fresh terminal:
   ```
   ros2 topic hz /velodyne_points     # ~10 Hz
   ros2 topic hz /imu/data_raw        # ~100 Hz (imu_sign_corrector then
                                      #   republishes as /imu/data_rep145)
   ros2 topic hz /whill/odom          # ~2.5 Hz
   ```
3. **[Optional pre-flight] Ctrl-C Terminal A**. Wait until the
   `ros2 launch` process fully exits (~2 s) before step 4 — leaving it
   half-shut-down is what trips the double-publisher case.
4. **Terminal B** (M6-R localizer + sensors + EKF, one command):
   ```
   ros2 launch whill_safety m6r_bringup_launch.py site:=campus
   ```
5. **Wait ~20 s**. Confirm the lifecycle transitioned:
   ```
   ros2 lifecycle get /lidar_localization    # active [3]
   ```
6. **Terminal C** (RViz): click **2D Pose Estimate** on the map. For a
   chair already positioned at the map's origin (`campus` map from
   工農研横 is this case), the identity pose (0, 0, 0) is correct;
   otherwise drag on the map. `/initialpose` publishes, the localizer
   converges within a few seconds, and `map -> odom` starts flowing.

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

`m6r_bringup_launch.py` **includes** `whill_localization/
odom_bringup_launch.py`. Running both in parallel would have
`robot_localization` publish `odom -> base_link` twice on the same TF
edge. Symptoms: TF listeners see unbounded jitter and the localizer's
scan-to-map correction fights the doubled odom stream.

`m6r_bringup_launch.py` **replaces** `whill_localization/
localization_launch.py` (FAST-LIO). Do not run both. FAST-LIO was
frozen as a runtime localizer by the 2026-06-11 platform pivot; it
survives in the repo for offline map-making only
(`docs/ja/plans/2026-06-11-platform-pivot.md` §5).

Effective operator rule: at any given moment, exactly one of the three
launches below is running:

- `whill_localization/odom_bringup_launch.py` — for M4-R-only debugging
  (odom stack without a map)
- `whill_localization/localization_launch.py` — for offline FAST-LIO
  map making (M5-R prerequisite; runs against a bag, no live vehicle)
- `whill_safety/m6r_bringup_launch.py` — full M6-R operation (live)

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
