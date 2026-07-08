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
ros2 launch whill_safety m6r_bringup_launch.py site:=campus-outdoor-corrected
```

`site` names a directory under `docs/maps/`. The launch resolves
`docs/maps/<site>/static.pcd` at launch time and injects it as the
localizer's `map_path`. To change the NDT tuning (score_threshold,
resolution, etc.), edit `config/m6r_lidar_localization.yaml`; to change
which map is loaded, pass a different `site`.

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
