# whill_navigation

Nav2 bringup for the WHILL chair.

This package is the M5 home base. It composes the Nav2 lifecycle nodes
the chair needs to follow a goal pose, on top of whatever produces the
`map -> odom -> base_link` TF chain.

Status as of 2026-06-20 (M4-R close): `tf_bridge_launch.py` has been
physically removed. The `map -> camera_init` identity it published was
the FAST-LIO-as-runtime-localizer shortcut from M5-a; the platform-pivot
plan (§5 禁止 1) requires it gone before the new architecture lands.
`nav_launch.py` is **intentionally left in a broken state** — its Nav2
nodes still expect a `map` frame, but no localizer is currently wired
in. M6-R will drop a scan-to-map localizer into the include slot and
restore working bringup. Until then this package provides only the Nav2
lifecycle node graph; do not expect `ros2 launch whill_navigation
nav_launch.py` to localise.

## TF tree check (M4-R)

To verify the M4-R `odom -> base_link -> {imu_link, velodyne,
camera_link}` chain without Nav2, use the unified odom bringup from
`whill_localization`:

```bash
ros2 launch whill_localization odom_bringup_launch.py
# in another terminal
ros2 run tf2_tools view_frames
```

The output `frames.pdf` should show `odom -> base_link` (published by
the `robot_localization` `ekf_filter_node`) and the three
`base_link -> sensor` static edges.

## Historical M5-a TF tree (removed)

For context, the M5-a `tf_bridge_launch.py` set up this tree. It is no
longer published by this package; the diagram is preserved as a record
of what M6-R needs to replace:

```
map                                       (whill_navigation, identity — removed)
└── camera_init                           (FAST-LIO, runtime — frozen as a localizer)
    └── body                              (FAST-LIO, runtime)
        └── base_link                     (whill_navigation, identity — removed)
            ├── imu_link                  (whill_sensors_bringup, M4R-2)
            ├── velodyne                  (whill_sensors_bringup, M4R-2)
            └── camera_link               (whill_sensors_bringup, M4R-2)
                ├── camera_depth_frame    (realsense2_camera)
                ├── camera_color_frame    (realsense2_camera)
                └── ...
```

The two identity hops (`map -> camera_init` and `body -> base_link`)
were structurally incapable of fixing P1/P2/P3 in the platform-pivot
diagnosis: a `map` frame fixed to the FAST-LIO start pose accumulates
all of FAST-LIO's drift directly into the world frame, and there was no
re-localization path. M6-R replaces both with a proper localizer.

## Open items / next sub-milestones

- **M6-R — scan-to-map localizer.** Pick between
  `lidar_localization_ros2` (NDT-OMP, default candidate per the
  platform-pivot plan §3.3) and alternatives, add the include in
  `nav_launch.py` at the marked slot, restore initial-pose UX.
- **M6-R — failsafe node.** Watch matching score / covariance, gate
  `cmd_vel` on divergence (§3.3 of the plan).
- **M6-R — obstacle layer + `use_collision_detection: true`.** Currently
  disabled because the M5-a map quality fed ghost obstacles into the
  costmap; depends on the M5-R map pipeline.

## Caveats

- The M5-d (goal-following) and M5-e (tuning) milestones are frozen
  per the platform-pivot plan. Do not add new features that assume
  the removed `tf_bridge_launch.py` exists.
- This README's `Open items` section reflects the M6-R plan; check
  `docs/ja/plans/2026-06-11-platform-pivot.md` §4 for the
  authoritative milestone definition before starting work.
