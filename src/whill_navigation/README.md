# whill_navigation

Nav2 bringup for the WHILL chair.

This package is the M5 home base. It composes the M4 localization
launch with the TF wiring + Nav2 lifecycle nodes the chair needs to
follow a goal pose.

Status as of 2026-05-08: only the **TF bridge** (M5-a) is wired up.
Subsequent commits on `m5/navigation` will add map building (M5-b),
the Nav2 lifecycle launch (M5-c), and chair-tuned `nav2_params.yaml`
(M5-c / M5-e).

## Quick start

Live on the chair (FAST-LIO + TF bridge, no Nav2 yet):

```bash
ros2 launch whill_navigation nav_launch.py
```

Offline TF check against a recorded bag — start FAST-LIO + tf_bridge,
then play a bag in another terminal:

```bash
# terminal 1
ros2 launch whill_localization fast_lio_launch.py rviz:=false
ros2 launch whill_navigation tf_bridge_launch.py
# terminal 2
ros2 bag play <m4 motion bag> --clock \
    --topics /velodyne_points /imu/data_raw /imu/mag /tf_static
# terminal 3
ros2 run tf2_tools view_frames
```

## TF tree this package sets up

```
map                                       (whill_navigation, identity)
└── camera_init                           (FAST-LIO, runtime)
    └── body                              (FAST-LIO, runtime)
        └── base_link                     (whill_navigation, identity)
            ├── imu_link                  (whill_sensors_bringup, identity)
            ├── velodyne                  (whill_sensors_bringup, identity)
            └── camera_link               (whill_sensors_bringup, identity)
                ├── camera_depth_frame    (realsense2_camera)
                ├── camera_color_frame    (realsense2_camera)
                └── ...
```

The two new static identities (`map -> camera_init` and `body -> base_link`)
are aliases that connect FAST-LIO's coordinate convention to Nav2's
expectations without any custom pose-relay code. The `body` frame in
FAST-LIO is the IMU body frame, which is identity-equivalent to our
`imu_link` and (because the M3 static TF makes `base_link -> imu_link`
identity) to our `base_link`.

## Open items / next sub-milestones

- **M5-b — Build a saved map.** Drive a slow loop with FAST-LIO's
  `pcd_save_en` flipped to true, save the resulting PCD under
  `docs/m5-maps/<env>.pcd`, convert to a 2D occupancy grid for
  `nav2_map_server`. Or use `octomap_server` for 3D.
- **M5-c — Nav2 lifecycle bringup.** Add `map_server`,
  `planner_server`, `controller_server`, `bt_navigator`,
  `behavior_server`, `lifecycle_manager_navigation` to
  `nav_launch.py` against a chair-tuned `config/nav2_params.yaml`.
  Pull `ros-humble-nav2-*` apt packages and uncomment the Nav2
  exec_depends in `package.xml`.
- **M5-d — Live goal-following.** RViz `2D Goal Pose` → chair
  follows path → reaches goal.
- **M5-e — Tune costmaps and controller** to match WHILL CR2
  Mode 2 dynamics.

## Caveats baked into the M5-a baseline

- The `map -> camera_init` identity assumes FAST-LIO's drift over the
  test area is acceptable. For larger-scale operation, replace this
  identity with a proper global re-localizer (AMCL on a saved map,
  or a loop-closure-aware FAST-LIO variant), and split off the
  `odom` frame to be driven by WHILL wheel odometry.
- The `body -> base_link` identity is correct *only* because the M3
  `whill_sensors_bringup/launch/static_tf_launch.py` uses identity
  for `base_link -> imu_link`. If that changes (e.g. M5-b finds the
  IMU is offset enough from the chair centre to matter for path
  planning), this bridge needs the same offset reflected.
