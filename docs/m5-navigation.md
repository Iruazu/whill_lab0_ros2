# M5 — Autonomous navigation on ROS 2 humble

> **Status**: M5-d **done** (2026-05-20). First successful autonomous
> goal-to-pose on the WHILL CR2: 1.6 m forward goal reached cleanly
> in ~6 s, smooth 0 → 0.3 m/s ramp at 0.3 m/s² (velocity_smoother),
> `bt_navigator: Goal succeeded`. Closing conditions for M5 are met;
> M5-e (further tuning + long-distance goals + dynamic obstacles)
> remains optional.

## Goal

Drive the WHILL chair autonomously to a goal pose given in a known
map. Build on M4's `/Odometry` from FAST-LIO and M2's
`/cmd_vel` → WHILL motion path; insert Nav2 (or an equivalent ROS 2
navigation stack) in between.

End state of M5: `ros2 launch whill_navigation nav_launch.py`,
drop a goal in RViz at a 5 m / 10 m / hallway-end pose, the chair
plans, follows the path with real-time obstacle avoidance, and
arrives within tolerance.

## Scope (in M5)

- Bridge FAST-LIO's `camera_init -> body` TF into Nav2's expected
  `map -> odom -> base_link` chain.
- Build a 2D occupancy grid (or a 3D-aware costmap) of the lab area
  the chair will operate in. Sourced from a one-shot drive with
  FAST-LIO's PCD save enabled, converted offline.
- Author a `whill_navigation` package with a top-level
  `nav_launch.py` composing `whill_localization` + Nav2 lifecycle
  bringup against the saved map and chair-tuned `nav2_params.yaml`.
- Live validation on the chair: send a `geometry_msgs/PoseStamped`
  goal in RViz, watch Nav2 plan a path and drive there.
- Tune costmap inflation, planner cadence, and `controller_server`
  speed/accel limits to suit WHILL CR2 dynamics.

## Out of scope (deferred to M6)

- Pedestrian flow / dynamic obstacle prediction (would have been
  done by `pedestrian_flow_navigator` in the noetic stack)
- Goal sequencing, recovery behaviours beyond Nav2 defaults
- ROS bag replay-driven validation harness — M5 verification is
  hands-on on the chair

## Hardware → Nav2 inputs

| Source | Topic / TF | Notes |
|--------|-----------|-------|
| FAST-LIO | `/Odometry` (or `nav_msgs/Odometry` remap) | published as `camera_init -> body`; needs to be re-broadcast as `map -> odom -> base_link` |
| `whill_sensors_bringup` | `/tf_static` | the four `base_link → imu_link / velodyne / camera_link` static transforms |
| `velodyne_pointcloud` | `/velodyne_points` | feed the local costmap directly, or via a `pointcloud_to_laserscan` 2D slice |
| Saved map | `nav_msgs/OccupancyGrid` (`map_server`) | offline-converted from the FAST-LIO PCD or built from a Cartographer / SLAM Toolbox pass |
| WHILL driver | `geometry_msgs/Twist` on `/whill/controller/cmd_vel` (M2) | Nav2's `controller_server` publishes here |

## Procedure (planned, to be revised as work happens)

1. **TF bridge.** Either remap FAST-LIO's `camera_init -> body` to
   `map -> base_link` (treats FAST-LIO as the global localization)
   and let `whill_sensors_bringup`'s static TF carry the rest, or
   inject a small node that splits the fix into `map -> odom`
   (drift) plus `odom -> base_link` (instantaneous). Easier path:
   the first remap-only approach.

2. **Map building.** Flip
   `pcd_save.pcd_save_en: true` in
   `whill_localization/config/velodyne_whill.yaml`, drive a slow
   loop covering the test area. Save the resulting PCD under
   `docs/m5-maps/<env>.pcd`. Convert to an occupancy grid via
   [`scripts/pcd_to_occupancy_grid.py`](../scripts/pcd_to_occupancy_grid.py)
   (numpy-only, crops drift outliers and slices Z to chair-relevant
   obstacle band before rasterising).

3. **Add Nav2 to `whill_lab.repos`** if a forked Nav2 is needed —
   otherwise the apt `ros-humble-nav2-*` packages are enough.

4. **Author `whill_navigation` package** with a
   `launch/nav_launch.py` that includes
   `whill_localization/localization_launch.py` and brings up the
   Nav2 lifecycle stack (`map_server`, `planner_server`,
   `controller_server`, `bt_navigator`, `behavior_server`,
   `lifecycle_manager_navigation`) against a chair-tuned
   `config/nav2_params.yaml`.

5. **First live test.** RViz `2D Goal Pose` at a 5 m forward pose,
   confirm `controller_server` publishes `cmd_vel`, the chair moves,
   reaches the goal.

6. **Tuning loop.** Iterate on `nav2_params.yaml`:
   - costmap `inflation_layer.inflation_radius` for chair width
   - `controller_server.FollowPath.max_vel_x` to match WHILL Mode 2
   - `bt_navigator` recovery behaviours
   - planner cadence

## Status

| Step | Status |
|------|--------|
| Branch `m5/navigation` cut from main | done (2026-05-08) |
| Milestone doc stub authored | done (this file) |
| **M5-a — TF bridge** `map → camera_init → body → base_link → sensors` | **done (2026-05-08)** — `whill_navigation/launch/tf_bridge_launch.py` adds two static identities (`map → camera_init`, `body → base_link`); `tf2_tools view_frames` against a run2 replay confirms the full chain (snapshot in [`m3-bench-data/frames-m5a-2026-05-08.pdf`](m3-bench-data/frames-m5a-2026-05-08.pdf)). |
| `whill_navigation` package skeleton + `nav_launch.py` composer | done (2026-05-08) |
| **M5-b — Saved map (PCD)** | **done (2026-05-08)** — `docs/m5-maps/lab.pcd`, 256,478 points, 8.2 MB, captured by replaying `m4_chair_live_2026-05-08_run2` with `pcd_save.pcd_save_en: true` + `publish.map_en: true` and calling the `/map_save` service. **Caveat: includes scattered points from FAST-LIO drift segments** (XY range hits ±350 m even though the bounded trajectory stayed within ~15 m of origin). M5-c's PCD → occupancy-grid conversion must clip / outlier-filter aggressively before `nav2_map_server` can consume it. |
| **2D occupancy grid (.pgm + .yaml) for `nav2_map_server`** | **done (2026-05-20)** — [`scripts/pcd_to_occupancy_grid.py`](../scripts/pcd_to_occupancy_grid.py) converts `lab.pcd` → [`docs/m5-maps/lab.pgm`](m5-maps/lab.pgm) + [`docs/m5-maps/lab.yaml`](m5-maps/lab.yaml). Defaults: XY crop ±20 m, Z slice [0.1, 1.5] m, 0.05 m / cell → 800×800. Density outlier filter (5×5 window, min cluster 5) + 1.5 m clear-disk around origin remove people-pass spikes and chair self-returns. Bresenham ray-cast from origin marks free space; unreached cells stay unknown. Commits `3270336`, `53d691f`. |
| **`nav2_params.yaml` chair-tuned** | **done (2026-05-20)** — RPP with `desired_linear_vel: 0.3 m/s`, `use_collision_detection: false` (static map still has phantoms), `robot_radius: 0.45`, `inflation_radius: 0.5`. Canonical 47-plugin bt_navigator list. Commits `262772f`, `98e0c65`, `53d691f`, `8cd7fe5`. |
| **Nav2 lifecycle bringup in `nav_launch.py`** | **done (2026-05-20)** — `map_server` + `planner_server` + `controller_server` + `behavior_server` + `bt_navigator` + **`velocity_smoother`** + `lifecycle_manager_navigation`. velocity_smoother enforces real accel limits (0.3 m/s² fwd, 1.0 rad/s² rot) that RPP itself doesn't ramp; remap `/cmd_vel_smoothed → /whill/controller/cmd_vel`. Commits `262772f`, `8cd7fe5`. |
| RViz2 dry-run: map + costmaps render without the chair | superseded — went straight to live M5-d on the chair |
| **Live goal-following on the chair** | **done (2026-05-20)** — 1.6 m forward goal from (0.08, 0.03) → (1.59, 0.00). Begin → `Goal succeeded` in ~6 s. cmd_vel ramp 0 → 0.3 m/s in 1 s (matches `velocity_smoother.max_accel: 0.3`), no recovery behaviour fired, FAST-LIO stayed tracked the whole run. User feedback: "結構いい感じ". |
| Tuning notes captured | pending — M5-e (optional follow-up: longer goals, dynamic obstacle layer, re-enable collision detection on a re-captured clean map) |

## Open questions (to be answered as M5 progresses)

- Is FAST-LIO accurate enough to drive Nav2 on its own, or do we
  need to wrap it with AMCL on the saved map for global
  re-localization?
- 2D occupancy grid sufficient or do we need a 3D costmap (the
  chair / desk room has table legs Nav2's 2D scan would miss)?
- Should `cmd_vel` go directly to WHILL or via a velocity smoother
  to match the chair's actual dynamics?

## References

- M4 milestone doc + config: [`m4-localization.md`](m4-localization.md),
  [`../src/whill_localization/`](../src/whill_localization/).
- M2 WHILL driver topics: [`m2-whill-core.md`](m2-whill-core.md).
- TF tree from M3: [`m3-sensors.md`](m3-sensors.md) + the M4
  `camera_init -> body` from FAST-LIO.
