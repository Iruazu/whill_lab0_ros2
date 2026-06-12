# Legacy repo index (`~/whill_lab0/`)

Language: [日本語](../ja/legacy-index.md) | [English](legacy-index.md)

The **entry-point map** to the legacy noetic implementation at `~/whill_lab0/`.
The `legacy-archaeologist` agent uses this as the starting point for its
investigations.

Glob-walking the legacy repo every time wastes the context window, so this
file accumulates a "where is what" outline. New findings from investigation
go into `docs/legacy-findings/<topic>.md` for detail; this file gets a 1–2
line headline pointing there.

## Legacy repo location

```
~/whill_lab0/
```

(If the location differs, update this file, the header of
`.claude/agents/legacy-archaeologist.md`, the relevant section of `CLAUDE.md`,
and `.claude/settings.json`.)

---

## Functional map

### Drive / wheel odometry
- Package: `ros_whill/`
- Main node: `ros_whill` (`ros_whill/src/ros_whill.cpp`)
- noetic upstream: `whill-labs/ros_whill`
- Notes:
  - Serial connection is selected via the `TTY_WHILL` environment variable
  - sub: `/whill/controller/joy` (`sensor_msgs/Joy`, axes[0]=yaw, axes[1]=forward, range ±100)
  - pub: `/whill/states/jointState`, `/odom`, `sensor_msgs/BatteryState`, tf `odom → base_link`
  - Speed profile: `ros_whill/params/initial_speedprofile.yaml`

### LiDAR / IMU / camera drivers
- Packages:
  - LiDAR: `velodyne-mast/`, `velodyne_pcl/` (Velodyne → `/velodyne_points`)
  - IMU: `rt_usb_9axisimu_driver/` (RT 9-axis IMU, `/dev/ttyACM0` → `/imu/data_raw`)
  - IMU tf: `tf_imus/` (`imu_link ↔ world`)
  - Camera: `realsense-ros/` (Intel RealSense)
  - Calibration: `velodyne_camera_calibration/`
  - Integrated driver wrapper: `sensor/`
- ROS 2 port target: `whill_sensors_bringup`
- Notes:
  - IMU standard deviations are configured via the `linear_acceleration_stddev`, `angular_velocity_stddev`, and `magnetic_field_stddev` parameters

### LiDAR-Inertial Odometry (FAST-LIO)
- Package: `whill_lab0/FAST_LIO/`
- Config: `whill_lab0/FAST_LIO/config/velodyne.yaml`
- ROS 2 side: `whill_localization`
- Notes:
  - Calibrated extrinsic values are already transcribed into this repo's `docs/en/m3-extrinsics-from-noetic.md`
  - Main node: `laserMapping` (`FAST_LIO/src/laserMapping.cpp`)
  - sub: `/velodyne_points`, `/imu/data` ← **the ROS 2 side uses `/imu/data_raw`; mind the topic name difference**
  - pub: `/integrated_to_init` (`nav_msgs/Odometry`)
  - Structure: iKD-Tree + EKF; IMU preprocessing in `IMU_Processing.hpp`
  - Main launch: `mapping_velodyne.launch` (`filter_size_surf=0.5`, `filter_size_map=0.5`, `cube_side_length=1000`)
    - Note: `cube_side_length=1000` overflows pcl::VoxelGrid's int32 indexing in this repo, so it has been reduced to 200
  - `loam_velodyne/` is also bundled, but the current operational line is FAST-LIO

### Autonomous driving (on-campus)
- Package: `loader_kiban/` (top-level orchestration)
- Main nodes:
  - `mapping_node` (`loader_kiban/src/mapping_node.cpp`)
    - in: `/integrated_to_init` → out: `/map` (`OccupancyGrid` 3000×3000, scale=20), `/localization/pose2d`
  - `path_planning_node` (`loader_kiban/src/pathplanning_node.cpp`)
    - **A\*** (diagonal cost aware) → `/path_planning/route` (`std_msgs/Float32MultiArray` `[x0,y0,x1,y1,...]`)
  - `motion_execution_node` (`loader_kiban/src/motion_execution_node.cpp`)
    - Pure-Pursuit-style follower → `/whill/controller/joy`
    - Parameters: `goal_tolerance=5.0`, `linear_speed=0.2`, `angular_gain=0.4`
- launch: `loader_kiban/launch/autonomous_navigation.launch`
- Related: global pose correction by `slam_localization/` (PCL NDT; `slam_localization_auto.cpp` / `ndt.cpp` / `gupndt.cpp` / `new_ndt.cpp`)
  - sub: `/velodyne_points`, `/map_cloud`, `/first_localization`, `/startslam`, `/endslam`
  - pub: `/second_localization`
  - Parameters: `ndt_leaf`, `ndt_epsilon`, `ndt_step_size`, `ndt_iteration`; scores are `ndt_final_score`, `ndt_final_accuracy`
- Notes: the NDT area has several backup sources (`*コピー.cpp`, `*バックアップ.cpp`) left over from work in progress. The mainline is `slam_localization_auto.cpp`.

### Driving assistance
- Package: (no standalone "assistance" package was found)
- Main node: `pedestrian_flow_navigator/` covers the equivalent function (collision avoidance) via a potential-field method
- Notes:
  - `relative_velocity/` and `position_to_velocity/` exist as peripheral functions (velocity estimation, Pose→Velocity conversion)

### Pedestrian flow / human detection
- Packages:
  - `pedestrian_flow_navigator/` (avoidance controller; **Potential Field** + **Lennard-Jones-type repulsion**)
    - sub: `/autoware_tracker/tracker/objects_world`, `/integrated_to_init`
    - pub: `/whill/controller/joy`; visualisation `/potential_field` (`visualization_msgs/MarkerArray`)
    - Key parameters: attraction `kv_=0.5`; repulsion `p_=2.0, q_=1.0, w_=7e-6`; sensitivity radius `rc_=7.0m`; robot radius 0.35 m; pedestrian radius 0.30 m; turn smoothing `sigma_WN_=π/20`
    - Goal coordinates are hard-coded as (199.4, 311.4) (externalise on port)
  - `autoware_tracker/` (IMM-UKF-PDA tracking → `/autoware_tracker/tracker/objects_world`)
    - `gating_thres=9.22`, `detection_probability=0.9`, `static_velocity_thres=0.5`
  - `ros_pede_movement/` (auxiliary pedestrian motion utilities)
- Notes:
  - The detection front end is `lidar_obstacle_detector/` (Euclidean clustering + BBox) plus `linefit_ground_segmentation/` (RANSAC ground separation)

### Navigation (move_base / navfn / etc.)
- Package: **standard stack not used**. `loader_kiban/` implements its own A\* + Pure Pursuit
- ROS 2 side: `whill_navigation` (Nav2 lifecycle)
- Notes:
  - On port, plan to replace the `loader_kiban` logic with Nav2's global planner / controller plugin
  - The existing `/map` is an OccupancyGrid, so feeding it into a Nav2 costmap is relatively straightforward
  - Both `motion_execution_node` (path following) and `pedestrian_flow_navigator` (avoidance) publish to `/whill/controller/joy`, so exclusive launch is assumed; the port should organise switching via a behavior tree

### Bringup / integrated launch
- Packages:
  - `loader_kiban/launch/autonomous_navigation.launch` — mapping + planning + motion in one shot
  - `FAST_LIO/launch/mapping_velodyne.launch` — LiDAR-Inertial Odometry
  - `ros_whill/launch/*` — WHILL driver
  - `slam_localization/launch/*` — NDT global localization
  - rviz: top-level `localiza_config.rviz`
- Notes: No single fully-integrated top-level launch was found. The likely operational pattern is starting each subsystem individually.

### Other / uncategorised
- `route/` — straight-line route interpolation tool
- `position_to_velocity/` — Pose → Velocity conversion (probably for Mocap / camera-based estimation)
- `relative_velocity/` — relative velocity computation
- `image_fps/` — image-streaming FPS measurement utility
- `ddynamic_reconfigure/` — dynamic_reconfigure helper library
- `catkin_simple/` — catkin macro library
- `reef_msgs/` — shared message definitions
- `Autoware/` — a subset of Autoware packages (`autoware_tracker` and friends depend on it)

---

## Cross-cutting signals for porting (synthesis)

Facts that surface from the functional map and bear directly on the porting plan:

1. **Topic convention deltas**: the legacy stack uses `/integrated_to_init` for odometry, `/whill/controller/joy` for joy commands, and `/imu/data` for the FAST-LIO IMU input. The ROS 2 side uses `/Odometry` / `/cmd_vel` / `/imu/data_raw`. Porting requires remaps or a rewrite.
2. **A global localizer already exists**: `slam_localization/`'s PCL NDT carries map-based localization against a prior point cloud (`/map_cloud`). This is the "layer that corrects FAST-LIO drift against the map" that the current ROS 2 stack lacks — and a strong candidate as the `map → odom` correction source during the Nav2 migration.
3. **Navigation is fully bespoke**: no move_base / Nav2. A\* + Pure Pursuit is hand-coded inside `loader_kiban`. Replacing it with Nav2's planner / controller plugins is the main work of the port.
4. **Exclusive-launch assumption**: `motion_execution_node` (path following) and `pedestrian_flow_navigator` (avoidance) both publish to `/whill/controller/joy`. The natural shape on Nav2 is exclusive switching via a behavior tree.
5. **Hard-coded values must be externalised**: the goal coordinates (199.4, 311.4), `goal_tolerance=5.0`, and the potential-field coefficients are written inline. Parameterise them on port.
6. **Map representation**: the legacy `/map` is an OccupancyGrid (3000×3000, scale=20). Feeding it into a Nav2 costmap is relatively easy.

---

## Items already investigated in detail

(Appended by `legacy-archaeologist` on completion.)

| Function | Detail file | Investigation date |
|----------|-------------|--------------------|
| — | — | — |

---

## Not worth porting / candidates for discard

- Backup sources inside `slam_localization/` (`*コピー.cpp`, `*バックアップ.cpp`, etc.) — remnants of work in progress
- `loam_velodyne/` — likely superseded by FAST-LIO and not worth porting (verify)
- `catkin_simple/`, `ddynamic_reconfigure/` — unnecessary under ROS 2 (replaced by ament / ros2 standards)
