# Migration plan: noetic → humble

Source: `Iruazu/whill_lab0` (24 packages, ROS noetic). Target: `Iruazu/whill_lab0_ros2`
(this repo, ROS 2 humble).

## Package inventory and migration strategy

Packages are grouped by how they will be ported. **Group A** is replaced by official ROS 2
upstream packages, **Group B** is ported by hand, **Group C** is treated as a separate
project.

### Group A — replaced by official ROS 2 upstream

| noetic package | ROS 2 replacement | Notes |
|----------------|-------------------|-------|
| `ros_whill` | [`whill-labs/ros2_whill`](https://github.com/whill-labs/ros2_whill) | Official WHILL Inc. ROS 2 driver |
| `realsense-ros` | [`IntelRealSense/realsense-ros`](https://github.com/IntelRealSense/realsense-ros) (`ros2` branch) | Official Intel ROS 2 wrapper |
| `velodyne-mast`, `velodyne_pcl` | [`ros-drivers/velodyne`](https://github.com/ros-drivers/velodyne) (`ros2` branch) | Official Velodyne ROS 2 driver |
| `rt_usb_9axisimu_driver` | upstream `ros2` branch | Same vendor, ROS 2 branch |
| `FAST_LIO` | [`hku-mars/FAST_LIO`](https://github.com/hku-mars/FAST_LIO) (`ros2` branch) | Same authors |
| `linefit_ground_segmentation` | community ROS 2 fork | Verify with `colcon test` |
| `catkin_simple` | (drop) | Replaced by `ament_cmake` |
| `ddynamic_reconfigure` | (drop) | Replaced by ROS 2 dynamic parameters API |

### Group B — custom code to be ported by hand

Maintainers have not been handed over, so actual runtime usage of each package is unknown.
Strategy: leave these unported until a downstream package (e.g. a bringup launch file)
references them, then port on demand and document here.

| Package | Likely role | Port priority |
|---------|-------------|---------------|
| `autoware_tracker` | object tracking around Whill | low (Autoware-AI dependent) |
| `pedestrian_flow_navigator` | navigation around pedestrians | M5 |
| `ros_pede_movement` | pedestrian motion utilities | M5 |
| `slam_localization` | campus-specific localization | M4 |
| `route` | route generation / following | M5 |
| `sensor` | sensor utilities (TBD) | TBD |
| `tf_imus` | TF publishing for IMU | M3 |
| `loader_kiban` | base / loader (TBD) | TBD |
| `position_to_velocity` | derived velocity | M5 |
| `relative_velocity` | obstacle relative velocity | M5 |
| `image_fps` | image FPS measurement | low |
| `reef_msgs` | message defs (REEF) | as needed |
| `loam_velodyne` | older LiDAR odometry | drop (FAST-LIO supersedes) |
| `lidar_obstacle_detector` | obstacle detection | M5 |
| `velodyne_camera_calibration` | extrinsic calibration | utility (one-shot) |

### Group C — out of scope for direct port

- `Autoware/` — bundled noetic-era Autoware AI. Successor on ROS 2 is Autoware Universe,
  which is effectively a different project. Not ported in this repo.

## Branch / PR strategy

```
main
├─ m1/env-setup           ← ROS 2 humble installation, scripts, docs
├─ m2/whill-core          ← whill_ros2 driver + teleop bringup
├─ m3/sensors             ← Velodyne + RealSense + IMU
├─ m4/localization        ← FAST-LIO + slam_localization port
├─ m5/navigation          ← pedestrian flow, route, obstacle detection
└─ m6/bringup-integration ← top-level launch + on-vehicle validation
```

Each milestone branches from the latest `main`, is delivered as a single PR, and is merged
only when the relevant pieces are runnable on the real WHILL hardware (or, for M1, on the
host).
