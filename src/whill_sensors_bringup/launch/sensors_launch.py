"""Top-level M3 sensor bringup: VLP-16 + D435 + RT IMU + base_link static TF.

Composes the three Group A upstream drivers (velodyne / realsense2_camera /
rt_usb_9axisimu_driver) with this package's own lifecycle-aware IMU launch
and static TF chain. The result is a single launch that brings every
sensor up with no manual `ros2 lifecycle set` or per-node terminals.

After the launch settles, the following topics are expected:

  /velodyne_points                            sensor_msgs/PointCloud2  ~10 Hz  (raw)
  /velodyne_points_filtered                   sensor_msgs/PointCloud2  ~10 Hz  (M5-e self filter)
  /scan                                       sensor_msgs/LaserScan    ~10 Hz
  /imu/data_raw                               sensor_msgs/Imu          ~100 Hz
  /imu/mag                                    sensor_msgs/MagneticField~100 Hz
  /camera/camera/color/image_raw              sensor_msgs/Image        30 Hz
  /camera/camera/depth/image_rect_raw         sensor_msgs/Image        30 Hz

and the TF tree is rooted at `base_link`. See README.md for the diagram.

`velodyne_self_filter` strips mount / chair-body returns out of the raw
cloud before FAST-LIO consumes it. See scripts/velodyne_self_filter.py
for the rationale; defaults keep z >= -0.10 m in the velodyne frame.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(pkg_share, *path_parts):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, *path_parts))
    )


def generate_launch_description():
    velodyne_share = get_package_share_directory('velodyne')
    realsense_share = get_package_share_directory('realsense2_camera')
    bringup_share = get_package_share_directory('whill_sensors_bringup')

    return LaunchDescription([
        _include(velodyne_share, 'launch', 'velodyne-all-nodes-VLP16-launch.py'),
        _include(realsense_share, 'launch', 'rs_launch.py'),
        _include(bringup_share, 'launch', 'imu_launch.py'),
        # robot_state_publisher reads urdf/whill_with_sensors.urdf.xacro,
        # which includes the upstream WHILL CR2 chassis URDF and adds
        # imu_link / velodyne / camera_link / base_footprint with the
        # noetic-inherited extrinsic offsets. Replaces the old
        # static_tf_launch.py (identity placeholders).
        _include(bringup_share, 'launch', 'robot_state_publisher_launch.py'),
        Node(
            package='whill_sensors_bringup',
            executable='velodyne_self_filter.py',
            name='velodyne_self_filter',
            output='screen',
            parameters=[{
                'input_topic': '/velodyne_points',
                'output_topic': '/velodyne_points_filtered',
                # Cylinder layer — catches the near-LiDAR mount strut and
                # cables. Geometry derived from a 226-scan static bag (chair
                # seated still) analyzed by scripts/analyze_velodyne_arc.py.
                # The close-range radial histogram peaks sharply at r ≈ 1.0–1.1 m
                # with chair-body returns extending out to ~1.15 m. Z spans
                # roughly [-0.25, +0.25] m in the velodyne frame; the +0.2 m
                # ceiling deliberately includes seat-back / shoulder returns
                # so the cylinder swallows the whole chair envelope, not just
                # the mount strut. Predicted live kept-ratio (cylinder only):
                # ~85 %.
                'self_radius': 1.2,
                'self_z_min': -0.3,
                'self_z_max': 0.2,
                'stats_every_n': 100,
                # Forward-arc sectors — catch chair-body clusters that sit
                # OUTSIDE the cylinder at fixed bearings. A follow-up bag
                # (236 scans, 2026-05-28) showed a persistent arc at
                # r ∈ [1.35, 1.40] m with the azimuth concentrated on the
                # right side and the left front of the LiDAR — there is no
                # response at all in the forward ±30° sector, which is where
                # FAST-LIO actually needs the floor / wall returns. Expanding
                # the cylinder to r=1.55 m would have swallowed the arc but
                # only kept 75.6 % of points; surgical sectors do better
                # because they preserve the forward sweep. The ranges below
                # are tightened around the analyzer's z p5/p95 ([-0.262,
                # +0.346]) and the radial peak band [1.25, 1.50], with a
                # small margin so a real-world stamp wobble doesn't reopen
                # the gap. Live kept-ratio gets printed by velodyne_self_filter
                # every stats_every_n scans — if it falls below ~70 % during
                # navigation, narrow r_max from 1.50 → 1.45 first; if still
                # bad, narrow the azimuth ranges.
                # A/B (2026-05-29) confirmed the FAST-LIO failure was the
                # filtered-topic republish path itself, not the sectors:
                # even with sectors disabled (cylinder kept ~96 %) FAST-LIO
                # still hit VoxelGrid overflow + No Effective Points. We've
                # rolled lid_topic back to /velodyne_points in
                # whill_localization/config/velodyne_whill.yaml so FAST-LIO
                # consumes the raw stream. The sectors here remain enabled
                # so the /velodyne_points_filtered topic still carves out
                # the arc — RViz can A/B it against /velodyne_points to
                # confirm the geometry before we revisit the wiring once
                # the latency root cause is fixed.
                # Mirrors preprocess.self_sectors in
                # whill_localization/config/velodyne_whill.yaml so RViz
                # can A/B /velodyne_points vs /velodyne_points_filtered
                # against the same geometry FAST-LIO actually applies.
                # The outer right sector (r=1.85-2.15) was bisected out
                # 2026-05-29 because it starved FAST-LIO of registration
                # features — see the comment block in velodyne_whill.yaml.
                'forward_arc_enabled': True,
                'forward_arc_sectors': [
                    # Right chair-body cluster, INNER ring (r ≈ 1.0 m).
                    # az_min, az_max, r_min, r_max, z_min, z_max
                    '-125, -60, 1.20, 1.50, -0.30, 0.35',
                    # Left-front mount-arm cluster (r ≈ 1.0 m).
                    '25, 65, 1.20, 1.60, -0.30, 0.35',
                ],
            }],
        ),
    ])
