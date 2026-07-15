"""Sensor-layer bringup: VLP-16 + RT IMU + base_link static TF (+ RealSense opt-in).

**Scope**: this launch stands up the sensor drivers and the static TF
chain rooted at `base_link`. It is **transitively included** by
`whill_localization/odom_bringup_launch.py` and (via that) by
`whill_safety/m6r_bringup_launch.py`. Do **not** run this launch in
parallel with either — starting sensors_launch alongside odom_bringup
or m6r_bringup produces two velodyne drivers, two IMU nodes, two static
TF publishers, etc. (2026-07-16 field: measured
`/velodyne_points` at 39.4 Hz with a doubled bringup, RealSense USB
device contention).

**When to use standalone**: sensor-only smoke test (no localizer, no
Nav2, no WHILL driver). If you are bringing up the full M6-R stack,
run `whill_safety/m6r_bringup_launch.py` in ONE terminal and nothing
else at the sensor layer.

After the launch settles the topics are:

  /velodyne_points                            sensor_msgs/PointCloud2  ~10 Hz
  /scan_raw                                   sensor_msgs/LaserScan    ~10 Hz
  /imu/data_raw                               sensor_msgs/Imu          ~100 Hz
  /imu/mag                                    sensor_msgs/MagneticField~100 Hz

RealSense (opt-in) additionally publishes:

  /camera/camera/color/image_raw              sensor_msgs/Image        30 Hz
  /camera/camera/depth/image_rect_raw         sensor_msgs/Image        30 Hz

TF is rooted at `base_link`. See README.md for the diagram.

`/scan` remap note: upstream `velodyne-all-nodes-VLP16-launch.py` spins up
a `velodyne_laserscan_node` that publishes the full ring collapsed to 2D
on `/scan`. That output has no height filter, so it picks up ground
returns and ceiling returns that a Nav2 obstacle_layer would mark as
static hazards. The M6R4-2 chain uses `pointcloud_to_laserscan_node`
(nav_launch.py) with an explicit height band to feed `/scan` for the
obstacle_layer. To avoid a dual publisher on `/scan`, remap the upstream
velodyne_laserscan_node's output to `/scan_raw` here. No downstream
consumer exists for `/scan_raw` today; it stays around as a diagnostic
surface if we ever want the pre-filter view.

RealSense arg: off by default (`realsense:=false`). The D435 is not
consumed by the M6-R stack today, and past USB 2.1 enumeration issues
mean an accidentally-plugged D435 can burn a launch cycle even when the
data is not needed. Set `realsense:=true` only for camera-specific
smoke tests.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
# SetRemap lives under launch_ros.actions in humble (not launch.actions).
# Importing from launch.actions raises ImportError at launch time.
from launch_ros.actions import SetRemap


def _include(pkg_share, *path_parts):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, *path_parts))
    )


def generate_launch_description():
    velodyne_share = get_package_share_directory('velodyne')
    realsense_share = get_package_share_directory('realsense2_camera')
    bringup_share = get_package_share_directory('whill_sensors_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'realsense',
            default_value='false',
            description='Start the Intel RealSense D435 driver. Off by '
                        'default: the D435 is not consumed by the M6-R '
                        'runtime stack, and starting it while unused burns '
                        'a launch cycle when USB 2.1 enumeration fails or '
                        'another process holds the device. Flip true only '
                        'for camera-specific smoke tests.'),

        # GroupAction scopes SetRemap to just the velodyne include so the
        # /scan → /scan_raw rename does not affect anything else.
        GroupAction([
            SetRemap(src='/scan', dst='/scan_raw'),
            _include(velodyne_share, 'launch', 'velodyne-all-nodes-VLP16-launch.py'),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_share, 'launch', 'rs_launch.py')),
            condition=IfCondition(LaunchConfiguration('realsense')),
        ),
        _include(bringup_share, 'launch', 'imu_launch.py'),
        _include(bringup_share, 'launch', 'static_tf_launch.py'),
    ])
