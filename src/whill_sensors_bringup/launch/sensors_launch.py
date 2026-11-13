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

Velodyne calibration (#144): the unit is a VLP-16 **Puck Hi-Res**
(±10° vertical FOV, confirmed on the nameplate), but upstream
`velodyne-all-nodes-VLP16-launch.py` hard-codes `VLP16db.yaml` (±15°) for
the transform node and exposes no argument to change it. With the wrong
table every ring elevation is wrong and every height downstream (NDT,
Patchwork++, /scan, failsafe, GLIM) is distorted. We therefore start the
same three nodes here (same upstream param yamls, same
shutdown-on-driver-exit handler) and override only `calibration` via the
`velodyne_calibration` argument. Default is `VLP16_hires_db.yaml`; pass
`velodyne_calibration:=<share>/params/VLP16db.yaml` only to reproduce the
pre-#144 clouds (e.g. running against a map built before #144). Old
bags holding only `/velodyne_points` are corrected offline by
`scripts/velodyne_reproject_bag.py`, not by this argument. Values and provenance:
docs/calibration-ledger.md §2.

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
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg_share, *path_parts):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, *path_parts))
    )


def _velodyne_actions():
    """Mirror upstream velodyne-all-nodes-VLP16-launch.py with a selectable
    calibration table (#144). Upstream's param yamls are reused unchanged so
    only `calibration` differs from the upstream behaviour."""
    driver_share = get_package_share_directory('velodyne_driver')
    pointcloud_share = get_package_share_directory('velodyne_pointcloud')
    laserscan_share = get_package_share_directory('velodyne_laserscan')

    driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[os.path.join(
            driver_share, 'config', 'VLP16-velodyne_driver_node-params.yaml')])
    transform = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        # The later dict wins over the yaml's bare `VLP16db.yaml`. The node
        # opens that path as given (transform.cpp), so it must be absolute —
        # upstream's launch overrides it for the same reason.
        parameters=[
            os.path.join(pointcloud_share, 'config',
                         'VLP16-velodyne_transform_node-params.yaml'),
            {'calibration': LaunchConfiguration('velodyne_calibration')},
        ])
    laserscan = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='both',
        parameters=[os.path.join(
            laserscan_share, 'config',
            'default-velodyne_laserscan_node-params.yaml')],
        # /scan belongs to nav_launch's pointcloud_to_laserscan (see the
        # module docstring); keep this unfiltered ring on /scan_raw.
        remappings=[('/scan', '/scan_raw')])
    # Upstream shuts the whole launch down when the driver dies. Keep that:
    # a stack that silently runs without LiDAR would leave the localizer and
    # failsafe on stale data.
    on_driver_exit = RegisterEventHandler(OnProcessExit(
        target_action=driver,
        on_exit=[EmitEvent(event=Shutdown())]))
    return [driver, transform, laserscan, on_driver_exit]


def generate_launch_description():
    pointcloud_share = get_package_share_directory('velodyne_pointcloud')
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

        DeclareLaunchArgument(
            'velodyne_calibration',
            default_value=os.path.join(
                pointcloud_share, 'params', 'VLP16_hires_db.yaml'),
            description='Elevation table for velodyne_transform_node. The '
                        'unit is a VLP-16 Puck Hi-Res (+-10 deg), so the '
                        'default is VLP16_hires_db.yaml (#144). Pass '
                        '<velodyne_pointcloud share>/params/VLP16db.yaml '
                        'only to reproduce the pre-#144 (+-15 deg) clouds.'),

        *_velodyne_actions(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_share, 'launch', 'rs_launch.py')),
            condition=IfCondition(LaunchConfiguration('realsense')),
        ),
        _include(bringup_share, 'launch', 'imu_launch.py'),
        _include(bringup_share, 'launch', 'static_tf_launch.py'),
    ])
