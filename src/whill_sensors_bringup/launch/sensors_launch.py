"""Top-level M3 sensor bringup: VLP-16 + D435 + RT IMU + base_link static TF.

Composes the three Group A upstream drivers (velodyne / realsense2_camera /
rt_usb_9axisimu_driver) with this package's own lifecycle-aware IMU launch
and static TF chain. The result is a single launch that brings every
sensor up with no manual `ros2 lifecycle set` or per-node terminals.

After the launch settles, the following topics are expected:

  /velodyne_points                            sensor_msgs/PointCloud2  ~10 Hz
  /scan                                       sensor_msgs/LaserScan    ~10 Hz
  /imu/data_raw                               sensor_msgs/Imu          ~100 Hz
  /imu/mag                                    sensor_msgs/MagneticField~100 Hz
  /camera/camera/color/image_raw              sensor_msgs/Image        30 Hz
  /camera/camera/depth/image_rect_raw         sensor_msgs/Image        30 Hz

and the TF tree is rooted at `base_link`. See README.md for the diagram.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
        _include(bringup_share, 'launch', 'static_tf_launch.py'),
    ])
