"""Ground removal preprocessing (M6R4-b).

Runs `patchworkpp_node` as an out-of-lifecycle process. Subscribes to
`/velodyne_points` (VLP-16, SensorDataQoS) and publishes to
`/velodyne_points_no_ground`, which downstream `pointcloud_to_laserscan`
(m6r/4-nav2-obstacle-layer PR #81) can be pointed at once #81 lands.

  ros2 launch whill_perception ground_removal_launch.py

Design note: the algorithm core is the BSD-2-Clause Patchwork++ C++
library from `src/third_party/patchwork_plusplus/cpp/`. Upstream's ROS 2
wrapper has an inconsistent license (ros/LICENSE MIT vs ros/package.xml
GPL-3.0) so we do not link it. This launch starts our own BSD-3-Clause
node instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('whill_perception')
    params_yaml = os.path.join(pkg_share, 'config', 'patchworkpp.yaml')

    return LaunchDescription([
        Node(
            package='whill_perception',
            executable='patchworkpp_node',
            name='patchworkpp_node',
            output='screen',
            parameters=[params_yaml],
            # Remap the wrapper's abstract topic names to the concrete
            # site topics. Downstream (M6R4-2 pointcloud_to_laserscan)
            # picks up /velodyne_points_no_ground once its input is
            # flipped from /velodyne_points.
            remappings=[
                ('cloud_in', '/velodyne_points'),
                ('cloud_no_ground', '/velodyne_points_no_ground'),
            ],
        ),
    ])
