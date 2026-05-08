"""Launch FAST-LIO alone (no sensor drivers).

Use this for offline replay against a recorded rosbag — start the bag
with --clock and FAST-LIO will consume /velodyne_points + /imu/data_raw
from the bag's timestamps.

For live operation that also brings up the sensor drivers, see
`localization_launch.py` instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    whill_loc_share = get_package_share_directory('whill_localization')
    fast_lio_share = get_package_share_directory('fast_lio')

    # The FAST-LIO yaml is hardcoded rather than exposed through a
    # `config_file` LaunchArgument: when this launch is wrapped by another
    # via `IncludeLaunchDescription`, `LaunchConfiguration('config_file')`
    # silently resolves to an empty string and the params file is dropped
    # from the fastlio_mapping command line, causing the node to fall back
    # to upstream defaults (`lidar_type=AVIA`, Livox topics, ...) and
    # SIGSEGV when fed Velodyne PointCloud2 messages. Resolving the path
    # at launch-description generation time avoids that quirk.
    config_file = os.path.join(whill_loc_share, 'config', 'velodyne_whill.yaml')
    default_rviz = os.path.join(fast_lio_share, 'rviz', 'fastlio.rviz')

    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Spawn RViz2 with the FAST-LIO default layout.'),
        DeclareLaunchArgument(
            'rviz_cfg',
            default_value=default_rviz,
            description='RViz config file path.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='When true (default), the FAST-LIO node consumes '
                        'the /clock topic — set this to true for `ros2 bag '
                        'play --clock` replays. Override to false for live '
                        'sensor operation.'),

        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz_use),
            output='log',
        ),
    ])
