"""Standalone bringup for the WHILL wheel-odometry wrapper.

Subscribes /whill/states/model_cr2 (whill_msgs/ModelCr2State, emitted by
ros2_whill/whill_driver) and republishes /whill/odom (nav_msgs/Odometry)
so Phase A's ekf_odom can fuse wheel + IMU into the standard Nav2
odom -> base_link chain.

This launch does NOT start the whill_driver itself. The driver belongs to
whill_sensors_bringup (or the top-level whill_bringup once M6 lands); this
file is included from whill_localization/state_estimation_launch.py and
expects the driver to already be up.

YAML path is resolved at launch-description build time rather than via
LaunchConfiguration: same reason as fast_lio_launch.py / nav_launch.py —
when this file is wrapped by IncludeLaunchDescription, a LaunchConfiguration
substitution into a Node parameters list silently resolves to an empty
string and the node falls back to its declared defaults. Edit the installed
YAML and rebuild with --symlink-install if you need to override defaults.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('whill_odometry')
    params_yaml = os.path.join(pkg_share, 'config', 'whill_odometry.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Set true when consuming /clock from a `ros2 bag '
                        'play --clock` replay. Propagated to the '
                        'whill_odometry node so its internal now() reads '
                        'come from /clock rather than the wall clock.'),

        Node(
            package='whill_odometry',
            executable='odometry_node',
            name='whill_odometry',
            output='screen',
            parameters=[
                params_yaml,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
