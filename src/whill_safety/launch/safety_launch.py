"""M6R-3 lite safety layer: failsafe_node + twist_mux.

Included from m6r_bringup_launch.py during live operation. For bag-
replay verification (no live sensors, no twist_mux needed), run
failsafe_node directly:

    ros2 run whill_safety failsafe_node --ros-args -p use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('whill_safety')
    twist_mux_yaml = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Forwarded to failsafe_node and twist_mux. '
                        'Only relevant for bag replay via --clock.'),

        Node(
            package='whill_safety',
            executable='failsafe_node',
            name='failsafe_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_yaml, {'use_sim_time': use_sim_time}],
            # twist_mux publishes on `cmd_vel_out` by default; remap to the
            # shared /cmd_vel bus so Nav2's velocity_smoother (M6R-4) and
            # any other downstream consumer subscribe at the usual path.
            remappings=[('cmd_vel_out', '/cmd_vel')],
            output='screen',
        ),
    ])
