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
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument(
            'failsafe_layer_d_rescue',
            default_value='false',
            description='failsafe_node layer_d_rescue_enabled (Issue #161, '
                        'ADR-0013 proposed). true = while Layer D is the '
                        'only active layer, failsafe relays a fresh '
                        'reverse/rotate /cmd_vel_teleop command on '
                        '/cmd_vel_safety. Default off until the user '
                        'adopts ADR-0013.'),
        DeclareLaunchArgument(
            'failsafe_startup_grace_sec',
            default_value='0.0',
            description='failsafe_node startup_grace_sec (Issue #159). '
                        '0.0 = an input that never arrives is only logged '
                        '(DEAD INPUT). > 0 = after this many seconds, each '
                        'B/C/D input that has received nothing stops the '
                        'chair until its first message. Default off until '
                        'the user approves the change.'),

        Node(
            package='whill_safety',
            executable='failsafe_node',
            name='failsafe_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                # value_type=bool: the launch string must arrive as a bool,
                # not as the string 'true' that would fail the type check.
                'layer_d_rescue_enabled': ParameterValue(
                    LaunchConfiguration('failsafe_layer_d_rescue'),
                    value_type=bool),
                # value_type=float: a value typed as "10" would otherwise
                # reach the node as an integer and fail its double check.
                'startup_grace_sec': ParameterValue(
                    LaunchConfiguration('failsafe_startup_grace_sec'),
                    value_type=float),
            }],
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
