"""Top-level Nav2 bringup for the WHILL chair.

Composition order:
  1. M4 localization (whill_localization/localization_launch.py) —
     sensors + FAST-LIO + RViz, producing /Odometry and the
     camera_init -> body TF.
  2. M5-a TF bridge (this package) — map -> camera_init and
     body -> base_link static transforms so Nav2 sees a Nav2-shaped
     TF chain.
  3. Nav2 lifecycle bringup (M5-c) — map_server + planner_server +
     controller_server + behavior_server + bt_navigator behind a
     lifecycle_manager that autostarts them in order.

cmd_vel routing:
  controller_server  ─┐
                      ├─> /cmd_vel ─> velocity_smoother ─> /whill/controller/cmd_vel
  behavior_server    ─┘                              (remapped from /cmd_vel_smoothed)

velocity_smoother enforces real acceleration limits — RPP itself doesn't
ramp, so without the smoother the chair gets a 0 → desired_linear_vel
step which felt dangerous to a seated rider on the first M5-d run.

For offline replay (no chair, no live FAST-LIO), use
`whill_localization/fast_lio_launch.py` separately and include only
this package's tf_bridge_launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    loc_share = get_package_share_directory('whill_localization')
    nav_share = get_package_share_directory('whill_navigation')

    # Hardcode the params path at launch description build time, not via
    # LaunchConfiguration — substitutions into Node(parameters=[...])
    # resolve to empty string when this file is wrapped by
    # IncludeLaunchDescription. See docs/session-2026-05-08.md.
    nav2_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')

    # The saved map yaml is workspace-relative (not installed under any
    # package share), so allow override via the `map` launch arg.
    default_map_yaml = os.path.expanduser(
        '~/whill_lab0_ros2/docs/m5-maps/lab.yaml')

    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Absolute path to the map yaml consumed by map_server.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'localization_launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'tf_bridge_launch.py'))),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params,
                        {'yaml_filename': LaunchConfiguration('map')}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
            # Publishes raw /cmd_vel; velocity_smoother picks it up and
            # produces the rate-limited stream the chair actually consumes.
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            # Smoother subscribes /cmd_vel (default) and publishes
            # /cmd_vel_smoothed. Remap the output straight to the WHILL
            # driver's input topic so we don't need a separate relay.
            remappings=[('/cmd_vel_smoothed', '/whill/controller/cmd_vel')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
            }],
        ),
    ])
