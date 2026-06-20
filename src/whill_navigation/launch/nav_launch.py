"""Top-level Nav2 bringup for the WHILL chair.

NOTE (M4-R close, 2026-06-20): This launch is intentionally left in a
broken state. M4R-4 / Issue #38 removed `tf_bridge_launch.py` because the
`map -> camera_init` identity it published violates the platform-pivot
plan (§5 禁止 1). The Nav2 nodes below still reference the bringup that
the old TF bridge enabled, so launching this file currently produces a
graph with no `map` frame author — Nav2 will start but not localise.

The fix is M6-R: replace the identity bridge with a real scan-to-map
localizer (`lidar_localization_ros2` or equivalent) that publishes
`map -> odom`. Until then this file is left visible (not renamed to
`.disabled`) so contributors trip on the breakage on purpose instead of
silently inheriting a stale dependency.

What stays composed below, untouched, so M6-R can drop the localizer in
without re-deriving the Nav2 wiring:

  - map_server + planner_server + controller_server + behavior_server
    + bt_navigator + velocity_smoother behind lifecycle_manager
  - velocity_smoother remaps `/cmd_vel_smoothed -> /whill/controller/cmd_vel`
    so the WHILL driver consumes the rate-limited stream directly.

cmd_vel routing (unchanged):
  controller_server  ─┐
                      ├─> /cmd_vel ─> velocity_smoother ─> /whill/controller/cmd_vel
  behavior_server    ─┘                              (remapped from /cmd_vel_smoothed)

velocity_smoother enforces real acceleration limits — RPP itself doesn't
ramp, so without the smoother the chair gets a 0 → desired_linear_vel
step which felt dangerous to a seated rider on the first M5-d run.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('whill_navigation')

    # Hardcode the params path at launch description build time, not via
    # LaunchConfiguration — substitutions into Node(parameters=[...])
    # resolve to empty string when this file is wrapped by
    # IncludeLaunchDescription. See docs/session-2026-05-08.md.
    nav2_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')

    # The saved map yaml is workspace-relative (not installed under any
    # package share), so allow override via the `map` launch arg.
    # Legacy M5-b path. M5R-5 (#47) renamed docs/m5-maps/ -> docs/maps/
    # lab-legacy-m5b/ to align with the new `docs/maps/<site>/` registry
    # (docs/maps/README.md). M5R-7 (#51) will re-aim this at the M5-R
    # pipeline output (`docs/maps/<site>/occupancy.yaml`). Kept pointed at
    # the legacy path until then because map_server resolves this default
    # at lifecycle CONFIGURE; switching it to a not-yet-existing M5-R
    # output before #51 would just trade one broken default for another.
    default_map_yaml = os.path.expanduser(
        '~/whill_lab0_ros2/docs/maps/lab-legacy-m5b/lab.yaml')

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

        # NOTE: No localization include here. The M5-a `tf_bridge_launch.py`
        # (map -> camera_init identity) was removed by M4R-4 / Issue #38.
        # M6-R is responsible for wiring a scan-to-map localizer that
        # publishes `map -> odom` and slotting its include statement at this
        # position.

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
