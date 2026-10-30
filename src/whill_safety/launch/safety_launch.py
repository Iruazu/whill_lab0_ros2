"""M6R-3 lite safety layer: failsafe_node + twist_mux.

Included from m6r_bringup_launch.py during live operation. For bag-
replay verification (no live sensors, no twist_mux needed), run
failsafe_node directly:

    ros2 run whill_safety failsafe_node --ros-args -p use_sim_time:=true

avoidance (docs/ja/plans/2026-10-28-avoidance-baseline.md §2.2, C3):

  false (default)  failsafe_node gets ONLY use_sim_time, exactly as before
                   2026-10-28, so every threshold is the node's built-in
                   default (Layer D sector ±30°, 1-2 m).
  true             Layer D becomes the rectangle x 1.0-2.0 m, |y| <= 0.5 m
                   so an obstacle beside the planned pass no longer vetoes
                   the avoidance manoeuvre (07-24 bag: 39-43 sector
                   engagements vs 3 rectangle ones). The detection inputs
                   of Collision Monitor are added to
                   extra_perception_topics: Humble's Collision Monitor
                   silently IGNORES a source older than source_timeout
                   (nav2_collision_monitor/src/source.cpp, humble branch,
                   Source::sourceValid) instead of stopping, so failsafe
                   is the component that stops when they go silent.

twist_mux is identical in both modes (priorities / timeouts untouched).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Avoidance-mode Layer D. Written out explicitly (not left to the node's
# defaults) so the launch file documents the geometry that was replayed
# against the 07-24 bag; forward_min_m / forward_max_m stay at the node
# defaults (1.0 / 2.0 m) because the plan keeps the same distance band.
_AVOIDANCE_FAILSAFE_PARAMS = {
    'layer_d_shape': 'rectangle',
    'layer_d_rect_half_width_m': 0.5,
}
# Detection clouds consumed by Collision Monitor under avoidance:=true
# (whill_navigation/config/collision_monitor.yaml). Keep in sync with the
# cm_lidar_topic / cm_depth_topic defaults of collision_monitor_launch.py.
_DEFAULT_WATCH_TOPICS = ('/velodyne_points_hires_no_ground,'
                         '/camera/camera/depth/color/points')


def failsafe_mode_params(avoidance, watch_topics_csv):
    """failsafe_node params beyond use_sim_time for the given mode.

    Pure function so test/test_safety_launch_params.py can pin that
    avoidance:=false adds nothing (= node defaults, the demo behaviour).
    """
    avoidance = str(avoidance).lower()
    if avoidance not in ('true', 'false'):
        raise RuntimeError(f'safety_launch: avoidance={avoidance!r} は true|false。')
    if avoidance == 'false':
        return {}
    params = dict(_AVOIDANCE_FAILSAFE_PARAMS)
    watch = [t.strip() for t in watch_topics_csv.split(',') if t.strip()]
    # An empty list would be typed BYTE_ARRAY by rclpy and rejected by the
    # node's STRING_ARRAY declaration; omit the key instead.
    if watch:
        params['extra_perception_topics'] = watch
    return params


def _failsafe_node(context):
    params = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    params.update(failsafe_mode_params(
        LaunchConfiguration('avoidance').perform(context),
        LaunchConfiguration('failsafe_extra_perception_topics').perform(context)))
    return [Node(
        package='whill_safety',
        executable='failsafe_node',
        name='failsafe_node',
        output='screen',
        parameters=[params],
    )]


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
            'avoidance',
            default_value='false',
            description='true: failsafe Layer D rectangle (x 1-2 m, |y|<=0.5 m) '
                        'and perception-silence watch on the Collision '
                        'Monitor inputs. false: failsafe defaults unchanged.'),
        DeclareLaunchArgument(
            'failsafe_extra_perception_topics',
            default_value=_DEFAULT_WATCH_TOPICS,
            description='Comma-separated PointCloud2 topics watched for '
                        'silence (stop after perception_timeout_s) when '
                        'avoidance:=true. Ignored when avoidance:=false.'),

        OpaqueFunction(function=_failsafe_node),
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
