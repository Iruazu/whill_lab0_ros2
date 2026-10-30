"""Nav2 Collision Monitor for the avoidance baseline (C3).

Included by nav_launch.py ONLY when avoidance:=true. Kept as a separate
launch with its own lifecycle manager so nav_launch.py's Nav2 lifecycle
list, and therefore the avoidance:=false bringup, stays byte-identical.

cmd_vel chain with this launch active (official placement: after the
velocity smoother, https://docs.nav2.org/tutorials/docs/using_collision_monitor.html):

    twist_mux -> /cmd_vel -> velocity_smoother -> /cmd_vel_smoothed
              -> collision_monitor -> /whill/controller/cmd_vel -> whill_driver

If collision_monitor is not ACTIVE nothing reaches the driver, so a failed
start fails closed (the chair does not move) rather than bypassing the
monitor.

Parameters come from config/collision_monitor.yaml (rationale there). The
launch rewrites only topic names and use_sim_time into a per-run copy, the
same template -> temp yaml pattern as nav_launch.py (dict overrides next to
a yaml were unreliable on this host for map_server; avoid the question).

Standalone check (private ROS_DOMAIN_ID, no chair driver):

    ros2 launch whill_navigation collision_monitor_launch.py \\
        cmd_vel_out_topic:=/cm_test/cmd_vel_out
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    use_sim_time = arg('use_sim_time').lower() == 'true'
    with open(arg('params_file')) as f:
        params = yaml.safe_load(f)
    p = params['collision_monitor']['ros__parameters']
    p['use_sim_time'] = use_sim_time
    p['cmd_vel_in_topic'] = arg('cmd_vel_in_topic')
    p['cmd_vel_out_topic'] = arg('cmd_vel_out_topic')
    p['lidar']['topic'] = arg('cm_lidar_topic')
    p['depth']['topic'] = arg('cm_depth_topic')

    tmp = tempfile.NamedTemporaryFile(
        prefix='whill_collision_monitor_', suffix='.yaml', delete=False,
        mode='w')
    yaml.safe_dump(params, tmp)
    tmp.close()

    return [
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[tmp.name],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['collision_monitor'],
                # Same bond timeout as nav_launch's Nav2 manager.
                'bond_timeout': 4.0,
            }],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('whill_navigation')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'collision_monitor.yaml'),
            description='Collision Monitor parameter yaml.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true only for bag replay with --clock.'),
        DeclareLaunchArgument(
            'cmd_vel_in_topic', default_value='/cmd_vel_smoothed',
            description='Input: velocity_smoother output.'),
        DeclareLaunchArgument(
            'cmd_vel_out_topic', default_value='/whill/controller/cmd_vel',
            description='Output: WHILL driver input. Override for offline '
                        'tests so nothing can reach a driver.'),
        DeclareLaunchArgument(
            'cm_lidar_topic', default_value='/velodyne_points_hires_no_ground',
            description='Hi-Res-corrected ground-removed LiDAR cloud (C2).'),
        DeclareLaunchArgument(
            'cm_depth_topic', default_value='/camera/camera/depth/color/points',
            description='D435 depth point cloud.'),
        OpaqueFunction(function=_nodes),
    ])
