# SPDX-License-Identifier: BSD-3-Clause
"""Detection point clouds for the classical avoidance baseline (C2).

Started from whill_navigation/nav_launch.py only when `avoidance:=true`
(docs/ja/plans/2026-10-28-avoidance-baseline.md §2, §2.1). With
avoidance:=false nothing here runs, so the demo stack is unchanged.

    /velodyne_packets ─▶ velodyne_transform_node_hires (VLP16_hires_db.yaml)
                           └─▶ /velodyne_points_hires            [velodyne]
                                 └─▶ patchworkpp_node_hires (frame_id_override)
                                       └─▶ /velodyne_points_hires_no_ground   [velodyne_hires]
                                             └─▶ lidar_obstacle_filter
                                                   └─▶ /perception/lidar_obstacles_filtered [base_link]
    /camera/camera/depth/color/points ─▶ depth_obstacle_filter
                                           └─▶ /perception/depth_obstacles [base_link]

    static TF base_link ─▶ velodyne_hires (measured, plan §2.1)

Why a second transform node: the LiDAR is a VLP-16 Puck Hi-Res (±10°) but
the driver stack decodes it with VLP16db.yaml (±15°). Fixing that in place
moves the localizer, the map and every LiDAR threshold (plan §8), so the
existing driver, transform node, `velodyne` TF and Patchwork++ #1 stay as
they are and this parallel chain serves detection only. The existing
velodyne_driver_node (sensors_launch, via m6r_bringup) must be running; this
launch only subscribes to its /velodyne_packets.

The two `*_filter` nodes apply the obstacle height band in base_link (see
config/perception_3d.yaml for why not in STVL's map frame). STVL consumes the
filtered clouds; Collision Monitor (C3) can consume either.

After the LiDAR root fix (driver on VLP16_hires_db.yaml, measured
base_link->velodyne TF; plan §8) nothing here needs editing, only args:

    lidar_hires:=false                 # drop transform #2 and the velodyne_hires TF
    lidar_points:=/velodyne_points     # Patchwork++ #2 input
    lidar_frame:=velodyne              # frame_id_override (then a no-op relabel)
    lidar_no_ground:=<name>            # optional rename of the no-ground topic

nav_launch.py exposes the same four as perception_lidar_* and also points the
STVL sensor_frame at lidar_frame.
"""

import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# base_link -> velodyne_hires, plan §1.1 / §2.1 (2026-10-28): x, y by tape
# (twice, matches the old URDF), z / roll / pitch by a floor fit of the
# Hi-Res-reprojected cloud, yaw carried over from the noetic extrinsic (a
# floor fit cannot observe yaw). Same values as DEFAULT_LIDAR_TF in
# scripts/check_object_placement.py, which is the acceptance tool.
_HIRES_TF = dict(x=0.42, y=0.36, z=0.777,
                 roll_deg=0.24, pitch_deg=0.97, yaw_deg=-0.317)


def _nodes(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    share = get_package_share_directory('whill_perception')
    filter_yaml = os.path.join(share, 'config', 'perception_3d.yaml')

    # Start from upstream's VLP16 transform params so only the calibration
    # differs from the existing node (min_range 0.9, organize_cloud, ...).
    # Resolved here, not via substitutions, because nav_launch includes this
    # file from inside an OpaqueFunction.
    vp_share = get_package_share_directory('velodyne_pointcloud')
    with open(os.path.join(vp_share, 'config',
                           'VLP16-velodyne_transform_node-params.yaml')) as f:
        tparams = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    tparams['calibration'] = os.path.join(vp_share, 'params', 'VLP16_hires_db.yaml')
    tparams['use_sim_time'] = use_sim_time

    t = _HIRES_TF
    lc = LaunchConfiguration
    hires = IfCondition(lc('lidar_hires'))
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_velodyne_hires',
            arguments=[
                '--x', str(t['x']), '--y', str(t['y']), '--z', str(t['z']),
                '--roll', str(math.radians(t['roll_deg'])),
                '--pitch', str(math.radians(t['pitch_deg'])),
                '--yaw', str(math.radians(t['yaw_deg'])),
                '--frame-id', 'base_link', '--child-frame-id', 'velodyne_hires',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=hires,
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            # Distinct name: the existing node is `velodyne_transform_node`.
            name='velodyne_transform_node_hires',
            output='screen',
            parameters=[tparams],
            remappings=[('velodyne_points', '/velodyne_points_hires')],
            condition=hires,
        ),
        Node(
            package='whill_perception',
            executable='patchworkpp_node',
            name='patchworkpp_node_hires',
            output='screen',
            # lidar_frame after the yaml so the arg wins over the file.
            parameters=[os.path.join(share, 'config', 'patchworkpp_hires.yaml'),
                        {'use_sim_time': use_sim_time,
                         'frame_id_override': lc('lidar_frame').perform(context)}],
            remappings=[('cloud_in', lc('lidar_points').perform(context)),
                        ('cloud_no_ground', lc('lidar_no_ground').perform(context))],
        ),
        Node(
            package='whill_perception',
            executable='cloud_height_filter_node',
            name='lidar_obstacle_filter',
            output='screen',
            parameters=[filter_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('cloud_in', lc('lidar_no_ground').perform(context)),
                        ('cloud_out', '/perception/lidar_obstacles_filtered')],
        ),
        Node(
            package='whill_perception',
            executable='cloud_height_filter_node',
            name='depth_obstacle_filter',
            output='screen',
            parameters=[filter_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('cloud_in', LaunchConfiguration('depth_topic')),
                        ('cloud_out', '/perception/depth_obstacles')],
            condition=IfCondition(LaunchConfiguration('depth')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true only for bag replay with --clock.'),
        DeclareLaunchArgument(
            'lidar_hires', default_value='true',
            description='Interim (until the LiDAR root fix, plan §8): start the '
                        'second velodyne_transform_node with VLP16_hires_db.yaml '
                        '(/velodyne_points_hires) and the velodyne_hires static TF.'),
        DeclareLaunchArgument(
            'lidar_points', default_value='/velodyne_points_hires',
            description='Patchwork++ #2 input. /velodyne_points after the root fix.'),
        DeclareLaunchArgument(
            'lidar_frame', default_value='velodyne_hires',
            description='Frame of the no-ground cloud (frame_id_override) and the '
                        'LiDAR sensor_frame for STVL. velodyne after the root fix.'),
        DeclareLaunchArgument(
            'lidar_no_ground', default_value='/velodyne_points_hires_no_ground',
            description='Patchwork++ #2 output; also read by Collision Monitor '
                        '(cm_lidar_topic) and failsafe extra_perception_topics (C3).'),
        DeclareLaunchArgument(
            'depth', default_value='true',
            description='Start the D435 depth filter. false for LiDAR-only '
                        'replays (bags without depth).'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/camera/depth/color/points',
            description='D435 point cloud (config/realsense.yaml in '
                        'whill_sensors_bringup: untextured, 2x2 decimation).'),
        OpaqueFunction(function=_nodes),
    ])
