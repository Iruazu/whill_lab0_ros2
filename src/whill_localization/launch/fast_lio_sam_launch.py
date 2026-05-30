"""FAST-LIO-SAM (loop-closure SLAM) bringup for the WHILL chair.

Wraps the upstream `fast_lio_sam/fastlio_mapping` executable with the
WHILL-specific VLP-16 config (`config/fast_lio_sam_velodyne.yaml`) so we
don't have to fork the third-party launch graph just to swap a yaml path.

The upstream `mapping_mid360.launch.py` does two extra things we replay
here verbatim:

  1. A hand-rolled `load_ros1_yaml_as_params()` flattener — FAST-LIO-SAM
     ships ROS 1 style nested yaml ("section: { key: value }") which
     ROS 2's parameter server does not auto-decompose; the helper turns
     it into "section/key: value" pairs the way rclcpp expects.
  2. Six FAST-LIO scalar params (`sam_enable`, `feature_extract_enable`,
     `point_filter_num`, `max_iteration`, `filter_size_surf`,
     `filter_size_map`, `cube_side_length`, `runtime_pos_log_enable`)
     that live OUTSIDE the yaml. Keep them aligned with the M5-d
     velodyne_whill.yaml values where they overlap, so the loop-closure
     SLAM and the stock FAST-LIO see the same surface scale.

Config path is resolved at launch-description build time (same
LaunchConfiguration-vs-IncludeLaunchDescription lesson as
`fast_lio_launch.py` and `state_estimation_launch.py`).
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _flatten(d, parent='', sep='/'):
    # ROS 1 style yaml uses nested mappings; ROS 2's parameter declaration
    # wants flat "ns/key" entries. Same algorithm as the upstream loader
    # — kept here so a future upstream change can't silently break us.
    out = {}
    for k, v in d.items():
        key = f"{parent}{sep}{k}" if parent else k
        if isinstance(v, dict):
            out.update(_flatten(v, key, sep))
        else:
            out[key] = v
    return out


def generate_launch_description():
    yaml_path = os.path.join(
        get_package_share_directory('whill_localization'),
        'config', 'fast_lio_sam_velodyne.yaml',
    )
    with open(yaml_path, 'r') as f:
        nested = yaml.safe_load(f)
    flat = _flatten(nested)

    # Mirror the upstream mid360 launch — values copied so behaviour
    # matches the reference unless we deliberately override. cube_side_length
    # is held at 1000.0 (upstream default) for now; revisit if the
    # int32-overflow signature comes back (see m4-localization.md / the
    # stock FAST-LIO yaml comment).
    fast_lio_overrides = [
        {'sam_enable': True},
        {'feature_extract_enable': False},
        {'point_filter_num': 3},
        {'max_iteration': 3},
        {'filter_size_surf': 0.5},
        {'filter_size_map': 0.5},
        {'cube_side_length': 1000.0},
        {'runtime_pos_log_enable': False},
        flat,
    ]

    rviz_cfg = os.path.join(
        get_package_share_directory('fast_lio_sam'),
        'rviz_cfg', 'sam_ros2.rviz',
    )

    return LaunchDescription([
        Node(
            package='fast_lio_sam',
            executable='fastlio_mapping',
            output='screen',
            parameters=fast_lio_overrides,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),
    ])
