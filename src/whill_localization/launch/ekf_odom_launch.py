"""Launch the M4-R odom-layer EKF (robot_localization).

Starts a single `ekf_node` from `robot_localization` configured by
`config/ekf_odom.yaml`. This node fuses `/whill/odom` (M4R-1) and
`/imu/data_raw` (M3) and publishes:

- `/odometry/filtered` (`nav_msgs/Odometry`) at 30 Hz
- `odom -> base_link` TF (the EKF is the *only* publisher of this edge)

Scope (what this launch does NOT start):

- Sensor / WHILL drivers. They are started by `whill_sensors_bringup` and
  the whill driver respectively. M4R-4 will provide an
  `odom_bringup_launch.py` that wires drivers + static TFs + this EKF
  into a single command.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    whill_loc_share = get_package_share_directory('whill_localization')

    # Resolve the params path at launch-description generation time, not
    # via LaunchConfiguration. When this launch is `IncludeLaunchDescription`'d
    # by a future top-level bringup (M4R-4), LaunchConfiguration substitutions
    # that were never re-declared at the outer level silently expand to an
    # empty string, and ekf_node would then fall back to its built-in
    # defaults (1 Hz output, no inputs fused). Same include-path quirk as
    # `odom_bringup_launch.py`.
    ekf_params = os.path.join(whill_loc_share, 'config', 'ekf_odom.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Set to true for `ros2 bag play --clock` replays. '
                        'Default false matches live chair operation; bag '
                        'replays of M3/M4 data should override to true.'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_params,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
