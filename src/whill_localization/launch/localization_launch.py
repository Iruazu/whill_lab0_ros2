"""Live LiDAR-Inertial Odometry on the chair.

Composes the M3 sensor stack (`whill_sensors_bringup/sensors_launch.py`)
with FAST-LIO via `fast_lio_launch.py`. Use this when the chair is on,
sensors are mounted, and you want a real-time `/Odometry` and
`map -> odom` TF.

For offline replay use `fast_lio_launch.py` directly with `ros2 bag play
... --clock`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('whill_sensors_bringup')
    whill_loc_share = get_package_share_directory('whill_localization')

    return LaunchDescription([
        # All three sensors + base_link static TF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'sensors_launch.py'))),

        # FAST-LIO. Use real time (not sim time) when running live, so
        # override `use_sim_time` from the default `true` in
        # fast_lio_launch.py. RViz inherited from fast_lio_launch.py.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(whill_loc_share, 'launch', 'fast_lio_launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items()),
    ])
